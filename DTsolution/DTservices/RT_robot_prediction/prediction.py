from enum import Enum, auto
from pathlib import Path
import time
import threading
from queue import Queue

from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import (
    LoadProgram,
    Play,
    Calibrate,
    KinematicModelState,
    CtrlMsg
)
from communication.typed_protocol_client import TypedRabbitMQClient
from DTsolution.models.kinematic_model import KinematicModel


class State(Enum):
    IDLE = auto()
    WAITING_FOR_PLAY = auto()
    PLAY_BUFFERED = auto()


model_lock = threading.Lock()
state = State.IDLE

ctrl_queue: Queue[CtrlMsg | Calibrate] = Queue()
publish_queue: Queue[KinematicModelState] = Queue()


def inject_ctrl_msg_to_model(model: KinematicModel, msg: CtrlMsg | Calibrate):
    global state

    with model_lock:
        match (state, msg):
            case (_, Calibrate()):
                state = State.IDLE
                print("Calibration applied.", flush=True)
                model.current_joint_angles = msg.joint_positions
                
            case (State.PLAY_BUFFERED, LoadProgram()):
                model.fmi2SetCommandedJointAngles(msg.joint_positions)
                model.fmi2StartMovement()
                state = State.IDLE
                print("Executing buffered Play command...", flush=True)

            case (_, LoadProgram()):
                model.fmi2SetCommandedJointAngles(msg.joint_positions)
                state = State.WAITING_FOR_PLAY
                print("Program loaded successfully.", flush=True)

            case (State.WAITING_FOR_PLAY, Play()):
                model.fmi2StartMovement()
                state = State.IDLE

            case (_, Play()):
                state = State.PLAY_BUFFERED
                print("Play arrived early! Buffering until LoadProgram arrives.", flush=True)


def run_simulation(model: KinematicModel, dt: float = 0.05):
    i = 0

    while True:
        start_time = time.time()

        while not ctrl_queue.empty():
            msg = ctrl_queue.get()
            inject_ctrl_msg_to_model(model, msg)

        with model_lock:
            current_time = i * dt
            model.fmi2DoStep(current_time, dt)

            pos = [float(x) for x in model.fmi2GetJointPositions()]
            vel = [float(x) for x in model.fmi2GetJointVelocities()]

            state_msg = KinematicModelState(
                robot_mode="RUNNING" if model.moving else "IDLE",
                q_actual=pos,
                qd_actual=vel,
                q_target=pos,
                joint_max_speed=[60.0] * len(pos),
                joint_max_acceleration=[80.0] * len(pos),
                tcp_pose=[0.0] * 6
            )

        publish_queue.put(state_msg)

        elapsed_time = time.time() - start_time
        time.sleep(max(0, dt - elapsed_time))
        i += 1


def main():
    config = load_config(Path("connect.yml"))
    model = KinematicModel()

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING KINEMATIC MODEL SERVICE")

        typed_client.subscribe(LoadProgram, ctrl_queue.put, queue_name="model_load_program")
        typed_client.subscribe(Play, ctrl_queue.put, queue_name="model_play")
        typed_client.subscribe(Calibrate, ctrl_queue.put, queue_name="model_calibrate")

        threading.Thread(
            target=lambda: typed_publisher_loop(typed_client, publish_queue),
            daemon=True
        ).start()

        threading.Thread(
            target=lambda: run_simulation(model),
            daemon=True
        ).start()

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
