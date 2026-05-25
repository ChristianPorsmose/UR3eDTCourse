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


model_lock = threading.Lock()

ctrl_queue: Queue[CtrlMsg | Calibrate] = Queue()
publish_queue: Queue[KinematicModelState] = Queue()


# Update your state tracker to include a buffer flag
dt_state = {
    "is_program_loaded": False,
    "play_pending": False  # <-- NEW: Remembers if Play arrived early
}

def inject_ctrl_msg_to_model(model: KinematicModel, msg: CtrlMsg | Calibrate):
    global dt_state
    
    with model_lock:
        if isinstance(msg, LoadProgram):
            # 1. Load the program
            model.fmi2SetCommandedJointAngles(msg.joint_positions)
            dt_state["is_program_loaded"] = True
            print("Program loaded successfully.", flush=True)

            # 2. Immediately check if a Play command is waiting for this
            if dt_state["play_pending"]:
                print("Executing buffered Play command...", flush=True)
                model.fmi2StartMovement()
                
                # Reset states
                dt_state["play_pending"] = False
                dt_state["is_program_loaded"] = False 

        elif isinstance(msg, Calibrate):
            model.fmi2SetCommandedJointAngles(msg.joint_positions)
            model.fmi2StartMovement()
            dt_state["is_program_loaded"] = False
            dt_state["play_pending"] = False
            print("Calibration applied.", flush=True)

        elif isinstance(msg, Play):
            # 1. If program isn't loaded yet, hold onto this command!
            if not dt_state["is_program_loaded"]:
                print("Play arrived early! Buffering until LoadProgram arrives.", flush=True)
                dt_state["play_pending"] = True
                return
            
            # 2. Otherwise, execute normally
            model.fmi2StartMovement()
            
            # Reset state
            dt_state["is_program_loaded"] = False


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

            state = KinematicModelState(
                robot_mode="RUNNING" if model.moving else "IDLE",
                q_actual=pos,
                qd_actual=vel,
                q_target=pos,
                joint_max_speed=[60.0] * len(pos),
                joint_max_acceleration=[80.0] * len(pos),
                tcp_pose=[0.0] * 6
            )

        publish_queue.put(state)

        elapsed_time = time.time() - start_time
        time.sleep(max(0, dt - elapsed_time))
        i += 1


def main():
    config = load_config(Path("connect.yml"))
    model = KinematicModel()

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING KINEMATIC MODEL SERVICE")

        typed_client.subscribe(
            LoadProgram,
            ctrl_queue.put,
            queue_name="model_load_program"
        )

        typed_client.subscribe(
            Play,
            ctrl_queue.put,
            queue_name="model_play"
        )

        typed_client.subscribe(
            Calibrate,
            ctrl_queue.put,
            queue_name="model_calibrate"
        )

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