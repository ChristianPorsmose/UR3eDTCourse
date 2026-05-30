import threading
import queue
from pathlib import Path

import numpy as np

from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import Calibrate, JointProgram, LoadProgram, Play, SurfaceViolation
from communication.typed_protocol_client import TypedRabbitMQClient
from models.ur3e import build_ur3e
from models.kinematic_model import KinematicModel

SURFACE_Z = 0.0
SIM_DT_S = 0.05

publish_queue: queue.Queue[SurfaceViolation] = queue.Queue()

ur3e = build_ur3e()
current_q = np.zeros(6)

def simulate_program(program: JointProgram, client: TypedRabbitMQClient) -> None:
    global current_q
    target_q = np.array(program.joint_positions)

    km = KinematicModel(
        max_velocity=np.deg2rad(program.max_velocity),
        max_acceleration=np.deg2rad(program.acceleration),
    )
    km.fmi2Instantiate()
    km.current_joint_angles = current_q.copy()
    km.fmi2SetCommandedJointAngles(target_q.tolist())
    km.fmi2StartMovement()
    sim_duration = km.movement_duration
    km.fmi2SetupExperiment(0.0, sim_duration)

    violating: set[int] = set()
    min_z: list[float] = []
    t = 0.0

    print(f"Simulating... t={t:.2f}/{sim_duration:.2f}s", end="\r", flush=True)

    while t <= sim_duration:
        km.fmi2DoStep(t, SIM_DT_S)
        t += SIM_DT_S

        q = np.array(km.fmi2GetJointPositions())
        step_z = [float(T.t[2]) for T in ur3e.fkine_all(q)]

        if not min_z:
            min_z = step_z[:]
        else:
            min_z = [min(min_z[i], step_z[i]) for i in range(len(step_z))]

        for i, z in enumerate(step_z):
            if z < SURFACE_Z:
                violating.add(i)

    violation = bool(violating)
    violating_sorted = sorted(violating)

    if violation:
        print(f"What-if violation: joints {violating_sorted}, min z={[f'{z:.3f}' for z in min_z]}")
        publish_queue.put(SurfaceViolation(
            violation_detected=True,
            violating_joints=violating_sorted,
            joint_z_positions=min_z,
        ))
    else:
        print("What-if simulation: clear — executing program")
        current_q = target_q
        client.publish(LoadProgram(
            joint_positions=program.joint_positions,
            max_velocity=program.max_velocity,
            acceleration=program.acceleration,
        ))
        client.publish(Play())


def main():
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as client:
        print("STARTING SURFACE DETECTION SERVICE (what-if mode)")

        def on_calibrate(msg: Calibrate):
            global current_q
            current_q = np.array(msg.joint_positions)

        client.subscribe(
            Calibrate,
            on_calibrate,
            "surface_det_calibrate",
        )
        client.subscribe(
            JointProgram,
            lambda program: simulate_program(program, client),
            "surface_det_program",
        )

        threading.Thread(
            target=lambda: typed_publisher_loop(client, publish_queue),
            daemon=True,
        ).start()

        client.client.start_consuming()


if __name__ == "__main__":
    main()
