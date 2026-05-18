import time
import threading
import queue
from pathlib import Path

import numpy as np

from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import PhysicalTwinState, SurfaceViolation
from communication.typed_protocol_client import TypedRabbitMQClient
from models.ur3e import build_ur3e

SURFACE_Z = 0.0     # meters — links with z below this are flagged
CHECK_INTERVAL = 0.5

publish_queue: queue.Queue[SurfaceViolation] = queue.Queue()
latest_state: PhysicalTwinState | None = None

ur3e = build_ur3e()


def on_state(state: PhysicalTwinState) -> None:
    global latest_state
    latest_state = state


def check_surface() -> None:
    if latest_state is None:
        return

    q = np.array(latest_state.q_actual)
    transforms = ur3e.fkine_all(q)

    violating: list[int] = []
    z_positions: list[float] = []

    for i, T in enumerate(transforms):
        z = float(T.t[2])
        z_positions.append(z)
        if z < SURFACE_Z:
            violating.append(i)

    violation = len(violating) > 0
    if violation:
        print(f"Surface violation: links {violating}, z={[f'{z:.3f}' for z in z_positions]}")

    publish_queue.put(SurfaceViolation(
        violation_detected=violation,
        violating_joints=violating,
        joint_z_positions=z_positions,
    ))


def detection_loop() -> None:
    while True:
        try:
            check_surface()
        except Exception as e:
            print(f"Surface check error: {e}")
        time.sleep(CHECK_INTERVAL)


def main():
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as client:
        print("STARTING SURFACE DETECTION SERVICE")
        client.subscribe(PhysicalTwinState, on_state, "surface_det_pt")

        threading.Thread(target=detection_loop, daemon=True).start()
        threading.Thread(
            target=lambda: typed_publisher_loop(client, publish_queue),
            daemon=True,
        ).start()

        client.client.start_consuming()


if __name__ == "__main__":
    main()
