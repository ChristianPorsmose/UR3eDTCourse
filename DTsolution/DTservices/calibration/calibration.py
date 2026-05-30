from collections import deque
import threading
import time
import numpy as np
from pathlib import Path

from communication.typed_protocol import (
    PhysicalTwinState,
    KinematicModelState,
    Calibrate,
)
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
import queue

# Minimum per-joint offset (rad) to bother calibrating.
CALIBRATION_THRESHOLD = 0.005
# Minimum seconds between successive calibrations.
CALIBRATION_COOLDOWN = 10.0
# Joint velocity (rad/s) below which a joint is considered idle.
VELOCITY_THRESHOLD = 0.01
# How often the calibration loop wakes up to check state.
POLL_INTERVAL = 0.5

# Holds the single most-recent message from each source.
# deque(maxlen=1) gives thread-safe last-value semantics via the GIL.
latest_pt: deque[PhysicalTwinState] = deque(maxlen=1)
latest_km: deque[KinematicModelState] = deque(maxlen=1)
publish_queue: queue.Queue = queue.Queue()


def _is_idle(state: PhysicalTwinState | KinematicModelState) -> bool:
    return state.robot_mode.upper() == "IDLE"


def calibration_loop() -> None:
    last_calibration_time = 0.0

    while True:
        time.sleep(POLL_INTERVAL)

        if not latest_pt or not latest_km:
            continue

        pt = latest_pt[-1]
        km = latest_km[-1]

    # print robot status: 
        print(f"[DEBUG] phys {pt.robot_mode} kin {km.robot_mode} ", flush=True)

        print(
            f"[Calibration Loop] pt q_actual={pt.q_actual} km q_actual={km.q_actual}",
            flush=True,
        )

        now = time.time()

        if not (_is_idle(pt) and _is_idle(km)):
            continue

        if now - last_calibration_time < CALIBRATION_COOLDOWN:
            continue

        max_offset = float(np.max(np.abs(np.array(pt.q_actual) - np.array(km.q_actual))))

        if max_offset < CALIBRATION_THRESHOLD:
            continue

        print(
            f"[Calibration] Max joint offset {max_offset:.4f} rad — aligning kinematic model",
            flush=True,
        )

        max_vel = float(np.max(pt.joint_max_speed)) if pt.joint_max_speed else 60.0
        max_acc = float(np.max(pt.joint_max_acceleration)) if pt.joint_max_acceleration else 80.0

        publish_queue.put(Calibrate(
            joint_positions=list(pt.q_actual),
            max_velocity=max_vel,
            acceleration=max_acc,
        ))

        last_calibration_time = now


def main():
    config = load_config(Path("connect.yml"))
    print("STARTING CALIBRATION SERVICE", flush=True)

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        typed_client.subscribe(
            PhysicalTwinState, latest_pt.append, "calibration_pt_queue"
        )
        typed_client.subscribe(
            KinematicModelState, latest_km.append, "calibration_km_queue"
        )

        threading.Thread(target=calibration_loop, daemon=True).start()
        threading.Thread(
            target=lambda: typed_publisher_loop(typed_client, publish_queue),
            daemon=True,
        ).start()

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
