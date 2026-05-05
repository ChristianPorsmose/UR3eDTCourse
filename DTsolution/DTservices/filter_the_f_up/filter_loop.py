from collections import deque
from dataclasses import asdict
import threading
import numpy as np
from pathlib import Path
from communication.typed_protocol import FilteredState, PhysicalTwinState
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
import queue
from ParticleFilter import ParticleFilter


consumer_queue: queue.Queue[PhysicalTwinState] = queue.Queue()
publish_queue:  queue.Queue[FilteredState]      = queue.Queue()

ESTIMATION_WINDOW = 50
NUM_JOINTS        = 6
# Fixed based on observed encoder precision (~2e-6 rad from tracking error plot).
# Cannot be reliably estimated from the signal when the robot is moving.
MEASUREMENT_NOISE_STD = 1e-5

pf = ParticleFilter(num_particles=1000, dim=NUM_JOINTS)
measurement_window: deque[np.ndarray] = deque(maxlen=ESTIMATION_WINDOW)
timestamp_window:   deque[float]      = deque(maxlen=ESTIMATION_WINDOW)


def update_noise_estimates() -> None:
    """Adapt process noise to signal dynamics; measurement noise stays fixed."""
    arr    = np.array(measurement_window)   # (N, 6)
    diffs  = np.diff(arr, axis=0)           # (N-1, 6)

    mean_dt = max(float(np.diff(np.array(timestamp_window)).mean()), 1e-6)

    per_step_std = float(np.mean(np.std(diffs, axis=0)))
    proc_noise   = per_step_std / np.sqrt(mean_dt)

    pf.measurement_noise_std = MEASUREMENT_NOISE_STD
    pf.process_noise_std     = max(proc_noise, MEASUREMENT_NOISE_STD)


def filter_loop() -> None:
    while True:
        newest_state = consumer_queue.get()
        timestamp = newest_state.timestamp
        state_pos = np.array(newest_state.q_actual)
        state_vel = np.array(newest_state.qd_actual)

        measurement_window.append(state_pos)
        timestamp_window.append(timestamp)

        if not pf.is_initialized:
            pf.initialize(state_pos, state_vel, timestamp)
            continue

        if len(measurement_window) == ESTIMATION_WINDOW:
            update_noise_estimates()

        dt = timestamp - pf.old_time
        pf.old_time = timestamp

        if dt <= 0:
            continue

        pf.predict(dt, control_vel=state_vel)
        pf.update(state_pos)
        pf.resample()

        pf.particles[:, NUM_JOINTS:] = np.random.normal(
            state_vel, MEASUREMENT_NOISE_STD, (pf.num_particles, NUM_JOINTS)
        )

        pos_est, vel_est = pf.estimate()

        state_dict = asdict(newest_state)
        state_dict['q_actual']  = pos_est.tolist()
        state_dict['qd_actual'] = vel_est.tolist()
        publish_queue.put(FilteredState(**state_dict))


def main():
    config = load_config(Path("connect.yml"))
    print("STARTING PARTICLE FILTER")
    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        typed_client.subscribe(
            PhysicalTwinState,
            lambda msg: consumer_queue.put(msg),
            queue_name="filter_queue"
        )
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()
        threading.Thread(target=filter_loop, daemon=True).start()
        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
