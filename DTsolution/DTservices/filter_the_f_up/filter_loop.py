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


consumer_queue : queue.Queue[PhysicalTwinState] = queue.Queue()
kinematic_queue = deque(maxlen=20)
publish_queue : queue.Queue[FilteredState] = queue.Queue()

PROCESS_NOISE_STD = 6.310483282548284e-06  # TODO : estimate
MEASUREMENT_NOISE_STD = 6.310483282548284e-06 # TODO : estimate

pf = ParticleFilter(num_particles=1000, process_noise_std=PROCESS_NOISE_STD, measurement_noise_std=MEASUREMENT_NOISE_STD)

def filter_loop():
    while True:
        newest_state = consumer_queue.get()
        timestamp = newest_state.timestamp
        state_pos = np.array(newest_state.q_actual)
        state_vel = np.array(newest_state.qd_actual)

        if not pf.is_initialized:
            pf.initialize(state_pos, state_vel, timestamp)
            continue

        dt = timestamp - pf.old_time
        pf.old_time = timestamp

        if dt <= 0:
            continue

        pf.predict(dt)
        pf.update(state_pos)
        pf.resample()

        pos_est, vel_est = pf.estimate()

        publish_queue.put(
            FilteredState(
            **asdict(newest_state),
            q_actual=pos_est.tolist(),
            qd_actual=vel_est.tolist()
        )
        )       

def main():
    config = load_config(Path("connect.yml"))
    print("STARTING PARTICLE FILTER")
    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        typed_client.subscribe(
                PhysicalTwinState,
                lambda msg : consumer_queue.put(msg),
                queue_name="filter_queue"
            )
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()
        threading.Thread(target=filter_loop, daemon=True).start()
        typed_client.client.start_consuming()

if __name__ == "__main__":
    main()