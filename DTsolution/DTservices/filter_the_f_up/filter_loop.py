from collections import deque
import threading
import numpy as np
from pathlib import Path

from communication.protocol import ROUTING_KEY_STATE
from communication.protocol import RobotArmStateKeys as rb
from utils.utils import load_config, publisher_loop
from communication.rabbitmq import ROUTING_KEY_FILTERED_STATE, Rabbitmq
import queue
from ParticleFilter import ParticleFilter

consumer_queue = queue.Queue()
kinematic_queue = deque(maxlen=20)
publish_queue = queue.Queue()

PROCESS_NOISE_STD = 6.310483282548284e-06  # TODO : estimate
MEASUREMENT_NOISE_STD = 6.310483282548284e-06 # TODO : estimate

pf = ParticleFilter(num_particles=1000, process_noise_std=PROCESS_NOISE_STD, measurement_noise_std=MEASUREMENT_NOISE_STD)

def filter_loop():
    while True:
        newest_state = consumer_queue.get()
        timestamp = newest_state[rb.TIMESTAMP]
        state_pos = np.array(newest_state[rb.Q_ACTUAL])
        state_vel = np.array(newest_state[rb.QD_ACTUAL])

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
        msg = {
            rb.TIMESTAMP: timestamp,
            rb.Q_ACTUAL: pos_est.tolist(),
            rb.QD_ACTUAL: vel_est.tolist()
        }
        publish_queue.put(msg)
        

def main():
    config = load_config(Path("connect.yml"))
    print("STARTING PARTICLE FILTER")
    with Rabbitmq(**config) as rabbit_mq:
        rabbit_mq.subscribe(
                ROUTING_KEY_STATE,
                lambda _, __, ___, body : consumer_queue.put(body),
                queue_name="filter_queue"
            )
        threading.Thread(target=lambda: publisher_loop(rabbit_mq, ROUTING_KEY_FILTERED_STATE, publish_queue), daemon=True).start()
        threading.Thread(target=filter_loop, daemon=True).start()
        rabbit_mq.start_consuming()

if __name__ == "__main__":
    main()