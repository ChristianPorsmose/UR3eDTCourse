from collections import deque
from functools import partial
import threading
import numpy as np
from scipy.interpolate import interp1d
from pathlib import Path

from communication.protocol import RobotArmStateKeys as rb
from communication.protocol import Deviation as d
from communication.protocol import ROUTING_KEY_DEVIATION, ROUTING_KEY_STATE, ROUTING_KEY_KINEMATIC
from utils.utils import load_config
from communication.rabbitmq import Rabbitmq
import queue

latest_mock_queue = queue.Queue(maxsize=1)
kinematic_queue = deque(maxlen=20) # might need to be longer for accuraacy
publish_queue = queue.Queue()

def interpolate(target_time, data):
    if len(data) < 2:
        return None

    times = np.array([d[rb.TIMESTAMP] for d in data])
    values = np.array([d[rb.Q_ACTUAL] for d in data])

    f = interp1d(times, values, axis=0, kind='linear', fill_value="extrapolate")
    return f(target_time)


def deviation_loop():
    while True:
        latest_mock = latest_mock_queue.get()
        mock_time = latest_mock[rb.TIMESTAMP]
        kin_value = interpolate(mock_time, list(kinematic_queue))
        if kin_value is None:
            continue
        deviation = np.array(latest_mock[rb.Q_ACTUAL]) - np.array(kin_value)
        msg = {
            d.TIMESTAMP: mock_time,
            d.DEVIATIONS: deviation.tolist()
        }
        publish_queue.put(msg)


def publisher_loop(rabbit_mq: Rabbitmq):
    publish = partial(rabbit_mq.send_message, routing_key=ROUTING_KEY_DEVIATION)

    while True:
        msg = publish_queue.get()  

        rabbit_mq.connection.add_callback_threadsafe(
            lambda m=msg: publish(message=m)
        )

def main():
    config = load_config(Path("connect.yml"))
    print("STARTING ABNORMAL MOVEMENT")
    with Rabbitmq(**config) as rabbit_mq:

        subscriptions = {
            (ROUTING_KEY_STATE, "abnormal_queue_1"): lambda x: latest_mock_queue.put(x),
            (ROUTING_KEY_KINEMATIC, "abnormal_queue_2"): lambda x: kinematic_queue.append(x),
        }

        for (key, queue_name), func in subscriptions.items():
            rabbit_mq.subscribe(
                key,
                lambda _,__,___, body, f=func: f(body),
                queue_name
            )

        threading.Thread(target=deviation_loop, daemon=True).start()
        threading.Thread(target=lambda: publisher_loop(rabbit_mq), daemon=True).start()

        rabbit_mq.start_consuming()


if __name__ == "__main__":
    main()