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

mock_event = threading.Event()
publish_event = threading.Event()


def consume_mock_output(msg: dict):
    latest_mock_queue.put(msg)
    mock_event.set()

def consume_kinematic_output(msg: dict):
    kinematic_queue.append(msg)

def interpolate(target_time, data):
    if len(data) < 2:
        return None

    times = np.array([d[rb.TIMESTAMP] for d in data])
    values = np.array([d[rb.Q_ACTUAL] for d in data])

    f = interp1d(times, values, axis=0, kind='linear', fill_value="extrapolate")
    return f(target_time)


def deviation_loop():
    while True:
        mock_event.wait()
        latest_mock = latest_mock_queue.get(msg)
        mock_event.set()
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
        publish_event.set()

        mock_event.clear()

def publisher_loop(rabbit_mq: Rabbitmq):
    publish = partial(rabbit_mq.send_message,routing_key=ROUTING_KEY_DEVIATION )
    while True:
        publish_event.wait()

        while publish_queue:
            msg = publish_queue.get()

            rabbit_mq.connection.add_callback_threadsafe(
                lambda m=msg: publish(message=m)
            )

        publish_event.clear()

def main():
    config = load_config(Path("connect.yml"))

    with Rabbitmq(**config) as rabbit_mq:

        subscriptions = {
            ROUTING_KEY_STATE: consume_mock_output,
            ROUTING_KEY_KINEMATIC: consume_kinematic_output,
        }

        for key, func in subscriptions.items():
            rabbit_mq.subscribe(
                key,
                lambda ch, method, props, body, f=func: f(body)
            )

        threading.Thread(target=deviation_loop, daemon=True).start()
        threading.Thread(target=lambda: publisher_loop(rabbit_mq), daemon=True).start()

        rabbit_mq.start_consuming()


if __name__ == "__main__":
    main()