from collections import deque
from functools import partial
import threading
import numpy as np
from pathlib import Path

from communication.protocol import RobotArmStateKeys as rb
from communication.protocol import StuckJoint as s
from communication.protocol import ROUTING_KEY_STATE, ROUTING_KEY_STUCK_JOINT, ROUTING_KEY_CTRL
from utils.utils import load_config
from communication.rabbitmq import Rabbitmq
import queue

consumer_queue = queue.Queue()

kinematic_queue = deque(maxlen=20)
publish_queue = queue.Queue()

def consume_mock_output(msg: dict):
    consumer_queue.put(msg)

def publisher_loop(rabbit_mq: Rabbitmq):
    publish = partial(rabbit_mq.send_message, routing_key=ROUTING_KEY_STUCK_JOINT)

    while True:
        msg = publish_queue.get()

        rabbit_mq.connection.add_callback_threadsafe(
            lambda m=msg: publish(message=m)
        )

def is_constant(values: np.ndarray, eps: float) -> np.ndarray:
    return (np.max(values, axis=0) - np.min(values, axis=0)) < eps

def stuck_joints_loop():
    history = deque(maxlen=10)
    EPSILON = 1e-3

    while True:
        msg = consumer_queue.get()
        history.append(msg)

        if len(history) < history.maxlen:
            continue

        q_actuals = np.array([m[rb.Q_ACTUAL] for m in history])
        q_target = np.array(msg[rb.Q_TARGET])

        constant_mask = (np.max(q_actuals, axis=0) - np.min(q_actuals, axis=0)) < EPSILON
        deviation = np.abs(q_actuals[-1] - q_target)

        stuck_mask : np.ndarray = constant_mask & (deviation > EPSILON)

        if np.any(stuck_mask):
            publish_queue.put({
                s.TIMESTAMP: msg[rb.TIMESTAMP],
                s.STUCK_JOINTS: stuck_mask.tolist()
            })


def main():
    config = load_config(Path("connect.yml"))

    with Rabbitmq(**config) as rabbit_mq:
        print("STARTING STUCK JOINT DETECTION")
        rabbit_mq.subscribe(
                ROUTING_KEY_CTRL,
                lambda _, __, ___, body, : consumer_queue.put(body),
                queue_name="stuck_joint"
            )
        threading.Thread(target=lambda: publisher_loop(rabbit_mq), daemon=True).start()

        rabbit_mq.start_consuming()


if __name__ == "__main__":
    main()