from collections import deque
import threading
import numpy as np
from pathlib import Path

from communication.protocol import RobotArmStateKeys as rb
from communication.protocol import StuckJoint as s
from communication.protocol import ROUTING_KEY_STUCK_JOINT, ROUTING_KEY_STATE
from utils.utils import load_config, publisher_loop
from communication.rabbitmq import Rabbitmq
import queue

consumer_queue = queue.Queue()

kinematic_queue = deque(maxlen=20)
publish_queue = queue.Queue()


def is_constant(values: np.ndarray, eps: float) -> np.ndarray:
    return (np.max(values, axis=0) - np.min(values, axis=0)) < eps

def stuck_joints_loop():
    history = deque(maxlen=10)
    CONST_EPS = 0.01 
    DEV_EPS = 1e-3

    while True:
        msg = consumer_queue.get()
        history.append(msg)

        if len(history) < history.maxlen:
            continue
        q_actuals = np.array([m[rb.Q_ACTUAL] for m in history])
        q_target = np.array(msg[rb.Q_TARGET])

        constant_mask = is_constant(q_actuals, eps=CONST_EPS)

        deviation = np.abs(q_actuals[-1] - q_target)

        stuck_mask : np.ndarray = constant_mask & (deviation > DEV_EPS)

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
                ROUTING_KEY_STATE,
                lambda _, __, ___, body : consumer_queue.put(body),
                queue_name="stuck_joint"
            )
        threading.Thread(target=lambda: publisher_loop(rabbit_mq, ROUTING_KEY_STUCK_JOINT, publish_queue), daemon=True).start()
        threading.Thread(target=stuck_joints_loop, daemon=True).start()
        rabbit_mq.start_consuming()


if __name__ == "__main__":
    main()