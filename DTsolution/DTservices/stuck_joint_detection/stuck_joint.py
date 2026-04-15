from collections import deque
import threading
import numpy as np
from pathlib import Path
from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import PhysicalTwinState, StuckJointStatus, InjectFault
from communication.typed_protocol_client import TypedRabbitMQClient
import queue
import datetime

consumer_queue : queue.Queue[PhysicalTwinState] = queue.Queue()
publish_queue : queue.Queue[StuckJointStatus] = queue.Queue()

def is_constant(values: np.ndarray, eps: float) -> np.ndarray:
    return (np.max(values, axis=0) - np.min(values, axis=0)) < eps

def stuck_joints_loop():
    history : deque[PhysicalTwinState] = deque(maxlen=10)
    CONST_EPS = 0.01 
    DEV_EPS = 1e-3

    while True:
        msg = consumer_queue.get()
        history.append(msg)

        if len(history) < history.maxlen:
            continue

        q_actuals = np.array([m.q_actual for m in history])
        q_target = np.array([m.q_target for m in history])

        constant_mask = is_constant(q_actuals, eps=CONST_EPS)

        deviation = np.abs(q_actuals[-1] - q_target)

        stuck_mask : np.ndarray = constant_mask & (deviation > DEV_EPS)

        if np.any(stuck_mask):
            publish_queue.put(
                StuckJointStatus(
                    datetime.datetime.now().time(),
                    stuck_mask.tolist()
                )
            )

def main():
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING STUCK JOINT DETECTION")
        typed_client.subscribe(
                PhysicalTwinState,
                consumer_queue.put,
                queue_name="stuck_joint"
            )
        typed_client.subscribe(
            InjectFault,
            lambda msg : print(msg),
            "load_queue_debug"
        )
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()
        threading.Thread(target=stuck_joints_loop, daemon=True).start()
        typed_client.client.start_consuming()

if __name__ == "__main__":
    main()