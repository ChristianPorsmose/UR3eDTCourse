from collections import deque
import threading
import numpy as np
from pathlib import Path
from utils.utils import load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import FilteredState, LoadProgram, StuckJointStatus, InjectFault
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.typed_protocol import PhysicalTwinState 
import queue

consumer_queue : queue.Queue[FilteredState] = queue.Queue()
publish_queue : queue.Queue[StuckJointStatus] = queue.Queue()

def is_constant(values: np.ndarray, eps: float) -> np.ndarray:
    return (np.max(values, axis=0) - np.min(values, axis=0)) < eps

def stuck_joint_loop():
    history : deque[any] = deque(maxlen=10)
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

        deviation = np.abs(q_actuals[-1] - q_target[-1])

        stuck_mask : np.ndarray = constant_mask & (deviation > DEV_EPS)

        # if np.any(stuck_mask):
        #     publish_queue.put(
        #         StuckJointStatus(
        #             stuck_mask.tolist(),
        #             joint_positions = q_actuals[-1].tolist()
        #         )
        #     )
        # publish_queue.put(
        #     StuckJointStatus(
        #         stuck_mask.tolist(),
        #         joint_positions = q_actuals[-1].tolist()
        #     )
        # )
        print(f"DEBUG: Detector calculated mask: {stuck_mask}") # ADD THIS
        # Inside stuck_joints_loop in stuck_joint.py
        publish_queue.put(
            StuckJointStatus(
                stuck_joints = stuck_mask.tolist(), # Ensure this matches your dataclass field name
                joint_positions = q_actuals[-1].tolist()
            )
)

# def main():
#     config = load_config(Path("connect.yml"))

#     with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
#         print("STARTING STUCK JOINT DETECTION")
#         typed_client.subscribe(
#             FilteredState, 
#             lambda msg: (print("!!! MESSAGE RECEIVED BY SERVICE !!!"), consumer_queue.put(msg)),
#             queue_name="stuck_joint"
#         )
#         typed_client.subscribe(
#             InjectFault,
#             lambda msg : print(msg),
#             queue_name="stuck_joint_inject"
#         )
#         threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()
#         threading.Thread(target=stuck_joints_loop, daemon=True).start()
#         typed_client.client.start_consuming()
def main():
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING STUCK JOINT DETECTION")
        
        # 1. Use PhysicalTwinState because it MATCHES the key "robotarm.pt.state"
        # 2. Use the standard typed subscribe (no raw client hacks needed)
        typed_client.subscribe(
            PhysicalTwinState, 
            consumer_queue.put, 
            queue_name="stuck_joint_queue"
        )

        typed_client.subscribe(
            InjectFault,
            lambda msg : print(f"FAULT RECEIVED: {msg}"),
            queue_name="stuck_joint_inject"
        )
        
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()
        threading.Thread(target=stuck_joint_loop, daemon=True).start()
        
        typed_client.client.start_consuming()
if __name__ == "__main__":
    main()