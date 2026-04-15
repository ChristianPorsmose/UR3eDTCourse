import threading
import time
import numpy as np
from pathlib import Path
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import LoadProgram, Play, MsgProtocol, InjectFault, StuckJoint
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import load_config, typed_publisher_loop
import queue

control_queue :  queue.Queue[MsgProtocol]  = queue.Queue()

def create_random_program(scale: float = 0.5*np.pi, vel: float = 60, acc: float = 80) -> LoadProgram:
    position = ((np.random.rand(6) - 0.5) * scale).tolist()
    return LoadProgram(
        joint_positions = [position],
        max_velocity = vel,
        acceleration= acc
    )

def inject_stuck_joints():
    control_queue.put(
        InjectFault(
            StuckJoint(
                [0, 1, 2]
            )
        )
    )

def enqueue_program(scale: float = 0.5*np.pi):
    control_queue.put(create_random_program(scale))
    control_queue.put(Play())


def main():
    config = load_config(Path("connect.yml"))
    print("STARTING MOVE GENERATOR")

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, control_queue), daemon=True).start()

        while True:
            enqueue_program(scale=4*np.pi)
            print("Enqueued new program")
            time.sleep(30)
            if np.random.rand() < 1:
                inject_stuck_joints()
                print("Injected stuck joint fault")
            time.sleep(30)

if __name__ == "__main__":
    main()