import threading
import time
import numpy as np
from pathlib import Path
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import LoadProgram, Play, MsgProtocol, InjectStuckJoint
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import load_config
import queue

control_queue :  queue.Queue[MsgProtocol]  = queue.Queue()

def create_random_program(scale: float = 0.5*np.pi, vel: float = 60, acc: float = 80) -> LoadProgram:
    position = ((np.random.rand(6) - 0.5) * scale).tolist()
    return LoadProgram(
        joint_positions = position,
        max_velocity = vel,
        acceleration= acc
    )

def inject_stuck_joints():
    control_queue.put(
        InjectStuckJoint([0,1,2])
    )

def enqueue_program(scale: float = 0.5*np.pi):
    control_queue.put(create_random_program(scale))
    control_queue.put(Play())

def publisher_loop(rabbit_mq: TypedRabbitMQClient):
    while True:
        msg = control_queue.get()
        rabbit_mq.publish(msg)

def main():
    config = load_config(Path("connect.yml"))
    print("STARTING MOVE GENERATOR")

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        threading.Thread(target=lambda: publisher_loop(typed_client), daemon=True).start()
        while True:
            time.sleep(30)
            enqueue_program(scale=4*np.pi)
            inject_stuck_joints()
            print("Enqueued new program")

if __name__ == "__main__":
    main()