import threading
import time
import numpy as np
from pathlib import Path
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import LoadProgram, Play, MsgProtocol, InjectStuckJoint, InjectWear
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

def inject_wear():
    control_queue.put(
        InjectWear(5000, 0.1, [1,2,3,4,5])
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
        start_time = time.time()
        wear_injected = False
        stuck_injected = False
        while True:
            time.sleep(10)
            enqueue_program(scale=4*np.pi)
            elapsed = time.time() - start_time
            time.sleep(40)
            if not wear_injected and elapsed >= 120:
                inject_wear()
                wear_injected = True
                print("Injected wear")
            if False and not stuck_injected and elapsed >= 400:
                inject_stuck_joints()
                stuck_injected = True
                print("Injected stuck joints")
            #print("Enqueued new program")

if __name__ == "__main__":
    main()