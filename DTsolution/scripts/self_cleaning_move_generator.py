import threading
import time
import numpy as np
from functools import partial
from pathlib import Path
from communication import protocol
from communication.rabbitmq import Rabbitmq
from utils.utils import load_config
import queue

control_queue = queue.Queue()
publish_event = threading.Event()

def create_random_program(scale: float = 0.5*np.pi, vel: float = 60, acc: float = 80):
    position = ((np.random.rand(6) - 0.5) * scale).tolist()
    return {
        protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.LOAD_PROGRAM,
        protocol.CtrlMsgKeys.JOINT_POSITIONS: [position],
        protocol.CtrlMsgKeys.MAX_VELOCITY: vel,
        protocol.CtrlMsgKeys.ACCELERATION: acc
    }

def enqueue_program(scale: float = 0.5*np.pi):
    control_queue.put(create_random_program(scale))
    control_queue.put({protocol.CtrlMsgKeys.TYPE: protocol.CtrlMsgFields.PLAY})
    publish_event.set()


def publisher_loop(rabbit_mq: Rabbitmq):
    while True:
        publish_event.wait()

        while not control_queue.empty():
            msg = control_queue.get()
            rabbit_mq.send_message(routing_key=protocol.ROUTING_KEY_CTRL, message=msg)
            print("Published:", msg)

        publish_event.clear()


def main():
    config = load_config(Path("connect.yml"))
    print("STARTING MOVE GENERATOR")

    with Rabbitmq(**config) as rabbit_mq:
        threading.Thread(target=lambda: publisher_loop(rabbit_mq), daemon=True).start()

        while True:
            enqueue_program(scale=4*np.pi)
            print("Enqueued new program")
            time.sleep(60)

if __name__ == "__main__":
    main()