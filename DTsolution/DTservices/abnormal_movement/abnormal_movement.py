from collections import deque
import threading
import numpy as np
from pathlib import Path
from communication.typed_protocol import FilteredState, KinematicModelState, Deviation
from communication.typed_protocol_client import TypedRabbitMQClient
from utils.utils import interpolate, load_config, typed_publisher_loop
from communication.rabbitmq import Rabbitmq
import queue

latest_mock_queue : queue.Queue[FilteredState] = queue.Queue(maxsize=1)
kinematic_queue : deque[KinematicModelState] = deque(maxlen=20) # might need to be longer for accuraacy
publish_queue : queue.Queue[Deviation] = queue.Queue()
EPSILON = 1.6 # maybe change this? 

def deviation_loop():
    while True:
        latest_mock = latest_mock_queue.get()
        mock_time = latest_mock.timestamp
        kin_value = interpolate(mock_time, list(kinematic_queue), lambda x: x.q_actual)
        if kin_value is None :
            continue
        deviation = np.array(latest_mock.q_actual) - np.array(kin_value)
        if not np.any(np.abs(deviation) > EPSILON):
            continue
        publish_queue.put(Deviation(joint_deviations=deviation.tolist()))
 

def main():
    config = load_config(Path("connect.yml"))
    print("STARTING ABNORMAL MOVEMENT")
    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        
        typed_client.subscribe(
            FilteredState, lambda x: latest_mock_queue.put(x), "abnormal_queue_1"
        )
        typed_client.subscribe(
            KinematicModelState, lambda x: kinematic_queue.append(x),"abnormal_queue_2"
        )

        threading.Thread(target=deviation_loop, daemon=True).start()
        threading.Thread(target=lambda: typed_publisher_loop(typed_client, publish_queue), daemon=True).start()

        typed_client.client.start_consuming()

if __name__ == "__main__":
    main()