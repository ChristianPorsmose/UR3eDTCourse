
from functools import partial
from pathlib import Path
import numpy as np
import yaml
import queue
from scipy.interpolate import interp1d
from communication.rabbitmq import Rabbitmq 
from communication.protocol import RobotArmStateKeys as rb
from communication.typed_protocol_client import TypedRabbitMQClient


def load_config(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)
    
def typed_publisher_loop(typed_client : TypedRabbitMQClient, publish_queue : queue.Queue):
    while True:
        msg = publish_queue.get()  
        typed_client.client.connection.add_callback_threadsafe(
            lambda m=msg: typed_client.publish(m)
        )

def publisher_loop(rabbit_mq: Rabbitmq, routing_key : str, publish_queue : queue.Queue):
    publish = partial(rabbit_mq.send_message, routing_key=routing_key)

    while True:
        msg = publish_queue.get()  
        print(msg)
        rabbit_mq.connection.add_callback_threadsafe(
            lambda m=msg: publish(message=m)
        )

def interpolate(target_time: float, data: list[dict], value_key: str):
    if len(data) < 2:
        return None
    sorted_data = sorted(data, key=lambda d: d[rb.TIMESTAMP])

    times = np.array([d[rb.TIMESTAMP] for d in sorted_data])
    values = np.array([d[value_key] for d in sorted_data])
    f = interp1d(
        times,
        values,
        axis=0,
        kind='linear',
        bounds_error=False,
        fill_value=(values[0], values[-1]) 
    )

    return f(target_time)