
from functools import partial
from pathlib import Path
from typing import Any, Callable, Iterable, TypeVar
import numpy as np
import yaml
import queue
from scipy.interpolate import interp1d
from communication.rabbitmq import Rabbitmq 
from communication.protocol import RobotArmStateKeys as rb
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.typed_protocol import TimeStamped


def load_config(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)
    
def typed_publisher_loop(typed_client : TypedRabbitMQClient, publish_queue : queue.Queue):
    while True:
        msg = publish_queue.get() 
        typed_client.client.connection.add_callback_threadsafe(
            partial(typed_client.publish, msg)
        )

def publisher_loop(rabbit_mq: Rabbitmq, routing_key : str, publish_queue : queue.Queue):
    publish = partial(rabbit_mq.send_message, routing_key=routing_key)

    while True:
        msg = publish_queue.get()  
        print(msg)
        rabbit_mq.connection.add_callback_threadsafe(
            lambda m=msg: publish(message=m)
        )

TS = TypeVar("TS", bound="TimeStamped")

def interpolate(target_time: float, data: Iterable[TS], field_selector: Callable[[TS], Any]):
    if len(data) < 2:
        return None
    sorted_data = sorted(data, key=lambda d: d.timestamp)

    times = np.array([d.timestamp for d in sorted_data])
    values = np.array([field_selector(d) for d in sorted_data])
    
    f = interp1d(
        times,
        values,
        axis=0,
        kind='linear',
        bounds_error=False,
        fill_value=(values[0], values[-1]) 
    )

    return f(target_time)