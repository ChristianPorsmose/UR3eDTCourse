
from functools import partial
from pathlib import Path
import yaml
import queue
from communication.rabbitmq import Rabbitmq 



def load_config(path: Path) -> dict:
    with path.open() as f:
        return yaml.safe_load(f)
    
def publisher_loop(rabbit_mq: Rabbitmq, routing_key : str, publish_queue : queue.Queue):
    publish = partial(rabbit_mq.send_message, routing_key=routing_key)

    while True:
        msg = publish_queue.get()  
        
        rabbit_mq.connection.add_callback_threadsafe(
            lambda m=msg: publish(message=m)
        )