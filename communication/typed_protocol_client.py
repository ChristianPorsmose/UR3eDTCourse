from dataclasses import asdict
from communication.typed_protocol import MsgProtocol, T, CtrlMsg
from communication.rabbitmq import Rabbitmq
from typing import Callable, Type

MsgCallback = Callable[[MsgProtocol], None]

class TypedRabbitMQClient:
    def __init__(self, client: Rabbitmq) -> None:
        self.client = client

    def __enter__(self):
        self.client.__enter__()
        return self

    def __exit__(self, exc_type, exc, tb):
        return self.client.__exit__(exc_type, exc, tb)

    def subscribe(
        self,
        msg_type: Type[T],
        callback: Callable[[T], None],
        queue_name: str
    ):
        def handler(_, __, ___, body):
            if issubclass(msg_type, CtrlMsg):
                if body.get("type") == msg_type.type:
                    msg = msg_type(**body)
                    callback(msg)
                return 
            
            msg = msg_type(**body)
            callback(msg)

        self.client.subscribe(
            msg_type.routing_key(),
            handler,
            queue_name
        )

    def publish(self, msg: MsgProtocol):
        self.client.send_message(
            routing_key=msg.routing_key(),
            message=asdict(msg)
        )