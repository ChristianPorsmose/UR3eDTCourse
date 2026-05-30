from dt_solution.utils.utils import load_config
from pathlib import Path
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import LoadTCPProgram

with TypedRabbitMQClient(Rabbitmq(ip="localhost", port=5672, username="ur3e", password="ur3e", exchange="UR3E_AMQP", type="topic", vhost="/")) as typed_client:
    typed_client.publish(LoadTCPProgram(tcp_position=[0.2, 0.2, 0.2], tcp_rotation=[0, 0, 0], max_velocity=60, acceleration=80))