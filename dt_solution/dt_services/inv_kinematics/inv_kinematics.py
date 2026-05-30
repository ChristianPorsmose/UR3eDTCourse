from pathlib import Path

import numpy as np

from utils.utils import load_config
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.typed_protocol import StuckJointStatus, LoadTCPProgram, JointProgram
from communication.rabbitmq import Rabbitmq
from models.breakable_robot import BreakableRobot


def main():
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING INV_KINEMATICS SERVICE")

        robot = BreakableRobot()

        def on_load_tcp_program(body: LoadTCPProgram):
            q = robot.inv_kinematics(np.array(body.tcp_position), np.array(body.tcp_rotation))
            typed_client.publish(JointProgram(
                joint_positions=q.tolist(),
                max_velocity=body.max_velocity,
                acceleration=body.acceleration,
            ))

        typed_client.subscribe(
            StuckJointStatus,
            robot.update_state,
            queue_name="inv_kinematics_stuck_joint"
        )
        typed_client.subscribe(
            LoadTCPProgram,
            on_load_tcp_program,
            queue_name="inv_kinematics_load_tcp_program"
        )

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
