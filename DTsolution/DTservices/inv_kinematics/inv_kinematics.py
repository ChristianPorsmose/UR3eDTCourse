from pathlib import Path
import time
from xml.parsers.expat import model
from utils.utils import load_config
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.typed_protocol import StuckJointStatus, LoadTCPProgram, LoadProgram, Play
from communication.rabbitmq import Rabbitmq
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np

class DHparam():
    def __init__(self, d, a, alpha):
        self.d = d
        self.a = a
        self.alpha = alpha

    def get_params(self):
        return self.d, self.a, self.alpha

class BreakableRobot:
    def __init__(self, rabbit_mq_client: TypedRabbitMQClient = None):
        # Initialize state
        self.stuck_joints = [False] * 6
        self.joint_angles = [0] * 6
        self.ur3e = self.init_ur3e()

        if rabbit_mq_client != None:
            self.rabbit_mq_client = rabbit_mq_client

    def init_ur3e(self):
        # DHParams specified on website
        DHparams = [
            DHparam(0.15185, 0, np.pi/2),
            DHparam(0.0, -0.24355, 0.0),
            DHparam(0.0, -0.2132, 0.0),
            DHparam(0.13105, 0.0, np.pi/2),
            DHparam(0.08535, 0, -np.pi/2),
            DHparam(0.0921, 0, 0)
        ]

        # Define the sequence of elementary transforms
        ets = rtb.ETS()
        for i, link in enumerate(DHparams):
            # Unpack parameters
            d, a, alpha = link.get_params()

            # Set the angle of the stuck joint if it is stuck
            if self.stuck_joints[i]:
                ets *= rtb.ET.Rz(self.joint_angles[i])
            else:
                ets *= rtb.ET.Rz() 
            ets *= rtb.ET.tz(d) if d != 0 else rtb.ET.tx(a)
            ets *= rtb.ET.Rx(alpha)

        # Compile into an ERobot model
        ur3e = rtb.ERobot(ets, name="UR3e")

        return ur3e

    def inv_kinematics(self, tcp_pos: np.ndarray, tcp_rot: np.ndarray, publish: bool = True) -> np.ndarray:
        
        T = SE3(tcp_pos) * SE3.RPY(tcp_rot)
        
        # sol.q will only contain angles for the ACTIVE joints
        sol = self.ur3e.ikine_LM(T)
        
        # Reconstruct the full 6-DOF joint array
        full_q = np.zeros(len(self.stuck_joints))
        active_q_idx = 0
        
        for i, is_stuck in enumerate(self.stuck_joints):
            if is_stuck:
                # Inject the constant stuck angle
                full_q[i] = self.joint_angles[i]
            else:
                # Map the next solved active joint angle
                full_q[i] = sol.q[active_q_idx]
                active_q_idx += 1

        if publish:
            self.rabbit_mq_client.publish(
                LoadProgram(joint_positions=[full_q.tolist()], max_velocity=60, acceleration=80)
            )
            self.rabbit_mq_client.publish(
                Play()
            )

        return full_q

    def update_state(self, body: StuckJointStatus):
        self.stuck_joints = np.array(body.stuck_joints)[0]
        self.joint_positions = np.array(body.stuck_joints)
        self.ur3e = self.init_ur3e()

def main():
    # Load config
    config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**config)) as typed_client:
        print("STARTING INV_KINEMATICS SERVICE")

        # Instantiate robot
        ur3e = BreakableRobot(typed_client)

        typed_client.subscribe(
            StuckJointStatus,
            ur3e.update_state,
            queue_name="inv_kinematics_stuck_joint"
        )
        typed_client.subscribe(
            LoadTCPProgram,
            lambda body: ur3e.inv_kinematics(body.tcp_position, body.tcp_rotation),
            queue_name="inv_kinematics_load_tcp_program"
        )
       
        typed_client.client.start_consuming()

if __name__ == "__main__":
    main()