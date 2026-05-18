from pathlib import Path
import time
from utils.utils import load_config
from communication.typed_protocol_client import TypedRabbitMQClient
from communication.typed_protocol import StuckJointStatus, LoadTCPProgram, LoadProgram, Play
from communication.rabbitmq import Rabbitmq
import roboticstoolbox as rtb
from spatialmath import SE3
import numpy as np
from models.ur3e import build_ur3e

class BreakableRobot:
    def __init__(self, rabbit_mq_client: TypedRabbitMQClient = None):
        self.stuck_joints = [False] * 6
        self.joint_angles = [0] * 6
        self.ur3e = self._init_ur3e()

        if rabbit_mq_client != None:
            self.rabbit_mq_client = rabbit_mq_client

    def _init_ur3e(self) -> rtb.ERobot:
        return build_ur3e(self.stuck_joints, self.joint_angles)

    def inv_kinematics(self, tcp_pos: np.ndarray, tcp_rot: np.ndarray, 
                   publish: bool = True, ignore_all_rotations=False) -> np.ndarray:
    
        T = SE3(tcp_pos) * SE3.RPY(tcp_rot)
        num_stuck = int(np.sum(self.stuck_joints))
        
        # --- MASK LOGIC ---
        if ignore_all_rotations:
            # Strict 3D position only. Great for debugging reachability!
            mask = [1, 1, 1, 0, 0, 0] 
        else:
            mask = [1, 1, 1, 1, 1, 1]
            for i in range(num_stuck):
                mask[5 - i] = 0
                    
        # Define max retries for IK attempts with random initializations
        max_retries = 10
        success = False
        
        for attempt in range(max_retries):
            # Generate a random initial guess (q0) within joint limits roughly -pi to pi
            q0 = np.random.uniform(-np.pi, np.pi, self.ur3e.n)
            
            sol = self.ur3e.ikine_LM(T, mask=mask, q0=q0)
            
            if sol.success:
                success = True
                break # We found a solution, stop searching
                
        if not success:
            print(f"\nWarning: IK totally failed after {max_retries} attempts.")
            print(f"Reason: {sol.reason}")
            print("This pose is likely physically unreachable with the current stuck joints.")
        
        # Re-introdcue the stuck joints to the commanded joint angles
        full_q = np.zeros(len(self.stuck_joints))
        active_q_idx = 0
        
        for i, is_stuck in enumerate(self.stuck_joints):
            if is_stuck:
                full_q[i] = self.joint_angles[i]
            else:
                full_q[i] = sol.q[active_q_idx]
                active_q_idx += 1

        if publish:
            self.rabbit_mq_client.publish(
                LoadProgram(joint_positions=full_q.tolist(), max_velocity=60, acceleration=80)
            )
            self.rabbit_mq_client.publish(
                Play()
            )

        return full_q

    def update_state(self, body: StuckJointStatus):
        self.stuck_joints = body.stuck_joints 
        self.joint_angles = body.joint_positions 
        self.ur3e = self._init_ur3e()

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