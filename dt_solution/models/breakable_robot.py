import numpy as np
import roboticstoolbox as rtb
from spatialmath import SE3

from models.ur3e import build_ur3e
from communication.typed_protocol import StuckJointStatus


class BreakableRobot:
    def __init__(self):
        self.stuck_joints = [False] * 6
        self.joint_angles = [0] * 6
        self.ur3e = self._init_ur3e()

    def _init_ur3e(self) -> rtb.ERobot:
        return build_ur3e(self.stuck_joints, self.joint_angles)
    
    def _get_unstuck_joint_angles(self) -> list:
        return [
            angle for angle, is_stuck in zip(self.joint_angles, self.stuck_joints) 
            if not is_stuck
        ]

    def inv_kinematics(self, tcp_pos: np.ndarray, tcp_rot: np.ndarray,
                       ignore_all_rotations=False) -> np.ndarray:

        T = SE3(tcp_pos) * SE3.RPY(tcp_rot)
        num_stuck = int(np.sum(self.stuck_joints))

        if ignore_all_rotations:
            mask = [1, 1, 1, 0, 0, 0]
        else:
            mask = [10, 10, 10, 1, 1, 1]
            for i in range(num_stuck):
                mask[5 - i] = 0

        max_retries = 10
        success = False

        for attempt in range(max_retries):
            q0 = self._get_unstuck_joint_angles() if attempt == 0 else np.random.uniform(-np.pi, np.pi, self.ur3e.n)
            sol = self.ur3e.ikine_GN(T, mask=mask, q0=q0)
            if sol.success:
                success = True
                break

        if not success:
            print(f"\nWarning: IK totally failed after {max_retries} attempts.")
            print(f"Reason: {sol.reason}")
            print("This pose is likely physically unreachable with the current stuck joints.")
            return None


        full_q = np.zeros(len(self.stuck_joints))
        active_q_idx = 0
        for i, is_stuck in enumerate(self.stuck_joints):
            if is_stuck:
                full_q[i] = self.joint_angles[i]
            else:
                full_q[i] = sol.q[active_q_idx]
                active_q_idx += 1

        return full_q

    def update_state(self, body: StuckJointStatus):
        self.stuck_joints = body.stuck_joints
        self.joint_angles = body.joint_positions
        self.ur3e = self._init_ur3e()
