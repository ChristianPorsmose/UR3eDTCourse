import numpy as np
import roboticstoolbox as rtb


def build_ur3e(stuck_joints=None, joint_angles=None) -> rtb.ERobot:
    DH = [
        (0.15185, 0.0,      np.pi / 2),
        (0.0,    -0.24355,  0.0),
        (0.0,    -0.2132,   0.0),
        (0.13105, 0.0,      np.pi / 2),
        (0.08535, 0.0,     -np.pi / 2),
        (0.0921,  0.0,      0.0),
    ]
    ets = rtb.ETS()
    for i, (d, a, alpha) in enumerate(DH):
        if stuck_joints and stuck_joints[i]:
            ets *= rtb.ET.Rz(joint_angles[i])
        else:
            ets *= rtb.ET.Rz()
        ets *= rtb.ET.tz(d) if d != 0 else rtb.ET.tx(a)
        ets *= rtb.ET.Rx(alpha)
    return rtb.ERobot(ets, name="UR3e")
