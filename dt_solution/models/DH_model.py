import roboticstoolbox as rtb
import numpy as np

def get_DH_robot() -> rtb.robot.DHRobot:
    # Define links using DH params from website
    link1 = rtb.RevoluteDH(d=0.15185, a=0.0,      alpha=np.pi / 2)
    link2 = rtb.RevoluteDH(d=0.0,     a=-0.24355, alpha=0.0)
    link3 = rtb.RevoluteDH(d=0.0,     a=-0.2132,  alpha=0.0)
    link4 = rtb.RevoluteDH(d=0.13105, a=0.0,      alpha=np.pi / 2)
    link5 = rtb.RevoluteDH(d=0.08535, a=0.0,      alpha=-np.pi / 2)
    link6 = rtb.RevoluteDH(d=0.0921,  a=0.0,      alpha=0.0)

    # Instantiate robot
    robot = rtb.DHRobot([link1, link2, link3, link4, link5, link6], name="My_UR3e")
    return robot