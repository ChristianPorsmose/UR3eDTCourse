
from cmath import pi
from dataclasses import asdict

from models.breakable_robot import BreakableRobot

robot = BreakableRobot()

print(robot.ur3e.fkine([0, -pi, 0, -pi, 0, 0]).t)

