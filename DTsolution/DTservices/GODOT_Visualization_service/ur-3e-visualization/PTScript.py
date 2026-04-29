from py4godot.classes import gdclass

from communication.rabbitmq import ROUTING_KEY_STATE
from base_robot import RobotScriptBase


@gdclass
class PTScript(RobotScriptBase):
    def _ready(self):
        self.setup_robot(
            topic=ROUTING_KEY_STATE, 
            consumer_tag="long_name"
        )
