from py4godot.classes import gdclass

from communication.rabbitmq import ROUTING_KEY_RT_MODEL_STATE
from base_robot import RobotScriptBase

@gdclass
class DTScript(RobotScriptBase):
    def _ready(self):
        self.setup_robot(
            topic=ROUTING_KEY_RT_MODEL_STATE, 
            consumer_tag="short_name_hehehehhe"
        )