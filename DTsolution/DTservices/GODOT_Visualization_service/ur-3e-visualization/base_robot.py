from py4godot.methods import private
from py4godot.classes import gdclass
from py4godot.classes.core import Vector3
from py4godot.classes.Node3D import Node3D

from communication.rabbitmq import Rabbitmq
from pathlib import Path
import yaml
import threading
import queue
import math

class RobotScriptBase(Node3D):
    """
    Base class for robot synchronization via RabbitMQ.
    Children must define self.topic and self.consumer_tag in _ready().
    """
    
    def setup_robot(self, topic: str, consumer_tag: str):
        self.topic = topic
        self.consumer_tag = consumer_tag
        self.message_queue = queue.Queue()

        # Load configuration
        config_path = Path("communication/connect.yml")
        with open(config_path, 'r') as file:
            self.connect_config = yaml.safe_load(file)
            
        print(f"[{self.__class__.__name__}] Config loaded. Starting RabbitMQ thread...")
        
        # Background thread for non-blocking IO
        rmq_thread = threading.Thread(target=self.run_rabbitmq, daemon=True)
        rmq_thread.start()

    def _process(self, delta: float):
        while not self.message_queue.empty():
            body = self.message_queue.get()
            # Expecting 'q_actual' as a list of 6 angles
            if "q_actual" in body:
                for i in range(1, 7):
                    angle = body["q_actual"][i-1]
                    self.apply_rotation(angle, i)

    def run_rabbitmq(self):
        import time
        while True:
            try:
                with Rabbitmq(**self.connect_config) as rabbit_mq:
                    rabbit_mq.subscribe(self.topic, self.on_message_received, self.consumer_tag)
                    print(f"[{self.__class__.__name__}] Listening on {self.topic}...", flush=True)
                    rabbit_mq.start_consuming()
            except Exception as e:
                print(f"[{self.__class__.__name__}] RabbitMQ error: {e}, retrying in 5s...", flush=True)
                time.sleep(5)

    @private
    def on_message_received(self, ch, method, properties, body):
        self.message_queue.put(body)

    @private
    def apply_rotation(self, angle, link_num: int):
        joint_axis_lut = {1: "y", 2: "z", 3: "z", 4: "z", 5: "y", 6: "z"}
        angle_offsets = {1: 0, 2: -math.pi/2, 3: 0, 4: -math.pi/2, 5: 0, 6: 0} # Upright position
        rotation_signs = {1: 1, 2: -1, 3: 1, 4: -1, 5: 1, 6: 1} # Upright position
        children_base_string = "Base/Link1/Link2/Link3/Link4/Link5/Link6"

        # Calculate path: Base/Link1 is index 1, etc.
        child = self.get_node(children_base_string[:link_num * 6 + 5])
        
        if child:
            effective_angle = rotation_signs[link_num] * (angle - angle_offsets[link_num])
            rotation_rad = Vector3()
            rotation_rad.x = 0
            if joint_axis_lut[link_num] == "y":
                rotation_rad.y = effective_angle
                rotation_rad.z = 0
            else:
                rotation_rad.y = 0
                rotation_rad.z = effective_angle
            child.set_rotation(rotation_rad)