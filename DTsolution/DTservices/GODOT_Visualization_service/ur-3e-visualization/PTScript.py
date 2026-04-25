from py4godot.methods import private
from py4godot.signals import signal, SignalArg
from py4godot.classes import gdclass
from py4godot.classes.core import Vector3
from py4godot.classes.Node3D import Node3D

# # Custom imports
from communication import protocol
from communication.rabbitmq import Rabbitmq
from pathlib import Path
import yaml
import threading
import queue
import numpy as np


@gdclass
class PTScript(Node3D):
	def _ready(self):
		# Topic to subscribe to
		self.topic = "robotarm.pt.state"

		# Queue for putting messages received from topic
		self.message_queue = queue.Queue()
			
		print("Config loaded. Starting RabbitMQ thread...")
		
		# 2. Spin up a background thread. 
		# daemon=True ensures the thread dies when you close the Godot window.
		rmq_thread = threading.Thread(target=self.run_rabbitmq, daemon=True)
		rmq_thread.start()

	def _process(self, delta: float):
		while not self.message_queue.empty():
			body = self.message_queue.get()
			print(f"Godot received: {body}")
			for i in range(1, 7):
				angle = body["q_actual"][i-1]
				self.apply_rotation(angle, i)

	# 3. Move all the blocking RabbitMQ logic into this new function
	def run_rabbitmq(self):
		with Rabbitmq(ip="localhost", port=5672, username="ur3e", password="ur3e", vhost="/", exchange="UR3E_AMQP", type="topic") as rabbit_mq:
			rabbit_mq.subscribe(self.topic, self.on_message_received)
			print("Listening for messages...")
			rabbit_mq.start_consuming() # This loop now runs safely in the background!

	@private
	def on_message_received(self, ch, method, properties, body):
		self.message_queue.put(body)

	@private
	def apply_rotation(self, angle, link_num: 1 | 2 | 3 | 4 | 5 | 6):
		joint_axis_lut = {
			1: "y",
			2: "z",
			3: "z",
			4: "z",
			5: "y",
			6: "z"
		}

		children_base_string = "Base/Link1/Link2/Link3/Link4/Link5/Link6"

		child = self.get_node(children_base_string[:link_num * 6 + 5])
		if child:
			rotation_rad = Vector3() 

			rotation_rad.x = 0

			if (joint_axis_lut[link_num] == "y"):
				rotation_rad.y = angle
				rotation_rad.z = 0
			else:
				rotation_rad.y = 0
				rotation_rad.z = angle

			child.set_rotation(rotation_rad)
