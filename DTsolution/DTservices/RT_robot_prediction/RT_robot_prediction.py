from pathlib import Path
import time
from xml.parsers.expat import model

from utils.utils import load_config
from communication import protocol
from communication.rabbitmq import Rabbitmq
from DTsolution.models.kinematic_model import KinematicModel
import threading

model_lock = threading.Lock()

def inject_ctrl_msg_to_model(model : KinematicModel, body : dict):
    with model_lock:
        if body["type"] == "load_program":
            model.fmi2SetCommandedJointAngles(body["joint_positions"][0])
        elif body["type"] == "play":
            model.fmi2StartMovement()
    
def inject_ctrl_msg_to_influxdb(writer, body):
    if body["type"] not in ["load_program", "play"]:
        return
    
    point = point("control_messages").time(time.time_ns()).tag("source", "rt_robot_prediction_service")
    point = point.field("message_type", body["type"])
    point = point.field("joint_positions", str(body.get("joint_positions", [])))

        
def run_simulation(model : KinematicModel, rabbit_mq: Rabbitmq, dt=0.05):
    i = 0
    while True:
        start_time = time.time()
        
        with model_lock: # Keep EVERYTHING model-related inside the lock
            current_time = i * dt
            model.fmi2DoStep(current_time, dt)

            pos = [float(x) for x in model.fmi2GetJointPositions()]
            vel = [float(x) for x in model.fmi2GetJointVelocities()]

            # Define the state while we have the lock
            current_state = {
                "q_actual": pos,
                "qd_actual": vel,
                "q_target": pos,  # Assuming target positions are the same as actual positions for now
                "timestamp": float(time.time()),
                "robot_mode": "RUNNING" if model.moving else "IDLE",
                "source": "rt_robot_prediction_service",
                "joint_max_speed": 60.0,
                "joint_max_acceleration": 80.0,
                "tcp_pose": [0.0] * 6,
                "simulation_time": float(current_time),
                # Duplicates for your own use
                #"joint_positions": model.fmi2GetJointPositions(),
                #"joint_velocities": model.fmi2GetJointVelocities(),
            }

        # Publish outside the lock so we don't hold up the listener thread
        rabbit_mq.send_message(protocol.ROUTING_KEY_RT_MODEL_STATE, current_state)

        elapsed_time = time.time() - start_time
        time.sleep(max(0, dt - elapsed_time))
        i += 1
 
def main():
    config = load_config(Path("connect.yml"))

    # Connection 1: For the Background Listener
    rabbit_mq_listener = Rabbitmq(**config)
    rabbit_mq_listener.connect_to_server()
    
    # Connection 2: For the Simulation Loop
    rabbit_mq_publisher = Rabbitmq(**config)
    rabbit_mq_publisher.connect_to_server()

    model = KinematicModel()

    # Subscribe using the listener connection
    rabbit_mq_listener.subscribe(protocol.ROUTING_KEY_CTRL, 
                             lambda ch, method, properties, body_json: inject_ctrl_msg_to_model(model, body_json))
    
    # Start the listener thread
    rmq_thread = threading.Thread(target=rabbit_mq_listener.start_consuming, daemon=True)
    rmq_thread.start()

    # Start the simulation using the publisher connection
    try:
        run_simulation(model, rabbit_mq_publisher)
    finally:
        rabbit_mq_listener.close()
        rabbit_mq_publisher.close()

if __name__ == "__main__":
    main()

#this service should 
#consume control messages in mockup from rabbit mq, 
#inject into kinematic model, 
#publish kinematic model control messages to rabbitmq
#loop simulation at 20 hz