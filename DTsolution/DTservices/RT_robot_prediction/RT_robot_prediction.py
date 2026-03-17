from pathlib import Path
import time

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

        
def run_simulation(model : KinematicModel,rabbit_mq: Rabbitmq, dt=0.10):
    i= 0
    while True:
        start_time = time.time()
        with model_lock:
                current_time = i * dt
                model.fmi2DoStep(current_time, dt)

    current_state = {
        "timestamp": time.time(),
        "simulation_time": current_time,
        "joint_positions": model.fmi2GetJointPositions(),
        "joint_velocities": model.fmi2GetJointVelocities(),
        "source" : "rt_robot_prediction_service"
    }

    rabbit_mq.publish(protocol.ROUTING_KEY_STATE, current_state)

    elapsed_time = time.time() - start_time
    sleep_time = max(0, dt - elapsed_time)
    time.sleep(sleep_time)
    i += 1

def main():
    config = load_config(Path("connect.yml"))

    with Rabbitmq(**config) as rabbit_mq:
        model = KinematicModel(movement_fidelity=1000)

        rabbit_mq.subscribe(protocol.ROUTING_KEY_CTRL, lambda *_, body_json :
                            inject_ctrl_msg_to_model(model, body_json))
        
        rmq_thread = threading.Thread(target=rabbit_mq.start_consuming, daemon=True)
        rmq_thread.start()

        run_simulation(model, rabbit_mq)

if __name__ == "__main__":
    main()