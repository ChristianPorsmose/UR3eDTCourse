from pathlib import Path
import numpy as np
from utils.utils import load_config
from communication.rabbitmq import Rabbitmq
from communication.typed_protocol import PhysicalTwinState, WearStatus
from communication.typed_protocol_client import TypedRabbitMQClient
import mstlo_python as mstlo
from dt_solution.models.DH_model import get_DH_robot

# Set threshold for when wear should be detected
WEAR_THRESHOLD = 0.1 # Larger than some of the larger swings when no wear is present 

# load DH model
robot = get_DH_robot()

# Instantiate STL monitor
vars = mstlo.Variables()
vars.set("threshold", WEAR_THRESHOLD)
phi = mstlo.parse_formula("G[0,9](e < $threshold)")

# Create the Monitor (using Robustness Semantics)
monitor = mstlo.Monitor(
    phi, semantics="Rosi", variables=vars
)


def check_wear(pt_state: PhysicalTwinState, typed_client: TypedRabbitMQClient) -> None:
    # Calculate (x, y, z) for the TCP usin q_actual
    tcp_theoretical = np.array(robot.fkine(pt_state.q_actual).t)

    # Get the measures (x, y, z) for TCP from PT state
    tcp_measured = pt_state.tcp_pose[:3]
    tcp_measured = np.array(tcp_measured)

    # Calculate the error
    tcp_error_m = np.linalg.norm(tcp_theoretical - tcp_measured)
    tcp_error_mm = tcp_error_m * 1000  # Convert to mm for easier reading

    result = monitor.update("e", tcp_error_mm, pt_state.timestamp)
    final_result = result.verdicts()[0][1][1] # Extract the usable verdict over the "Global" operator
    if tcp_error_mm > WEAR_THRESHOLD or final_result < 0:
        print("Too large error: ", tcp_error_mm )
        print(result.verdicts()[0])
    
    # Publish final verdict
    typed_client.publish(
        WearStatus(wear_detected=final_result < 0, affected_joints=[]), # TODO: Maybe do ML for finding affected joints
        )

def main():
    connect_config = load_config(Path("connect.yml"))

    with TypedRabbitMQClient(Rabbitmq(**connect_config)) as typed_client:
        typed_client.subscribe(PhysicalTwinState, lambda s: check_wear(s, typed_client), "wear_det_pt")

        typed_client.client.start_consuming()


if __name__ == "__main__":
    main()
