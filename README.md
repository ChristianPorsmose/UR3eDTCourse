# UR3e Digital Twin Case Study

Welcome to the UR3e Case Study for the **Engineering Digital Twins** course at Aarhus University.

This repository contains the codebase and examples required to build and interact with a Digital Twin of a Universal Robots UR3e robotic arm. The project allows you to run a virtual mockup of the robot, interact with it via RabbitMQ, and develop your own digital twin components.

## Repository Structure

The repository is organized as follows:

- **`0_pre_requisites/`**: Contains instructions and notebooks for setting up your development environment. **Start here.**
- **`communication/`**: Contains the protocol definitions and RabbitMQ handling code used to exchange messages between the mockup and your digital twin.
- **`examples/`**: A series of Jupyter Notebooks demonstrating how to run, control, and configure the mockup.
- **`startup/`**: Python scripts and configurations required to launch the UR3e mockup services.
- **`ur3e_mockup/`**: The internal logic for the robot simulation.

## Getting Started

To get started with this case study, please follow these steps strictly in order:

1. **Environment Setup**: Navigate to `0_pre_requisites/` and complete the `0_environment_setup.ipynb` notebook. This ensures you have Python 3.11, Poetry, Docker, and RabbitMQ installed and configured correctly.
2. **Running the Mockup**: Follow the instructions in `examples/0_running_the_mockup.ipynb` to spin up the robot simulation.
3. **Interacting with the Robot**: Proceed through the remaining notebooks in the `examples/` folder to learn how to send control commands and receive state updates.

## Dependencies

This project is managed using [Poetry](https://python-poetry.org/). The core dependencies include:

- `roboticstoolbox-python`: For kinematic modeling and spatial math.
- `pika`: For RabbitMQ communication.
- `numpy`, `pandas`, `matplotlib`: For data handling and visualization.

To install the dependencies, run the following in the project root:

```bash
poetry install
```