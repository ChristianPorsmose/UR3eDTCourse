# Physical Twin Mockup

This folder contains the scripts that are used to simulate the behavior of the UR3e robot arm and its controller. 

# Contents

- [Physical Twin Mockup](#physical-twin-mockup)
- [Contents](#contents)
- [Controller](#controller)
- [Robot arm mockup](#robot-arm-mockup)

# Controller

The controller in [controller.py](/physical_twin_mockup/controller/controller.py) is responsible for sending the different operations to the robot arm. It inherits from the [base controller](/base_controller/base_controller.py) and uses the same control messages as the base controller.

# Robot arm mockup

The robot arm mockup in [robot_arm_mockup.py](/physical_twin_mockup/robot_arm_mockup/robot_arm_mockup.py) is responsible for simulating the behavior of the robot arm. The robot arm mockup listens for incoming messages from the controller and publishes it state via the routing key ```ROUTING_KEY_STATE```. 

The robot arm mockup can be configured in a [startup file](/startup/startup.conf):

- ```rmq_config```: The RabbitMQ configuration.
- ```initial_q```: The initial joint angles of the robot arm.
- ```missing_blocks```: The number of missing blocks from the task specification. This is used to simulate the behavior of the robot arm when it is not able to pick up a block from the plate due to a missing block.
- ```speedup```: The speedup factor of the robot arm.
- ```publish_freq```: The frequency at which the robot arm publishes its state.