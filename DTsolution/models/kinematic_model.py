import numpy as np

class KinematicModel:
    def __init__(self, max_velocity=np.deg2rad(60.0), max_acceleration=np.deg2rad(80.0)):
        self.time = 0.0 # All time measured in seconds
        
        # System Limits (Replace defaults with your actual robot specs)
        self.v_max = max_velocity
        self.a_max = max_acceleration

        # State Variables
        self.current_joint_angles = np.array([0.0] * 6)
        self.current_joint_velocities = np.array([0.0] * 6)

        # Inputs
        self.commanded_joint_angles = np.array([0.0] * 6)

        # Movement variables
        self.moving = False
        self.current_movement_start_time = 0.0
        self.current_movement_duration = 0.0
        self.current_acceleration_durations = np.array([0.0] * 6)

        # Experiment variables
        self.start_time = 0.0
        self.stop_time = 0.0

    def _determine_movement_duration_and_acceleration_duration(self, start_angles: list, end_angles: list) -> float:
        # Determine angle differences
        delta_x = np.abs(np.array(end_angles) - np.array(start_angles))
        max_delta_index = np.argmax(delta_x)
        max_delta_x = np.max(delta_x)

        # IDEAL CASE: t_acc <= T/3
        # Determine acceleration time
        t_acc = self.v_max / self.a_max
        delta_x_acc = (self.v_max / 2) * t_acc

        # Determine cruise time
        t_cruise = (max_delta_x - 2 * delta_x_acc) / self.v_max
        delta_x_cruise = self.v_max * t_cruise

        # Set IDEAL values
        movement_duration = t_cruise + 2 * t_acc
        max_acceleration_duration = t_acc

        # NON-IDEAL CASE check: t_acc > T/3
        if t_acc > t_cruise:
            # NON-IDEAL CASE: t_acc > T/3
            tau = np.sqrt(max_delta_x/(self.a_max))
            v = self.a_max * tau

            # Overwrite IDEAL values with NON-IDEAL values
            movement_duration = 2*tau
            max_acceleration_duration = tau

        acceleration_durations = np.zeros_like(self.current_joint_velocities)
        acceleration_durations[max_delta_index] = max_acceleration_duration
        # acceleration_coefficients = np.zeros_like(self.current_joint_velocities)
        # acceleration_coefficients[max_delta_index] = self.a_max

        # Calculate the acceleration durations and acceleration coefficients of the rest of the joints to make sure they all finish the movement at the same time
        max_joint_velocities = (movement_duration + np.sqrt(movement_duration**2 - 4 * delta_x / self.a_max)) / (2/self.a_max)
        acceleration_durations = max_joint_velocities / self.a_max

        return movement_duration, acceleration_durations


    def fmi2Instantiate(self):
        self.time = 0.0
        self.current_joint_angles = np.array([0.0] * 6)
        self.current_joint_velocities = np.array([0.0] * 6)
        self.commanded_joint_angles = np.array([0.0] * 6)
        self.moving = False

    def fmi2SetupExperiment(self, start_time: float, stop_time: float):
        self.start_time = start_time
        self.stop_time = stop_time

    def fmi2SetCommandedJointAngles(self, angles: list):
        # Setup variables for move to be made
        self.commanded_joint_angles = np.array(angles)
        self.current_movement_duration, self.current_acceleration_durations = self._determine_movement_duration_and_acceleration_duration(self.current_joint_angles, self.commanded_joint_angles)

    def fmi2StartMovement(self):
        self.moving = True
        self.current_movement_start_time = self.time
        # Capture the exact starting position for this movement

    def fmi2DoStep(self, current_time: float, step_size: float): 
        self.time = current_time + step_size

        if self.moving:
            t_rel = self.time - self.current_movement_start_time
            
            # Check if we have surpassed the total duration
            if t_rel >= self.current_movement_duration:
                self.current_joint_angles = self.commanded_joint_angles.copy()
                self.current_joint_velocities = np.array([0.0] * 6)
                self.moving = False
            else: # Else keep on doing the movement
                # Determine which joints are accelerating, cruising and decelerating
                is_accelerating = t_rel <= self.current_acceleration_durations
                is_decelerating = t_rel >= (self.current_movement_duration - self.current_acceleration_durations)
                is_cruising = ~(is_accelerating | is_decelerating)

                # Get acceleration/velocity sign
                delta_angles = self.commanded_joint_angles - self.current_joint_angles
                sign = delta_angles / (np.abs(delta_angles) + 1e-6)

                # Update volicities based on masks from above
                # Accelerating: add acceleration
                self.current_joint_velocities[is_accelerating] += self.a_max * step_size * sign[is_accelerating]

                # Decelerating: subtract acceleration
                self.current_joint_velocities[is_decelerating] -= self.a_max * step_size * sign[is_decelerating]

                # Cruising: (Implicitly does nothing, no code needed)

                # Update positions based on the velocity of each joint
                self.current_joint_angles += self.current_joint_velocities * step_size

    def fmi2GetJointPositions(self) -> list: 
        return self.current_joint_angles.copy().tolist()
        
    def fmi2GetJointVelocities(self):
        # We now calculate this for free during fmi2DoStep
        return self.current_joint_velocities.copy().tolist()
        
    def fmi2Terminate(self):
        pass