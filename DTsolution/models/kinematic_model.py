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
        self.current_acceleration_duration = 0.0

        # Experiment variables
        self.start_time = 0.0
        self.stop_time = 0.0

    def _determine_movement_duration_and_acceleration_duration(self, start_angles: list, end_angles: list) -> float:
        # Determine angle differences
        delta_x = np.abs(np.array(end_angles) - np.array(start_angles))
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
        acceleration_duration = t_acc

        # NON-IDEAL CASE check: t_acc > T/3
        if t_acc > t_cruise:
            # NON-IDEAL CASE: t_acc > T/3
            tau = np.sqrt(max_delta_x/(2*self.a_max))
            v = self.a_max * tau

            # Overwrite IDEAL values with NON-IDEAL values
            movement_duration = 3*tau
            acceleration_duration = tau

        return movement_duration, acceleration_duration


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
        self.current_movement_duration, self.current_acceleration_duration = self._determine_movement_duration_and_acceleration_duration(self.current_joint_angles, self.commanded_joint_angles)

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
                # Accelerating
                if t_rel <= self.current_acceleration_duration:
                    self.current_joint_velocities += self.a_max * step_size
                # Cruising
                elif t_rel < self.current_movement_duration - self.current_acceleration_duration:
                    # No change in velocity
                    pass
                # Decelerating
                else:
                    self.current_joint_velocities -= self.a_max * step_size
                self.current_joint_angles += self.current_joint_velocities * step_size

    def fmi2GetJointPositions(self) -> list: 
        return self.current_joint_angles.copy().tolist()
        
    def fmi2GetJointVelocities(self):
        # We now calculate this for free during fmi2DoStep
        return self.current_joint_velocities.copy().tolist()
        
    def fmi2Terminate(self):
        pass