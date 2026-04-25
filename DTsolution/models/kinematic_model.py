import numpy as np

class KinematicModel:
    def __init__(self, max_velocity=np.deg2rad(60.0), max_acceleration=np.deg2rad(80.0)):
        self.time = 0.0 # All time measured in seconds
        
        # System Limits (Replace defaults with your actual robot specs)
        self.v_max = max_velocity
        self.acceleration = max_acceleration

        # State Variables
        self.current_joint_angles = np.array([0.0] * 6)
        self.current_joint_velocities = np.array([0.0] * 6)

        # Inputs
        self.commanded_joint_angles = np.array([0.0] * 6)

        # Movement variables
        self.moving = False
        self.current_movement_start_time = 0.0
        self.current_movement_duration = 0.0
        self.angle_position_function = lambda x: np.array([0.0] * 6) # Return standard position always
        self.angle_velocity_function = lambda x: np.array([0.0] * 6) # Return standard velocity always

        # Experiment variables
        self.start_time = 0.0
        self.stop_time = 0.0

    def _determine_angle_position_function(self, x0: np.ndarray, v0: np.ndarray, xtarget: np.ndarray):
        # Determine angle differences
        delta_x = xtarget - x0

        # Determine max delta_x
        max_delta_index = np.argmax(np.abs(delta_x))
        max_delta_x = delta_x[max_delta_index]

        v0_proj = v0[max_delta_index] * np.sign(max_delta_x) if max_delta_x != 0 else v0
        
        # Determine whether we are in trapezoidal or triangular velocity profile CASE
        d_cruise = np.abs(max_delta_x) + v0_proj**2/(2*self.acceleration) - self.v_max**2 / self.acceleration
        triangular_case = d_cruise <= 0

        delta_t = -1

        if triangular_case:
            v_limit = np.sqrt(np.abs(max_delta_x) * self.acceleration + v0_proj**2/2)
            delta_t = (2*v_limit - v0_proj)/self.acceleration

        else: # trapezoidal case
            t_cruise = d_cruise / self.v_max
            delta_t = (2*self.v_max - v0_proj) / self.acceleration + t_cruise

        abs_delta_x = np.abs(delta_x)
        dir_x = np.sign(delta_x)
        dir_x = np.where(dir_x == 0, 1, dir_x) # Prevent dropping initial velocity on 0-distance joints
        v0_mag = np.abs(v0)

        # Calculate required v_limit magnitude for each joint to finish at exactly delta_t
        a = -1/self.acceleration
        b = delta_t - v0_mag/self.acceleration
        c = -v0_mag**2/(2*self.acceleration) - abs_delta_x
        d = b**2 - 4*a*c

        v_limit1 = (-b + np.sqrt(d))/(2*a)
        v_limit2 = (-b - np.sqrt(d))/(2*a)
        v_limit_mag = np.where(np.abs(v_limit1) < np.abs(v_limit2), v_limit1, v_limit2)

        # 3. Time intervals (Now guaranteed positive)
        t_acc = (v_limit_mag - v0_mag)/self.acceleration
        t_dec = v_limit_mag/self.acceleration
        t_cruise = delta_t - t_acc - t_dec

        # 4. Displacements (FIXED: Replaced self.v_max with v_limit_mag)
        d_acc = v0_mag * t_acc + (v_limit_mag - v0_mag)/2 * t_acc
        d_cruise = v_limit_mag * t_cruise


        # Position function
        def x(t):
            t1 = t - t_acc
            t2 = t1 - t_cruise

            conditions = [
                t <= 0,
                t <= t_acc,
                t < delta_t - t_dec,
                t < delta_t
            ]

            # Calculate magnitude shape
            choices = [
                0,
                v0_mag * t + (self.acceleration * t**2)/2,
                d_acc + v_limit_mag * t1,
                d_acc + d_cruise + t2 * (v_limit_mag - t2*self.acceleration) + t2**2 * self.acceleration/2
            ]

            # 5. Apply directionality back to the final displacement
            return x0 + dir_x * np.select(conditions, choices, default=abs_delta_x)

        # Velocity function
        def xderiv(t):
            t1 = t - t_acc
            t2 = t1 - t_cruise

            conditions = [
                t <= 0,
                t <= t_acc,
                t < delta_t - t_dec,
                t < delta_t
            ]

            # Calculate magnitude shape
            choices = [
                0,
                v0_mag + (self.acceleration * t),
                v_limit_mag,
                (v_limit_mag - 2*t2*self.acceleration) + t2 * self.acceleration
            ]

            # 5. Apply directionality back to the final displacement
            return dir_x * np.select(conditions, choices, default=0.0)

        return x, xderiv


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

    def fmi2StartMovement(self):
        # Save variables needed to calculate the new trajectory
        self.start_joint_angles = self.current_joint_angles
        self.start_joint_velocities = self.current_joint_velocities
        self.current_movement_start_time = self.time

        # Calculate the acc, cruise and deacc durations for the trajectory
        self.angle_position_function, self.angle_velocity_function = self._determine_angle_position_function(self.start_joint_angles, self.start_joint_velocities, self.commanded_joint_angles)
        self.moving = True

    def fmi2DoStep(self, current_time: float, step_size: float): 
        self.time = current_time + step_size

        t = self.time - self.current_movement_start_time
        self.current_joint_angles = self.angle_position_function(t)
        self.current_joint_velocities = self.angle_velocity_function(t)

    def fmi2GetJointPositions(self) -> list: 
        return self.current_joint_angles.copy().tolist()
        
    def fmi2GetJointVelocities(self):
        # We now calculate this for free during fmi2DoStep
        return self.current_joint_velocities.copy().tolist()
        
    def fmi2Terminate(self):
        pass