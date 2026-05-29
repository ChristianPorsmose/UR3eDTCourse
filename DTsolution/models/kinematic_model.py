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

        # Experiment variabless
        self.start_time = 0.0
        self.stop_time = 0.0

    def _determine_angle_position_function(self, x0: np.ndarray, v0: np.ndarray, xtarget: np.ndarray):
        # Determine angle differences
        delta_x = xtarget - x0

        # Determine max delta_x
        max_delta_index = np.argmax(np.abs(delta_x))
        max_delta_x = delta_x[max_delta_index]

        # Enforce scalars for projections
        v0_proj = v0[max_delta_index] * np.sign(max_delta_x) if max_delta_x != 0 else 0.0
        v0_mag = np.abs(v0_proj)
        
        # Determine whether we are in trapezoidal or triangular velocity profile CASE
        d_cruise = np.abs(max_delta_x) + v0_proj**2/(2*self.acceleration) - self.v_max**2 / self.acceleration
        triangular_case = d_cruise <= 0

        delta_t = -1
        t_acc = -1
        t_dec = -1
        t_cruise = 0

        if triangular_case:
            v_limit = np.sqrt(np.abs(max_delta_x) * self.acceleration + v0_proj**2/2)
            delta_t = (2*v_limit - v0_proj)/self.acceleration
            t_acc = (v_limit - v0_proj) / self.acceleration
            t_dec = v_limit / self.acceleration
        else: # trapezoidal case
            t_cruise = d_cruise / self.v_max
            delta_t = (2*self.v_max - v0_proj) / self.acceleration + t_cruise
            t_acc = (self.v_max - v0_proj) / self.acceleration
            t_dec = self.v_max / self.acceleration

        # Avoid 0/0 division. Mathematically, a_lim_i always resolves to self.acceleration anyway.
        denominator = t_dec - t_acc
        a_lim_i = v0_proj / denominator if denominator != 0 else self.acceleration
        
        v_lim_i = a_lim_i * t_dec
        v_lim_i_mag = np.abs(v_lim_i)
        
        # Proportional scaling so all joints finish exactly at the same time
        dir_x = delta_x / np.abs(max_delta_x) if max_delta_x != 0 else np.zeros_like(delta_x)

        # Position function
        def x(t):
            t1 = t - t_acc
            t2 = t1 - t_cruise

            def d_acc(t):
                return v0_mag * t + (self.acceleration * t**2)/2
            
            def d_cruise(t):
                return v_lim_i_mag * t

            def d_dec(t):
                return t * (v_lim_i_mag - t*self.acceleration) + t**2 * self.acceleration/2

            conditions = [
                t <= 0,
                t <= t_acc,
                t < delta_t - t_dec,
                t < delta_t
            ]

            # Calculate magnitude shape
            choices = [
                0.0,
                d_acc(t),
                d_acc(t_acc) + d_cruise(t1),
                d_acc(t_acc) + d_cruise(t_cruise) + d_dec(t2)
            ]

            # Apply directionality back to the final displacement
            return x0 + dir_x * np.select(conditions, choices, default=np.abs(max_delta_x))

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

            # Calculate magnitude shape for the velocity profile
            choices = [
                0.0,
                v0_mag + self.acceleration * t,    
                v_lim_i_mag,                       
                v_lim_i_mag - self.acceleration * t2 
            ]

            # Apply directionality back to the velocity
            return dir_x * np.select(conditions, choices, default=0.0)

        return x, xderiv, delta_t


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
        self.angle_position_function, self.angle_velocity_function, self.movement_duration = self._determine_angle_position_function(self.start_joint_angles, self.start_joint_velocities, self.commanded_joint_angles)
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