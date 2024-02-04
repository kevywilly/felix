import numpy as np
import math
class MecanumRobot:
    def __init__(self, wheel_radius, L, W, max_rpm):
        """
        Initialize the MecanumRobot object.

        Args:
        - wheel_radius: Radius of the wheels (m)
        - L: Distance between the front and rear wheels (m)
        - W: Distance between the left and right wheels (m)
        """
        self.wheel_radius = wheel_radius
        self.L = L
        self.W = W
        self.meters_per_rotation = 2*wheel_radius*math.pi
        self.max_rpm = max_rpm
        

    def forward_kinematics(self, v_x, v_y, omega):
        """
        Perform forward kinematics to calculate wheel velocities given desired robot velocities.

        Args:
        - v_x: Desired linear velocity along the x-axis (m/s)
        - v_y: Desired linear velocity along the y-axis (m/s)
        - omega: Desired angular velocity about the z-axis (rad/s)

        Returns:
        - v_fl: Velocity of the front-left wheel (m/s)
        - v_fr: Velocity of the front-right wheel (m/s)
        - v_rl: Velocity of the rear-left wheel (m/s)
        - v_rr: Velocity of the rear-right wheel (m/s)
        """
        v_fl = v_x - v_y - (self.L + self.W) * omega
        v_fr = v_x + v_y + (self.L + self.W) * omega
        v_rl = v_x + v_y - (self.L + self.W) * omega
        v_rr = v_x - v_y + (self.L + self.W) * omega

        return v_fl, v_fr, v_rl, v_rr

    
    def inverse_kinematics(self, v_fl, v_fr, v_rl, v_rr):
        """
        Perform inverse kinematics to calculate robot velocities given wheel velocities.

        Args:
        - v_fl: Velocity of the front-left wheel (m/s)
        - v_fr: Velocity of the front-right wheel (m/s)
        - v_rl: Velocity of the rear-left wheel (m/s)
        - v_rr: Velocity of the rear-right wheel (m/s)

        Returns:
        - v_x: Linear velocity along the x-axis (m/s)
        - v_y: Linear velocity along the y-axis (m/s)
        - omega: Angular velocity about the z-axis (rad/s)
        """
        v_x = (v_fl + v_fr + v_rl + v_rr) / 4
        v_y = (-v_fl + v_fr + v_rl - v_rr) / 4
        omega = (-v_fl + v_fr - v_rl + v_rr) / (4 * (self.L + self.W))

        return v_x, v_y, omega
    
    def max_linear_velocity(self)-> float:
        v_all = self.rpm_to_mps(self.max_rpm)
        v, _, _ = self.inverse_kinematics(v_all, v_all, v_all, v_all)
        return v
    
    def max_angular_velocity(self) -> float:
        v_all = self.rpm_to_mps(self.max_rpm)
        _, _, omega = self.inverse_kinematics(-v_all, v_all, -v_all, v_all)
        return omega

    def rpm_to_mps(self, rpm: float):
        rps = rpm/60
        mps = rps*self.meters_per_rotation
        
        return mps
    
    def mps_to_rpms(self, mps: float):
        rps = mps/self.meters_per_rotation
        rpm = rps*60
        return rpm
    
    def mps_to_motor_power(self, mps):
        return self.mps_to_rpms(mps)/self.max_rpm
        

"""
# Example usage:
wheel_radius = 0.0485  # Radius of the wheels (m)
L = 0.15  # Distance between front and rear wheels (m)
W = 0.229  # Distance between left and right wheels (m)

robot = MecanumRobot(wheel_radius, L, W)

# Forward kinematics: Calculate wheel velocities for desired robot velocities
v_x = 0  # Linear velocity along the x-axis (m/s)
v_y = 0  # Linear velocity along the y-axis (m/s)
omega = 3  # Angular velocity about the z-axis (rad/s)
v_fl, v_fr, v_rl, v_rr = robot.forward_kinematics(v_x, v_y, omega)

wheel_velocities = np.array([v_fl,v_fr,v_rl,v_rr])

print("Meters Per Rotation:", robot.meters_per_rotation)
print("Forward kinematics:", wheel_velocities, 'm/s')
print("Forward kinematics:", robot.mps_to_rpms(wheel_velocities), 'rpms')
print("Forward kinematics:", robot.mps_to_motor_power(wheel_velocities, 205), 'power')


# Inverse kinematics: Calculate robot velocities for given wheel velocities
v_x, v_y, omega = robot.inverse_kinematics(v_fl, v_fr, v_rl, v_rr)
print("\nInverse kinematics:")
print("v_x:", v_x, "m/s")
print("v_y:", v_y, "m/s")
print("omega:", omega, "rad/s")

mv_x, mv_y, m_omega = robot.max_linear_velocity(205)
print("Max Linear Velocity", mv_x)

mv_x, mv_y, m_omega = robot.max_angular_velocity(205)
print("Max Linear Velocity", m_omega)

"""
