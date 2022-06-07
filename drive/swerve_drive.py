from math import hypot
from casadi import *

class swerve_drive:
    def __init__(self, wheelbase_x, wheelbase_y, length, width, mass, moi, omega_max, tau_max, wheel_radius):
        """
            Initializes a swerve drive model with given characteristics. Given the 2D coordinate system
            of the field, the robot coordinate system is defined with its center placed on the center of
            the robot, and the x-axis points directly towards the front face of the robot. The y-axis
            points 90 degress counter-clockwise. The following diagram shows the coordinate system and
            some related dimensions:

             _________________________________________                    \ 
            /                                         \                   |
            |   00                              00    |                   |
            |   00  rear_left        front_left  00   |   \               |
            |   00               y                00  |   |               |
            |                    ^                    |   |               |
            |                    |                    |   |               |
            |                    +----> x             |   | wheelbase_y   | width
            |                                         |   |               |
            |                                         |   |               |
            |  00                                00   |   |               |
            |   00  rear_right      front_right  00   |   /               |
            |    00                              00   |                   |
            |                                         |                   |
            \_________________________________________/                   /
                 <---------- wheelbase_x -------->
            <----------------- length ---------------->

            Arguments:
                wheelbase_x  -- when facing side of robot, the horizontal distance between modules
                wheelbase_y  -- when facing front of robot, the horizontal distance between modules
                length       -- when facing side of robot, the horizontal distance across bumpers
                width        -- when facing front of robot, the horizontal distance across bumpers
                mass         -- mass of robot
                moi          -- moment of inertia of robot about axis of rotation (currently through
                                center of robot coordinate system)
                omega_max    -- maximum angular velocity of wheels (not related to direction controlling motor; motor
                                that controls speed)
                tau_max      -- maximum torque of wheels (similar to omega_max)
                wheel_radius -- radius of wheels
        """
        self.wheelbase_x = wheelbase_x
        self.wheelbase_y = wheelbase_y
        self.length = length
        self.width = width
        self.mass = mass
        self.moi = moi
        self.omega_max = omega_max
        self.tau_max = tau_max
        self.wheel_radius = wheel_radius
        self.force_max = tau_max / wheel_radius

    def solve_module_positions(self, k, theta):
        """
            Calculates the position of the the modules relative to the robot coordinate system at an instant.
            Given an index out of the sequence of sample points, returns a list containing the the four positions.

            Arguments:
                k -- index of sample point on trajectory
                theta -- the instantaneous heading of the robot on the kth sample point
        """
        # module_positions -- positions of each module relative to robot coordinate system
        # module_angles -- angle between x-axis of robot coordinate system and vector
        #                  pointing from center of robot to module.
        module_positions, module_angles = [], []
        d = hypot(self.wheelbase_x, self.wheelbase_y)
        # (1, 1), (1, -1), (-1, 1), (-1, -1)
        for a in [1, -1]:
            for b in [1, -1]:        
                module_angles.append(atan2(self.wheelbase_y*a, self.wheelbase_x*b))
        for module_theta in module_angles:
            module_positions.append([d*cos(module_theta+theta[k]), d*sin(module_theta+theta[k])])
        return module_positions

    def add_kinematics_constraint(self, solver, theta, vx, vy, omega, ax, ay, alpha, N):
        for k in range(N):
            modules = self.solve_module_positions(k, theta)

            max_v = self.omega_max * self.wheel_radius
            for module in modules:
                m_vx = vx[k] + module[0] * omega[k]
                m_vy = vy[k] + module[1] * omega[k]
                solver.subject_to(m_vx * m_vx + m_vy * m_vy < max_v * max_v)

            Fx = solver.variable(4)
            Fy = solver.variable(4)

            T = []
            for j in range(4):
                    T.append(modules[j][1] * Fx[j] - modules[j][0] * Fy[j])
                    solver.subject_to(Fx[j] * Fx[j] + Fy[j] * Fy[j] < self.force_max * self.force_max)

            solver.subject_to(ax[k] * self.mass == Fx[0] + Fx[1] + Fx[2] + Fx[3])
            solver.subject_to(ay[k] * self.mass == Fy[0] + Fy[1] + Fy[2] + Fy[3])
            solver.subject_to(alpha[k] * self.moi == sum(T))