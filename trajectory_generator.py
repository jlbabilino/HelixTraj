from typing import List, Dict

from casadi import *

from drive.swerve_drive import swerve_drive

class trajectory_generator:
    def __init__(self, drive: swerve_drive):
        self.drive = drive

    def generate(self, waypoints: List[Dict]):
        # Segments refer to the short parts of a trajectories between
        # succesive sample points while trajectory segments are segments
        # of the path where the robot is guaranteed to go through both endpoints
        # and at certain angles. N is a number of segments.

        # Split path into trajectory segments
        self.N_per_trajectory_segment = 100
        self.trajectory_segment_count = len(waypoints) - 1
        self.N_total = self.N_per_trajectory_segment * self.trajectory_segment_count

        self.opti = Opti()

        # Minimize time
        # List of time elapsed for each segment
        Ts, dts = [], []
        for k in range(self.trajectory_segment_count):
            T = self.opti.variable()
            dt = T / self.N_per_trajectory_segment
            Ts.append(T)
            dts.append(dt)

            self.opti.subject_to(T >= 0)
            # Most FRC paths take a few seconds
            self.opti.set_initial(T, 5)
        self.opti.minimize(sum(Ts))

        # Initialize variables
        # Each column of X corresponds to a sample point: [x, y, theta (heading), velocity x-component,
        # velocity y-component, angular velocity]
        self.X = self.opti.variable(6, self.N_total+1)

        self.x = self.X[0,:]
        self.y = self.X[1,:]
        self.theta = self.X[2,:]
        self.vx = self.X[3,:]
        self.vy = self.X[4,:]
        self.omega = self.X[5,:]

        # Each column of U corresponds to a segment between two sample points in the trajectory.
        # U stores variables that have meaning between sample points. For example, when the
        # robot goes between two sample points, it starts and ends with a different velocity.
        # So the robot accelerated during that segment.
        self.U = self.opti.variable(3, self.N_total)

        self.ax = self.U[0,:]
        self.ay = self.U[1,:]
        self.alpha = self.U[2,:]

        # Add dynamics constraint
        # X, U are columns of self.X, self.Y
        dynamics = lambda X, U: vertcat(
            X[3], # vx
            X[4], # vy
            X[5], # omega
            U[0], # ax
            U[1], # ay
            U[2]  # alpha
        )

        for k in range(self.N_total):
            # x_k+1 = x_k + vx_k * dt
            x_next = self.X[:, k] + dynamics(self.X[:, k], self.U[:, k]) * dts[k // self.N_per_trajectory_segment]
            self.opti.subject_to(self.X[:, k + 1] == x_next)

        # Set initial guess
        x_init, y_init, theta_init = generate_initial_trajectory(waypoints, self.N_per_trajectory_segment)
        self.opti.set_initial(self.x, x_init)
        self.opti.set_initial(self.y, y_init)
        self.opti.set_initial(self.theta, theta_init)

        # Add constraints
        self.drive.add_kinematics_constraint(self.opti, self.theta, self.vx, self.vy, self.omega, self.ax, self.ay, self.alpha, self.N_total)
        self.add_boundry_constraint()
        self.add_waypoint_constraint(waypoints)

        self.opti.solver("ipopt")
        solution = self.opti.solve()

        solution_dts = []
        for k in range(self.trajectory_segment_count):
            solution_dts.append(solution.value(Ts[k] / self.N_per_trajectory_segment)) # TODO: Try changing this to solution.value(dts[k])

        solution_x = solution.value(self.x)
        solution_y = solution.value(self.y)
        solution_theta = solution.value(self.theta)
        solution_vx = solution.value(self.vx)
        solution_vy = solution.value(self.vy)
        solution_omega = solution.value(self.omega)

        ts = [0]
        for solution_dt in solution_dts:
            for k in range(self.N_per_trajectory_segment):
                ts.append(ts[-1] + solution_dt)
        trajectory = []
        for j in range(len(solution_x)):
            # trajectory.append([str(round(ts[j],4)), str(round(x[j],4)), str(round(y[j],4)), str(round(theta[j],4)), str(round(vx[j],4)), str(round(vy[j],4)), str(round(omega[j],4))])
            trajectory.append({'ts': round(ts[j],4), 'x': round(solution_x[j],4), 'y': round(solution_y[j],4), 'heading': round(solution_theta[j],4), 'vx': round(solution_vx[j],4), 'vy': round(solution_vy[j],4), 'omega': round(solution_omega[j],4)})

        return trajectory

    def add_boundry_constraint(self):
        """
            Add constraints that represent the fact that the robot is still when starting and
            ending a path. This could be modified so that the robot is required to have a
            certain starting and ending velocity.
        """
        for k in [-1, 0]:
            self.opti.subject_to(self.vx[k] == 0)
            self.opti.subject_to(self.vy[k] == 0)
            self.opti.subject_to(self.omega[k] == 0)

    def add_waypoint_constraint(self, waypoints: List[Dict]):
        """
            Adds constraints that ensure the robot goes through each waypoint.

            Arguments:
                waypoints -- waypoints to add constraints for
        """
        for k in range(self.trajectory_segment_count + 1):
            index = k * self.N_per_trajectory_segment
            self.opti.subject_to(self.x[index] == waypoints[k]['x'])
            self.opti.subject_to(self.y[index] == waypoints[k]['y'])
            self.opti.subject_to(self.theta[index] == waypoints[k]['heading'])

def generate_initial_trajectory(waypoints: List[Dict], N_per_trajectory_segment: int):
    x, y, theta = [], [], []
    for k in range(len(waypoints) - 1):
        x.extend(np.linspace(waypoints[k]['x'], waypoints[k+1]['x'], N_per_trajectory_segment, False).tolist())
        y.extend(np.linspace(waypoints[k]['y'], waypoints[k+1]['y'], N_per_trajectory_segment, False).tolist())
        theta.extend(np.linspace(waypoints[k]['heading'], waypoints[k+1]['heading'], N_per_trajectory_segment, False).tolist())
    x.append(waypoints[-1]['x']) # last point
    y.append(waypoints[-1]['y'])
    theta.append(waypoints[-1]['heading'])
    return x, y, theta