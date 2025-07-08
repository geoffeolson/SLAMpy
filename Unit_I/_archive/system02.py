from lego_robot import *
from math import sin, cos, pi, sqrt
import numpy as np
import os
os.chdir("Unit_I")
from slam_d_library import get_observations, write_cylinders
from extended_kalman_filter import EKF
from graph_slam import Graph

class System:
    def __init__(self):
        # Robot constants
        self.scanner_displacement = 30.0
        self.ticks_to_mm = 0.349
        self.robot_width = 155.0

        # Cylinder extraction and matching constants
        self.minimum_valid_distance = 20.0
        self.depth_jump = 100.0
        self.cylinder_offset = 90.0
        self.max_cylinder_distance = 300.0

        # Filter constants
        self.control_motion_factor = 0.35
        self.control_turn_factor = 0.6
        self.measurement_distance_stddev = 200.0
        self.measurement_angle_stddev = 15.0 / 180.0 * pi

        # Measured start position
        self.initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * pi])
        self.initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi)**2])

        self.graph_slam = Graph()
        self.graph_slam.add_pose(self.initial_state.copy())

        self.ekf = EKF(self.initial_state, self.initial_covariance,
                       self.robot_width, self.scanner_displacement,
                       self.control_motion_factor, self.control_turn_factor,
                       self.measurement_distance_stddev, self.measurement_angle_stddev)

        self.states = []
        self.covariances = []
        self.matched_ref_cylinders = []

        self.logfile = LegoLogfile()
        self.logfile.read("robot4_motors.txt")
        self.logfile.read("robot4_scan.txt")
        self.logfile.read("robot_arena_landmarks.txt")
        self.reference_cylinders = [l[1:3] for l in self.logfile.landmarks]

    def run(self):
        for i in range(len(self.logfile.motor_ticks)):
            control = np.array(self.logfile.motor_ticks[i]) * self.ticks_to_mm
            x_i, x_j, Sigma_ij = self.ekf.predict(control)

            observations = get_observations(
                self.logfile.scan_data[i],
                self.depth_jump, self.minimum_valid_distance, self.cylinder_offset,
                self.ekf.state, self.scanner_displacement,
                self.reference_cylinders, self.max_cylinder_distance)
            for j in range(len(observations)):
                self.ekf.correct(*observations[j])

            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            self.matched_ref_cylinders.append([m[1] for m in observations])

            self.graph_slam.add_motion_constraint(x_j, Sigma_ij)

    def write_ekf_results(self, filename="Results_EKF.txt"):
        with open(filename, "w") as f:
            for i in range(len(self.states)):
                x = tuple(self.states[i] + [self.scanner_displacement * cos(self.states[i][2]),
                                            self.scanner_displacement * sin(self.states[i][2]), 0.0])
                f.write(f"F {x[0]} {x[1]} {x[2]}\n")
                e = EKF.get_error_ellipse(self.covariances[i])
                q = (e + (sqrt(self.covariances[i][2,2]),))
                f.write(f"E {q[0]} {q[1]} {q[2]} {q[3]}\n")
                write_cylinders(f, "W C", self.matched_ref_cylinders[i])

    def write_graph_slam_results(self, filename="Results_Graph_Slam.txt"):
        with open(filename, "w") as f:
            for x, y, theta in self.graph_slam.poses:
                f.write(f"F {x} {y} {theta}\n")
        print(f"Wrote {len(self.graph_slam.poses)} poses and constraints to {filename}")

if __name__ == '__main__':
    system = System()
    system.run()
    system.write_ekf_results()
    system.write_graph_slam_results()
