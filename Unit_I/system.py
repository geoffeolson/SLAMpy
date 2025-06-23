from lego_robot import *
from math import sin, cos, pi, sqrt
import numpy as np
import os
import json
from slam_d_library import get_observations, write_cylinders
from extended_kalman_filter import EKF
from graph_slam import GraphSLAM
from lego_robot import LegoLogfile
from slam_d_library import get_observations

class System:
    def __init__(self, ekf, graph_slam):

        # Dependencies
        self.ekf = ekf
        self.graph_slam = graph_slam

        # Logging containers
        self.states = []
        self.covariances = []
        self.matched_ref_cylinders = []

    # Main System Loop
    def run(self, logfile):
        reference_cylinders = [l[1:3] for l in logfile.landmarks]
        for i in range(len(logfile.motor_ticks)):

            # Odometry
            control = np.array(logfile.motor_ticks[i]) * self.ekf.ticks_to_mm
            x_i, x_j, Sigma_ij = self.ekf.predict(control)
            self.graph_slam.add_motion_constraint(x_j, Sigma_ij)
            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)

            # Obervations
            observations = get_observations(
                logfile.scan_data[i],
                self.ekf.depth_jump, self.ekf.minimum_valid_distance, self.ekf.cylinder_offset,
                self.ekf.state, self.ekf.scanner_displacement,
                reference_cylinders, self.ekf.max_cylinder_distance)
            self.matched_ref_cylinders.append([m[1] for m in observations])
            for obs in observations:
                pose_index, z_ik, landmark_index, Sigma_ik = obs
                self.ekf.correct(pose_index, z_ik, landmark_index, Sigma_ik)
                self.graph_slam.add_observation_constraint(pose_index, landmark_index, z_ik, Sigma_ik)

    def write_ekf_results(self, filename="Results_EKF.txt"):
        from math import sqrt, cos, sin
        from slam_d_library import write_cylinders
        with open(filename, "w") as f:
            for i in range(len(self.states)):
                x = tuple(self.states[i] + [
                    self.ekf.scanner_displacement * cos(self.states[i][2]),
                    self.ekf.scanner_displacement * sin(self.states[i][2]),
                    0.0])
                f.write(f"F {x[0]} {x[1]} {x[2]}\n")
                e = self.ekf.get_error_ellipse(self.covariances[i])
                q = (e + (sqrt(self.covariances[i][2,2]),))
                f.write(f"E {q[0]} {q[1]} {q[2]} {q[3]}\n")
                write_cylinders(f, "W C", self.matched_ref_cylinders[i])

    def write_graph_slam_results(self, filename="Results_Graph_Slam.txt"):
        with open(filename, "w") as f:
            for x, y, θ in self.graph_slam.poses:
                f.write(f"F {x} {y} {θ}\n")
        print(f"Wrote {len(self.graph_slam.poses)} poses and constraints to {filename}")

def lego_robot_test():

    #Initialize EKF
    config_json = """
    {
        "ticks_to_mm": 0.349,
        "cylinder_offset": 90.0,
        "depth_jump": 100.0,
        "minimum_valid_distance": 20.0,
        "max_cylinder_distance": 300.0,
        "robot_width": 155.0,
        "scanner_displacement": 30.0,
        "control_motion_factor": 0.35,
        "control_turn_factor": 0.6,
        "measurement_distance_stddev": 200.0,
        "measurement_angle_stddev": 0.2617993877991494
    }
    """
    config = json.loads(config_json)
    initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * np.pi])
    initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * np.pi) ** 2])
    ekf = EKF(initial_state.copy(), initial_covariance, 0, 0, 0, 0, 0, 0)  # dummy values
    ekf.read_json(config)

    #Initialize Graph SLAM
    graph_slam = GraphSLAM()
    graph_slam.add_pose(initial_state.copy())
    
    # Read Robot Data
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")

    # Run Main Sytyem
    system = System(ekf, graph_slam)
    system.run(logfile)
    system.write_ekf_results("Results_EKF.txt")
    system.write_graph_slam_results("Results_Graph_Slam.txt")


if __name__ == '__main__':
        # Get the directory of the current script
    python_file_directory = os.path.dirname(os.path.abspath(__file__))
    print("Current Directory " + os.chdir(python_file_directory))
    lego_robot_test()

