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
            prev_state = self.ekf.state
            x_i, x_j, sigma_ij = self.ekf.predict(control)
            motion = self.ekf.state - prev_state
            self.graph_slam.add_motion_constraint(i, x_i, x_j, sigma_ij)
            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)

            # Obervations
            observations = get_observations(
                logfile.scan_data[i],
                self.ekf.depth_jump, self.ekf.minimum_valid_distance, self.ekf.cylinder_offset,
                self.ekf.state, self.ekf.scanner_displacement,
                reference_cylinders, self.ekf.max_cylinder_distance)
            self.matched_ref_cylinders.append([m[1] for m in observations])

            for j in range(len(observations)):
                measurment, landmark = observations[j]
                Q = self.ekf.correct(measurment, landmark)
                self.graph_slam.add_pose(self.state)
                self.graph_slam.add_observation_constraint(i+1, measurment, landmark, Q)

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

def lego_robot_test(data_dir):

    #Initialize EKF
    ekf = EKF()  # dummy values
    ekf.load_json(os.path.join(data_dir,"config.json"))

    #Initialize Graph SLAM
    graph_slam = GraphSLAM()
    graph_slam.add_pose(ekf.state.copy())
    graph_slam.scanner_displacement = ekf.scanner_displacement
    
    # Read Robot Data
    logfile = LegoLogfile()
    logfile.read(os.path.join(data_dir,"robot4_motors.txt"))
    logfile.read(os.path.join(data_dir,"robot4_scan.txt"))
    logfile.read(os.path.join(data_dir,"robot_arena_landmarks.txt"))

    # Run Main Sytyem
    system = System(ekf, graph_slam)
    system.run(logfile)
    system.write_ekf_results(os.path.join(data_dir,"results_ekf.txt"))
    system.write_graph_slam_results(os.path.join(data_dir,"results_graph_slam.txt"))


if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    lego_robot_test("Data/LegoRobot")


##############################################
#   DEPRICATED
##############################################
    # config_json = """
    # {
    #     "ticks_to_mm": 0.349,
    #     "cylinder_offset": 90.0,
    #     "depth_jump": 100.0,
    #     "minimum_valid_distance": 20.0,
    #     "max_cylinder_distance": 300.0,
    #     "robot_width": 155.0,
    #     "scanner_displacement": 30.0,
    #     "control_motion_factor": 0.35,
    #     "control_turn_factor": 0.6,
    #     "measurement_distance_stddev": 200.0,
    #     "measurement_angle_stddev": 0.2617993877991494
    # }
    # """

