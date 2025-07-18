from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
import numpy as np
import os
os.chdir("Unit_I")
from slam_d_library import get_observations, write_cylinders
from extended_kalman_filter import EKF
from graph_slam import Graph


def compute_relative_pose(x_i, x_j):
    """Compute the relative pose x_ij from pose x_i to pose x_j."""
    dx = x_j[0] - x_i[0]
    dy = x_j[1] - x_i[1]
    dtheta = x_j[2] - x_i[2]
    theta = x_i[2]
    rot_dx = cos(-theta) * dx - sin(-theta) * dy
    rot_dy = sin(-theta) * dx + cos(-theta) * dy
    rot_dtheta = (dtheta + pi) % (2 * pi) - pi
    x_ij = np.array([rot_dx, rot_dy, rot_dtheta])
    return x_ij

def compute_info_matrix(Sigma_ij):
    """Compute the information matrix Omega_ij from the covariance matrix Sigma_ij."""
    try:
        Omega_ij = np.linalg.inv(Sigma_ij)  # Information matrix
    except np.linalg.LinAlgError:
        Omega_ij = np.zeros_like(Sigma_ij)  # If singular, use zero matrix
    return Omega_ij

def add_motion_constraint(i, x_i, x_j, Sigma_ij, graph_slam):
    x_ij = compute_relative_pose(x_i, x_j)
    Omega_ij = compute_info_matrix(Sigma_ij)
    graph_slam.add_pose(x_j)
    graph_slam.add_constraint(i, i + 1, x_ij, Omega_ij)


def test():

    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])

    ###########################
    #      GRAPH SLAM
    ###########################
    graph_slam = Graph()
    graph_slam.add_pose(initial_state.copy())  # Pose x0

    ######################
    #       EKF 
    ######################
    ekf = EKF(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)
    states = []
    covariances = []
    matched_ref_cylinders = []
    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    for i in range(len(logfile.motor_ticks)):

        # Prediction.
        control = np.array(logfile.motor_ticks[i]) * ticks_to_mm
        x_i, x_j, Sigma_ij = ekf.predict(control)
        
        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            ekf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in range(len(observations)):
            ekf.correct(*observations[j])
        # Save Results
        states.append(ekf.state)
        covariances.append(ekf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

        # Add the current state to the graph_slam.
        add_motion_constraint(i, x_i, x_j, Sigma_ij, graph_slam)

    ######################
    #       EKF 
    ######################
    # Write all states, all state covariances, and matched cylinders to file.
    with open("Results_EKF.txt", "w") as f:
        for i in range(len(states)):
            # Output the center of the scanner, not the center of the robot.
            # print >> f, "F %f %f %f" % \
            x = tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                                   scanner_displacement * sin(states[i][2]),
                                   0.0])
            line = "F " + str(x[0]) + " " + str(x[1]) + " " + str(x[2]) + "\n" 
            f.write(line)
            e = EKF.get_error_ellipse(covariances[i])
            q = (e + (sqrt(covariances[i][2,2]),))
            line = "E " + str(q[0]) + " " + str(q[1]) + " " + str(q[2]) + " " + str(q[3]) + "\n" 
            f.write(line)
            write_cylinders(f, "W C", matched_ref_cylinders[i])

    ###########################
    #      GRAPH SLAM
    ###########################
    # Write result to file
    with open("Results_Graph_Slam.txt", "w") as f:
        for x, y, θ in graph_slam.poses:
            f.write(f"F {x} {y} {θ}\n")

    print(f"Wrote {len(graph_slam.poses)} poses and constraints to graph_slam_prediction.txt")

if __name__ == '__main__':
    test()
