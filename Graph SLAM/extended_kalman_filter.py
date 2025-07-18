# The full Kalman filter, consisting of prediction and correction step.
# slam_07_f_kalman_filter
# You Tube SLAM D17
# Claus Brenner, 12.12.2012
from math import sin, cos, pi, atan2, sqrt
import numpy as np
from numpy import pi, array, diag, linalg
import os
import json
from lego_robot import *
from slam_d_library import get_observations, write_cylinders

class EKF:
    scanner_displacement = 30.0

    def __init__(self):

        # Constants.
        self.initial_state = np.zeros((3,))
        self.initial_covariance = np.diag([100**2, 100**2, (10 * pi / 180)**2])
        self.ticks_to_mm = 0.349
        self.cylinder_offset = 90.0
        self.depth_jump = 100.0
        self.minimum_valid_distance = 20.0
        self.max_cylinder_distance = 300.0
        self.robot_width = 155.0
        self.scanner_displacement = 30.0
        self.control_motion_factor = 0.35
        self.control_turn_factor = 0.6
        self.measurement_distance_stddev = 200.0
        self.measurement_angle_stddev = 0.2617993877991494
        self.observations = []

    def load_json(self, json_filename):
        with open(json_filename, 'r') as f:
            self.read_json(json.load(f))

    def read_json(self, json_obj):
        """Initialize EKF constants from a JSON object (dict)."""
        self.initial_state = np.array(json_obj["initial_state"])
        self.initial_state[2] *= pi/180
        S = np.array(json_obj["initial_covariance"])
        self.initial_covariance = np.diag([S[0]**2, S[1]**2, (S[2] * pi / 180)**2])
        self.ticks_to_mm = json_obj["ticks_to_mm"]
        self.cylinder_offset = json_obj["cylinder_offset"]
        self.depth_jump = json_obj["depth_jump"]
        self.minimum_valid_distance = json_obj["minimum_valid_distance"]
        self.max_cylinder_distance = json_obj["max_cylinder_distance"]
        self.robot_width = json_obj["robot_width"]
        self.scanner_displacement = json_obj["scanner_displacement"]
        self.control_motion_factor = json_obj["control_motion_factor"]
        self.control_turn_factor = json_obj["control_turn_factor"]
        self.measurement_distance_stddev = json_obj["measurement_distance_stddev"]
        self.measurement_angle_stddev = json_obj["measurement_angle_stddev"]

    def write_json(self):
        """Export EKF constants to a JSON-compatible dict."""
        return {
            "initial_state": [
                float(self.state[0]),
                float(self.state[1]),
                float(self.state[2] * 180 / pi)
            ],
            "covariance": [
                float(np.sqrt(self.covariance[0, 0])),
                float(np.sqrt(self.covariance[1, 1])),
                float(np.sqrt(self.covariance[2, 2]) * 180 / pi)
            ],
            "ticks_to_mm": self.ticks_to_mm,
            "cylinder_offset": self.cylinder_offset,
            "depth_jump": self.depth_jump,
            "minimum_valid_distance": self.minimum_valid_distance,
            "max_cylinder_distance": self.max_cylinder_distance,
            "robot_width": self.robot_width,
            "scanner_displacement": self.scanner_displacement,
            "control_motion_factor": self.control_motion_factor,
            "control_turn_factor": self.control_turn_factor,
            "measurement_distance_stddev": self.measurement_distance_stddev,
            "measurement_angle_stddev": self.measurement_angle_stddev
        }

    def print_motion(self, i):
        s = self.state
        a1 = self.control_motion_factor
        print(f"step:{i}, motion stdev:{a1} state: {s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f}")

    def print_observation(self, i, j):
        s = self.state
        a = self.measurement_distance_stddev
        print(f"step:{i}, obs:{j}, stdev:{a} state: {s[0]:.3f}, {s[1]:.3f}, {s[2]:.3f}")

    @staticmethod
    def g(state, control, w):
        """Takes the state of x_i(x, y, theta) and the control u_i(left, right)
            and returns the state of x_{i+1} as a numpy array.
        """
        x, y, theta = state  # state of x_i
        l, r = control  # left and right wheel velocities

        # turning case
        if r != l:
            alpha = (r - l) / w  # angle of the circular arc
            rad = l/alpha  # radius of the circular arc
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi

        # straight line case
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control

        # turning case
        if r != l:  
            alpha = (r - l) / w # angle of the circular arc
            rad = l/alpha # radius of the circular arc
            dg1_dtheta = (rad + w/2.)*(cos(theta+alpha) - cos(theta))
            dg2_dtheta = (rad + w/2.)*(sin(theta+alpha) - sin(theta))

        # straight line case
        else:
            dg1_dtheta = -l*sin(theta)
            dg2_dtheta =  l*cos(theta)

        # The derivative of g with respect to x, y and theta is a 3x3 matrix.
        m = array([
            [1, 0, dg1_dtheta], 
            [0, 1, dg2_dtheta], 
            [0, 0, 1]])

        return m

    @staticmethod
    def dg_dcontrol(state, control, w):            
        """
        Derivative of g with respect to control (left, right).
        g has 3 components and control has 2, so the result
        will be a 3x2 (rows x columns) matrix.
        """
        x, y, theta = state
        l, r = tuple(control)
      
        # turning case
        if r != l:
            alpha = (r - l) / w # angle of the circular arc
            theta2 = theta + alpha 
            dg1_dl =  w*r/((r-l)**2) * ( sin(theta2)-sin(theta)) - (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dl =  w*r/((r-l)**2) * (-cos(theta2)+cos(theta)) - (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dl = -1/w
            dg1_dr = -w*l/((r-l)**2) * ( sin(theta2)-sin(theta)) + (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dr = -w*l/((r-l)**2) * (-cos(theta2)+cos(theta)) + (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dr = 1/w
            
        # straight line case
        else:
            dg1_dl = 0.5 * (cos(theta) + (l/w) * sin(theta))
            dg2_dl = 0.5 * (sin(theta) - (l/w) * cos(theta))
            dg3_dl = -1/w
            dg1_dr = 0.5 * (cos(theta) - (l/w) * sin(theta))
            dg2_dr = 0.5 * (sin(theta) + (l/w) * cos(theta))
            dg3_dr = 1/w           

        m = array([
            [dg1_dl, dg1_dr], 
            [dg2_dl, dg2_dr], 
            [dg3_dl, dg3_dr]])
            
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
        """
        Predicts state and covariance at the next time step.
             covariance' = G * covariance * GT + R
        where R = V * sigma_control * VT. sigma_control is the
        covariance in control space, which depends on the move distance.
        """
        left, right = control

        # Compute covariance in the 2D (Left, Right) control space.
        a1 = self.control_motion_factor
        a2 = self.control_turn_factor
        Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
        Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
        sigma_control = diag([Gl2, Gr2])

        # Compute the state and covariance
        Vt = self.dg_dcontrol(self.state, control, self.robot_width)
        Rt = Vt @ sigma_control @ Vt.T
        Gt = self.dg_dstate(self.state, control, self.robot_width)
        self.covariance = Gt @ self.covariance @ Gt.T + Rt
        prev_state = self.state
        self.state = self.g(self.state, control, self.robot_width)   
        return prev_state, self.state, Rt

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """
        Takes state(x, y, theta) and landmark(x, y) and estimates the
        measurement(range, bearing) a 2D vector.
        """
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        """
        Computes the derivative of h, the estimated measurement(range, bearing) 
        with respect to the state(x, y, theta) a 2x3 matrix.
        """
        x, y, theta = state
        xm, ym = landmark

        # Compenstaed for scanner positon on robot
        d = scanner_displacement
        xl = x + d * cos(theta)
        yl = y + d * sin(theta)

        # position of the landmark in the robot's coordinate system
        dx = (xm - xl)
        dy = (ym - yl)
        q = dx**2 + dy**2  # squared distance to landmark

        # derivatives of the range
        dr_dx = - dx / sqrt(q)
        dr_dy = - dy / sqrt(q)
        dr_dtheta = d / sqrt(q) * (dx * sin(theta) - dy * cos(theta))

        # derivatives of the bearing angle
        da_dx = dy / q
        da_dy = - dx / q
        da_dtheta = - d / q * (dx * cos(theta) + dy * sin(theta)) - 1

        J = array([
            [dr_dx, dr_dy, dr_dtheta], 
            [da_dx, da_dy, da_dtheta]])

        return J

    def correct(self, measurement, landmark):
        """
        Perform EKF correction and returns observation noise.
        Takes measurement z_ik (range and bearing) and landmark position x_k(x, y)
        and computes inovation, the measurement noise, Kalman gain, and updates the state and covariance.
        """
        # Compute innovation (the observation error residual [range, bearing])
        z_pred = self.h(self.state, landmark, self.scanner_displacement)  # Predicted measurement
        innovation = np.array(measurement) - z_pred
        innovation[1] = (innovation[1] + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

        # Compute Q (the 2x2 noise covariance matrix for measurement [range, bearing angle]).
        sigma_r = self.measurement_distance_stddev
        sigma_a = self.measurement_angle_stddev
        Q = np.diag([sigma_r**2, sigma_a**2])
        
        # Compute K (the Kalman gain).
        H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        S = self.covariance
        K = S @ H.T @ np.linalg.inv(H @ S @ H.T + Q)

        # Update 3D state vector and  3x3 covariance matrix
        self.state = self.state + K @ innovation
        self.covariance = (np.eye(3) - K @ H) @ S

        return Q

if __name__ == '__main__':
    os.chdir("Unit_I")
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
    # Setup filter.
    kf = EKF(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in range(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in range(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    with open("kalman_prediction_and_correction.txt", "w") as f:
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
