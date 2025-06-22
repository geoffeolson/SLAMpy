# The full Kalman filter, consisting of prediction and correction step.
# slam_07_f_kalman_filter
# You Tube SLAM D17
# Claus Brenner, 12.12.2012
from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
import numpy as np
from numpy import *
from slam_d_library import get_observations, write_cylinders
import os
import json

class EKF:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.ticks_to_mm = 0.349
        self.cylinder_offset = 90.0
        self.depth_jump = 100.0
        self.minimum_valid_distance = 20.0
        self.max_cylinder_distance = 300.0
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    def read_json(self, json_obj):
        """Initialize EKF constants from a JSON object (dict)."""
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

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta

        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:
            # This is for the case r != l.
            alpha = (r - l) / w
            rad = l/alpha
            dg1_dtheta = (rad + w/2.)*(cos(theta+alpha) - cos(theta))
            dg2_dtheta = (rad + w/2.)*(sin(theta+alpha) - sin(theta))

        else:
            # This is for the special case r == l.
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
        x, y, theta = state
        l, r = tuple(control)

        if r != l:
            # This is for the case l != r.
            # Note g has 3 components and control has 2, so the result
            # will be a 3x2 (rows x columns) matrix.
            alpha = (r - l) / w
            theta2 = theta + alpha
            dg1_dl =  w*r/((r-l)**2) * ( sin(theta2)-sin(theta)) - (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dl =  w*r/((r-l)**2) * (-cos(theta2)+cos(theta)) - (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dl = -1/w
            dg1_dr = -w*l/((r-l)**2) * ( sin(theta2)-sin(theta)) + (r+l)/(2*(r-l)) * cos(theta2)
            dg2_dr = -w*l/((r-l)**2) * (-cos(theta2)+cos(theta)) + (r+l)/(2*(r-l)) * sin(theta2)
            dg3_dr = 1/w
            
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
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.

        left, right = control
        a1 = self.control_motion_factor
        a2 = self.control_turn_factor
        Gl2 = (a1 * left)**2 + (a2 * (left - right))**2
        Gr2 = (a1 * right)**2 + (a2 * (left - right))**2
        sigma_control = diag([Gl2, Gr2])
        Vt = self.dg_dcontrol(self.state, control, self.robot_width)
        Rt = np.dot(Vt, np.dot(sigma_control, Vt.T))
        Gt = self.dg_dstate(self.state, control, self.robot_width)
        self.covariance = np.dot(Gt, np.dot(self.covariance, Gt.T)) + Rt
        x_prev = self.state.copy()
        x_curr = self.g(self.state, control, self.robot_width)
        self.state = x_curr
        return (x_prev, x_curr, Rt)

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi

        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        # --->>> Put your method from 07_e_measurement derivative here.
        x, y, theta = state
        xm, ym = landmark
        d = scanner_displacement
        # --->>> Insert your code here.
        # Note that:
        # x y theta is state[0] state[1] state[2]
        # x_m y_m is landmark[0] landmark[1]
        # The Jacobian of h is a 2x3 matrix.
        xl = x + d * cos(theta)
        yl = y + d * sin(theta)
        dx = (xm - xl)
        dy = (ym - yl)
        q = dx**2 + dy**2
        dr_dx = - dx / sqrt(q)
        dr_dy = - dy / sqrt(q)
        dr_dtheta = d / sqrt(q) * (dx * sin(theta) - dy * cos(theta))
        da_dx = dy / q
        da_dy = - dx / q
        da_dtheta = - d / q * (dx * cos(theta) + dy * sin(theta)) - 1
        m = array([
            [dr_dx, dr_dy, dr_dtheta], 
            [da_dx, da_dy, da_dtheta]])

        return m

    # def correct(self, measurement, landmark):
    #     """The correction step of the Kalman filter."""
    #     H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
    #     sigma_r = self.measurement_distance_stddev
    #     sigma_a = self.measurement_angle_stddev
    #     Q = diag([sigma_r**2, sigma_a**2])
    #     S = self.covariance
    #     K = S @ H.T @ linalg.inv(H @ S @ H.T + Q)
    #     innovation = array(measurement) - self.h(self.state, landmark, self.scanner_displacement)
    #     innovation[1] = (innovation[1] + pi) % (2*pi) - pi
    #     self.state = self.state + K @ innovation
    #     self.covariance = (eye(3) - K @ H) @ S

    # def correct(self, measurement, landmark):
    #     """The correction step of the Kalman filter."""
    #     H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
    #     Sigma_r = self.measurement_distance_stddev
    #     Sigma_a = self.measurement_angle_stddev
    #     Q = diag([Sigma_r**2, Sigma_a**2])
    #     S = self.covariance

    #     z_pred = self.h(self.state, landmark, self.scanner_displacement)  # Predicted measurement
    #     innovation = array(measurement) - z_pred
    #     innovation[1] = (innovation[1] + pi) % (2*pi) - pi

    #     K = S @ H.T @ linalg.inv(H @ S @ H.T + Q)
    #     self.state = self.state + K @ innovation
    #     self.covariance = (eye(3) - K @ H) @ S

    #     return z_pred, Q  # Return predicted measurement and its covariance

    def correct(self, measurement, landmark, k):
        """
        Perform EKF correction and return observation residual and noise.

        Args:
            measurement: Observed range and bearing (z_ik).
            landmark: Landmark position (x_k, y_k).
            k: Landmark index.

        Returns:
            (k, z_ik, Q): A tuple with:
                k: Landmark index.
                z_ik: Innovation (residual) = z - h(x_i, k).
                Q: Measurement noise covariance matrix (2x2).
        """
        z_pred = self.h(self.state, landmark, self.scanner_displacement)  # Predicted measurement
        innovation = np.array(measurement) - z_pred
        innovation[1] = (innovation[1] + np.pi) % (2 * np.pi) - np.pi  # Normalize angle

        H = self.dh_dstate(self.state, landmark, self.scanner_displacement)
        S = self.covariance
        sigma_r = self.measurement_distance_stddev
        sigma_a = self.measurement_angle_stddev
        Q = np.diag([sigma_r**2, sigma_a**2])
        K = S @ H.T @ np.linalg.inv(H @ S @ H.T + Q)

        self.state = self.state + K @ innovation
        self.covariance = (np.eye(3) - K @ H) @ S

        return i, measurement, k, Q




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
