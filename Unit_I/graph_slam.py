"""
GraphSLAM class for simple 2D graph-based SLAM testing.

This version supports adding fixed 2D poses and relative pose constraints,
computing residuals, and preparing for Gauss-Newton optimization.
"""
from scipy.stats import sigmaclip
from extended_kalman_filter import EKF
from lego_robot import *
from numpy import pi, sin, cos, pi, atan2
from numpy.linalg import norm, inv, solve
import numpy as np        
import matplotlib.pyplot as plt
import numpy as np
import os
from itertools import combinations
from levenberg_marquardt import LevenbergMarquardt

class Graph:
    def __init__(self):
        self.ekf_states = None
        self.landmarks = []                  # List of 2D landmark positions [x, y]
        self.poses = []                     # List of 2D poses: [x, y, theta]
        self.constraints = []                # List of tuples: (i, j, z_ij, Omega_ij)
        self.observation_constraints = []    # List of tuples: (pose_index, landmark_index, z_ik, Omega_ik) where z_ik is the relative observation of landmark k from pose i
        self.scanner_displacement = 30.0     # Offset of observation scanner from robot center
        self.H = None                        # Matrix for system of equations used for solver
        self.b = None                        # Matrix for system of equations used for solver
        self.angle_scale = 1000
        ############################################################################
        # REMOVE TESTING CODE
        #self.motion_noise = np.diag([5**2, 5**2, (1.0*pi/180)**2])
        self.motion_noise = np.diag([5**2, 5**2, (1.0*pi/180)**2])
        ############################################################################


   
    def compute_relative_pose(self, x_i, x_j):
        """
        Compute x_ij the pose of x_j relative to x_i. This is the 2D pose [x, y, theta]
        """
        # Deltas x_j - x_i and angle theta_i
        dx = x_j[0] - x_i[0]
        dy = x_j[1] - x_i[1]
        dtheta = x_j[2] - x_i[2]
        θ = x_i[2]
        
        # Rotate the deltas by -θ to get relative pose in x_i's frame
        rot_dx = cos(-θ) * dx - sin(-θ) * dy
        rot_dy = sin(-θ) * dx + cos(-θ) * dy
        rot_dtheta = (dtheta + pi) % (2 * pi) - pi
        
        return np.array([rot_dx, rot_dy, rot_dtheta])

    def add_motion_constraint(self, i, j, x_i, x_j, Sigma_ij):
        """
        Add a motion constraint between the last pose and the new pose x_j.
        
        The robot motion model results in a covariance matrix with perfect correlation, 
        because robot rotation only results from unequal rotation between right and left tracks. 
        This correlation in the covariance matrix results in a singular matrix that cannot be inverted. 
        The following code adds a small motion_noise to the covariance matrix, providing 
        imperfect correlation resulting in a more realistic robot motion model and an invertable matrix.
        """
        x_ij = self.compute_relative_pose( x_i, x_j)
        
        #Compute the information matrix from the covariance matrix
        Omega_ij = np.linalg.inv(Sigma_ij + self.motion_noise)

        self.constraints.append((i, j, x_ij, Omega_ij))

    @staticmethod
    def v2t(pose_vec):
        """Convert [x, y, theta] → 3×3 homogeneous transform. """
        x, y, theta = pose_vec
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        T = np.array([
            [cos_theta, -sin_theta, x],
            [sin_theta,  cos_theta, y],
            [0,          0,         1]
        ])
        return T

    @staticmethod
    def t2v(T):
        """Convert 3×3 homogeneous transform → [x, y, theta]."""
        x = T[0, 2]
        y = T[1, 2]
        theta = np.arctan2(T[1, 0], T[0, 0])
        return np.array([x, y, theta])

    def add_pose(self, pose):
        """Add a new pose to the graph."""
        self.poses.append(np.array(pose))

    def add_observation_constraint(self, pose_index, measurment, landmark, Sigma_ij):
        """
        Add an observation constraint between a pose and a landmark.
        pose_index: Index of the robot pose at the time of observation.
        z_ij: measured observation from the pose to the landmark [range, bearing]
        x_k: landmark in the global frame [x, y].
        Sigma_ij: 2x2 covariance matrix of the observation [range, bearing].
        """
        ##################################################################
        #  REMOVE TESTING CODE 
        #Sigma_ij *= 0.1
        ####################################################################
        #Compute the information matrix from the covariance matrix
        Omega_ij = np.linalg.inv(Sigma_ij)

        self.observation_constraints.append((pose_index, np.array(measurment), np.array(landmark), np.array(Omega_ij)))

    def compute_error(self, constraint):
        """
        Compute the residual e_ij = z_ij - z_hat_ij
        where z_hat_ij is the predicted relative pose from i to j
        """
        i, j, Zij, _ = constraint
        xi = self.poses[i]
        xj = self.poses[j]

        # Convert to transforms
        Ti = Graph.v2t(xi)
        Tj = Graph.v2t(xj)

        Tij_pred = np.linalg.inv(Ti) @ Tj
        Zij_hat = Graph.t2v(Tij_pred)

        error = Zij - Zij_hat
        error[2] = (error[2] + pi) % (2 * pi) - pi
        return error

    @staticmethod
    def compute_jacobians(xi, xj):
        """
        Compute 3x3 Jacobians matricies A_ij and B_ij. 
        Thess are the derivatives of the pose error vector e_ij with respect to the corresponding poses xi and xj.
        Given: 
            xi: the pose i [x_i, y_i, theta_i] 
            xj: the pose j [x_j, y_j, theta_j]
        Returns:
            A_ij: 3x3 matrix derivative of the error e_ij with respect to xi
            B_ij: 3x3 matrix derivative of the error e_ij with respect to xj
        """
        # Extract values
        xi_x, xi_y, xi_theta = xi
        xj_x, xj_y, xj_theta = xj

        # 2D translation delta t
        delta_t = np.array([xj_x - xi_x, xj_y - xi_y])

        # Rotation matrix Ri and its transpose
        cos_theta = np.cos(xi_theta)
        sin_theta = np.sin(xi_theta)
        Ri = np.array([[cos_theta, -sin_theta],
                        [sin_theta,  cos_theta]])
        Ri_T = Ri.T

        # Skew-symmetric matrix S
        S = np.array([[0, -1],
                        [1,  0]])

        # Compute blocks for A_ij
        A_pos = -Ri_T
        A_theta = Ri_T @ S @ delta_t
        A_ij = np.zeros((3, 3))
        A_ij[0:2, 0:2] = A_pos
        A_ij[0:2, 2] = A_theta
        A_ij[2, 2] = -1

        # Compute blocks for B_ij
        B_ij = np.zeros((3, 3))
        B_ij[0:2, 0:2] = Ri_T
        B_ij[2, 2] = 1

        return A_ij, B_ij

    def controls_angle_scaling(self, e_ij, A_ij, B_ij, Omega_ij):
        """Scale angle part of e_ij and Jacobians
        converts the angles from radians to mili-radians
        This scaling improves the condition of the H matrix
        """
        scale = self.angle_scale
        e_ij[2] *= scale # Scale the angle residual (mrad)
        A_ij[2, :] *= scale  # Scale the angle Jacobian row
        B_ij[2, :] *= scale # Scale the angle Jacobian row
        S_inv = np.diag([1.0, 1.0, 1.0 / scale])
        Omega_ij = S_inv.T @ Omega_ij @ S_inv # Scale the information matrix
        return e_ij, A_ij, B_ij, Omega_ij

    def observation_angle_scaling(self, e_ik, J_i, Omega_ik):
        """ANGLE SCALING
        converts the angles from radians to mili-radians
        This scaling improves the condition of the H matrix
        """
        scale = self.angle_scale
        e_ik[1] *= scale # Scale the bearing residual (mrad)
        J_i[1, :] *= scale  # Scale the bearing Jacobian row
        S_inv = np.diag([1.0, 1.0 / scale])# Scale the information matrix
        Omega_ik = S_inv.T @ Omega_ik @ S_inv
        return e_ik, J_i, Omega_ik

    def delta_x_angle_unscaling(self, delta_x):
        """ Unscale the angle part of delta_x """
        scale = self.angle_scale
        size = int(len(delta_x) / 3) # size is the number of poses
        for i in range(size):
            delta_x[3 * i + 2] *= 1/scale  # Convert back from mrad to rad
        return delta_x

    def compute_H_b_of_controls(self):
        """
        Build the linear system H and b for Graph-Based SLAM.
        Follows directly from the derivation in the README.md
        """
        # N = 3 * len(self.poses)
        # self.H = np.zeros((N, N))
        # self.b = np.zeros((N, ))

        for constraint in self.constraints:
            i, j, z_ij, Omega_ij = constraint

            # Get current poses
            xi = self.poses[i]
            xj = self.poses[j]

            # Compute error and Jacobians
            e_ij = self.compute_error(constraint)
            A_ij, B_ij = Graph.compute_jacobians(xi, xj)

            e_ij, A_ij, B_ij, Omega_ij = self.controls_angle_scaling(e_ij, A_ij, B_ij, Omega_ij)

            # Compute blocks for H
            H_ii = A_ij.T @ Omega_ij @ A_ij
            H_ij = A_ij.T @ Omega_ij @ B_ij
            H_ji = B_ij.T @ Omega_ij @ A_ij
            H_jj = B_ij.T @ Omega_ij @ B_ij

            # Compute blocks for b
            b_i = - A_ij.T @ Omega_ij @ e_ij
            b_j = - B_ij.T @ Omega_ij @ e_ij

            # Assemble into global H and b
            i_idx = slice(3 * i, 3 * i + 3)
            j_idx = slice(3 * j, 3 * j + 3)

            self.H[i_idx, i_idx] += H_ii
            self.H[i_idx, j_idx] += H_ij
            self.H[j_idx, i_idx] += H_ji
            self.H[j_idx, j_idx] += H_jj

            self.b[i_idx] += b_i
            self.b[j_idx] += b_j

    def compute_H_b_of_observations(self):
        """
        Add contributions from observation constraints to the linear system H and b.
        """
        for i, z_ik, x_k, Omega_ik in self.observation_constraints:
            x_i = self.poses[i]

            # observation error
            z_pred = EKF.h(x_i, x_k, self.scanner_displacement)
            e_ik = z_ik - z_pred
            e_ik[1] = (e_ik[1] + pi) % (2 * pi) - pi

            # 2x3 Jacobian: the derivative of observation error with respect to pose x_i
            J_i = -EKF.dh_dstate(x_i, x_k, self.scanner_displacement )

            e_ik, J_i, Omega_ik = self.observation_angle_scaling(e_ik, J_i, Omega_ik)

            # Compute contribution to H and b
            H_ii = J_i.T @ Omega_ik @ J_i
            b_i  = J_i.T @ Omega_ik @ e_ik

            i_idx = slice(3 * i, 3 * i + 3)
            self.H[i_idx, i_idx] += H_ii
            self.b[i_idx] += b_i

    def anchor_pose(self, time_step):
        # Anchor pose at time_step
        s = slice(time_step, time_step + 3)
        self.H[s,:] = 0
        self.H[:,s] = 0
        self.H[s,s] = np.eye(3)
        self.b[s]   = 0

    def solve(self, debug=False):
        """
        Levenberg-Marquardt solver for Graph-Based SLAM.
        Efficient implementation: modifies only diagonal of H and restores it properly.
        """
        N = len(self.poses)
        self.H = np.zeros((3 * N, 3 * N))
        self.b = np.zeros((3 * N,))
        lm = LevenbergMarquardt()

        if debug:
            self.print_iteration(-1, 0, 0)
            self.plot_comparison()

        for iteration in range(lm.get_max_iterations()):

            # Recompute Jacobian and residuals if last update was accepted
            if lm.cost_is_decreasing():
                self.compute_H_b_of_controls()
                self.compute_H_b_of_observations()
                self.anchor_pose(0)
                lm.set_matrix(self.H)

            # Solve linear system with damped matrix
            delta_x = lm.solve_damped(-self.b)
            delta_x = self.delta_x_angle_unscaling(delta_x)

            # compute cost and evaluate the current update
            cost = np.linalg.norm(delta_x)
            lm.set_cost(cost)

            # Debug output
            if debug:
                self.print_iteration(iteration, cost, lm.get_lambda())
                self.plot_comparison()

            # Stop if converged
            if lm.cost_met_stop_criteria():
                print(f"Converged in {iteration + 1} iterations.")
                break

            # Apply Δx to poses if accepted
            if lm.cost_is_decreasing():
                self.apply_delta_to_poses(delta_x)

    def apply_delta_to_poses(self, delta):
        N = len(self.poses)
        for i in range(N):
            idx = slice(3 * i, 3 * i + 3)
            self.poses[i] += delta[idx]

    def print_iteration(self, i, norm_dx, lamb):
        x = self.poses
        if self.H is not None: condition = np.linalg.cond(self.H)
        else: condition = 0
        print(f"{i+1}. cost:{norm_dx:.6f}, condition:{condition:.1e}, lambda:{lamb:.1e}")

    def plot_comparison(self):
        fig, ax = plt.subplots() 

        # plot the poses from Graph SLAM
        p = np.array(self.poses.copy())
        ax.plot(p[:,0], p[:,1], 'y-', label='GraphSLAM', linewidth=4)

        # plot the EKF states
        p = np.array(self.ekf_states.copy())
        ax.plot(p[:,0], p[:,1], 'g--', label='EKF', linewidth=2)

        # Set properties of the Axes
        ax.set_title('EKF vs Graph SLAM')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.legend() # Display the legend based on the 'label' in ax.plot()
        ax.grid(True) # Add a grid to the plot

        # Display the plot
        plt.show(block=True)

    @staticmethod
    def plot_x_y_theta( data, title="Title"):
        import matplotlib.pyplot as plt
        import numpy as np

        # Undo interleaving of x, y, theta
        data2 = np.array(data.copy())
        x = data2[0::3]
        y = data2[1::3]
        theta = data2[2::3]
        time_step = np.arange(x.shape[0])

        # Plot on the left y axis
        fig, ax1 = plt.subplots()
        ax1.plot(time_step, x, 'g-', label='X')
        ax1.plot(time_step, y, 'b-', label='Y')
        ax1.set_xlabel('Time Step')
        ax1.set_ylabel('XY (mm)', color='g')
        ax1.tick_params(axis='y', labelcolor='g')
        ax1.legend(loc='upper left')

        #  Plot on the right y axis
        ax2 = ax1.twinx() 
        ax2.plot(time_step, theta*180/pi, 'r--', label='Theta')
        ax2.set_ylabel('Theta (deg)', color='b')
        ax2.tick_params(axis='y', labelcolor='b')
        ax2.legend(loc='upper right')

        plt.title(title)
        fig.tight_layout() # Adjust layout to prevent labels from overlapping

    def print_summary(self):
        print("\n***** FINAL OPTIMIZED STATE *******")
        print("Poses:")
        for idx, p in enumerate(self.poses):
            print(f"  x{idx} = {p[0]:.6f} {p[1]:.6f} {p[2]:.6f}")
        print("Constraints:")
        for i, j, z, _ in self.constraints:
            Zij = np.array([z[0],z[1],z[2] * 180 / pi])
            print(f"  x{i} → x{j} : {Zij}")
        print("observation Constraints:")
        for observation in self.observation_constraints:
            i,m,l,x = observation
            print(f" x{i} → meas:[{m[0]}, {m[1]*180/pi}]  landmark:[{l[0]}, {l[1]}]")
        print("Error:")
        for constraint in self.constraints:
            i, j, _, _ = constraint
            e_ij = self.compute_error(constraint)
            e_ij[2] = e_ij[2] * 180 / pi
            print(f"  x{i} → x{j} : {e_ij}")

def test_object_creation():
    """
    Unit test: Create Object and Test.
     Expected Output:
         Poses:
       x0 = [0.0 0.0 0.0]
       x1 = [1.0 0.0 0.0]
       x2 = [2.0 0.0 0.0]
     Constraints:
       x0 → x1 : [ 0.9  0.1  5.0 ]
       x1 → x2 : [ 1.1 -0.1  0.0 ]
     Error:
       x1 → x2 : [-0.1  0.1  5.0 ]
       x1 → x2 : [ 0.1 -0.1  0.0 ]
    """
    # create Graph Slam object
    # gs = Graph()

    # #Add Poses
    # x0 = [0.0, 0.0, 0.0]   # origin
    # x1 = [1.0, 0.0, 0.0]   # 1 meter forward
    # x2 = [2.0, 0.0, 0.0]   # 2 meter forward

    # gs.add_pose(x0)
    # gs.add_pose(x1)
    # gs.add_pose(x2)

    # #Add constraint
    # z_ij = [0.9, 0.1, 5 * pi / 180]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(0, 1, z_ij, Omega_ij)

    # z_ij = [1.1, -0.1, 0.0]
    # Omega_ij = np.diag([1.0, 1.0, 1.0])
    # gs.add_constraint(1, 2, z_ij, Omega_ij)

    # #print Results
    # gs.print_summary()

    # return gs

def test_jacobians():
    """
    Unit test: Compare analytical and numerical Jacobians.
    """

    eps = 1e-6  # small perturbation for finite difference

    # Example poses (arbitrary but reasonable values)
    xi = np.array([1.0, 2.0, 30 * np.pi / 180])  # Pose i: x, y, theta
    xj = np.array([2.5, 3.0, 40 * np.pi / 180])  # Pose j: x, y, theta

    # Dummy observation and information matrix (not used for Jacobian test)
    z_ij = np.array([0.0, 0.0, 0.0])
    Omega_ij = np.eye(3)

    # Build artificial constraint to reuse existing error function
    gs = Graph()
    gs.add_pose(xi)
    gs.add_pose(xj)
    constraint = (0, 1, z_ij, Omega_ij)

    # Compute baseline error
    e0 = gs.compute_error(constraint)

    # Compute analytical Jacobians
    A_analytic, B_analytic = Graph.compute_jacobians(xi, xj)

    # Numerical Jacobian for xi (A_ij)
    A_numeric = np.zeros((3, 3))
    for k in range(3):
        xi_perturbed = xi.copy()
        xi_perturbed[k] += eps
        gs.poses[0] = xi_perturbed  # update pose i
        e_perturbed = gs.compute_error(constraint)

        # IMPORTANT NOTE ON THE SIGN:
        # The error function is defined as: e_ij = z_ij - hat_z_ij
        # Perturbing xi affects hat_z_ij, and therefore affects e_ij negatively.
        # To correctly approximate the Jacobian, we must negate the finite difference:
        A_numeric[:, k] = -(e_perturbed - e0) / eps

        gs.poses[0] = xi  # restore

    # Numerical Jacobian for xj (B_ij)
    B_numeric = np.zeros((3, 3))
    for k in range(3):
        xj_perturbed = xj.copy()
        xj_perturbed[k] += eps
        gs.poses[1] = xj_perturbed  # update pose j
        e_perturbed = gs.compute_error(constraint)

        # Same sign logic applies to xj perturbation:
        B_numeric[:, k] = -(e_perturbed - e0) / eps

        gs.poses[1] = xj  # restore

    # Compare results
    print("\n\n***********JACOBIAN TEST****************")
    print("\n\n\n******A*********")
    print("Analytical A_ij:")
    print(A_analytic)
    print("Numerical A_ij:")
    print(A_numeric)
    print("Difference A:")
    print(A_analytic - A_numeric)

    print("\n\n\n******B*********")
    print("Analytical B_ij:")
    print(B_analytic)
    print("Numerical B_ij:")
    print(B_numeric)
    print("Difference B:")
    print(B_analytic - B_numeric)



def test_solver():
    """
    Unit test: Verify full Gauss-Newton optimization converges correctly.
    """

    print("\n\n*********** SOLVER TEST ***********")

    # Build Graph SLAM object using existing helper
    
    print("\nInitial State:")
    gs = test_object_creation()

    # Print initial error
    print("\nInitial state:")
    #gs.print_summary()

    # Run optimization
    gs.solve(max_iterations=10, tol=1e-6)

    # Print final optimized state
    print("\nFINAL OPTIMIZED STATE:")
    gs.print_summary()

def test_prediction():
    print("\n*****GRAPH SLAM PREDICTION TEST******")
    
    # EKF setup (front-end)
    initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    ticks_to_mm = 0.349
    robot_width = 155.0
    motion_factor = 0.35
    turn_factor = 0.6

    ekf = ExtendedKalmanFilter(
        initial_state,
        initial_covariance,
        robot_width,
        motion_factor,
        turn_factor
    )

    # Graph SLAM setup (back-end)
    graph_slam = Graph()
    graph_slam.add_pose(initial_state.copy())  # Pose x0

    # Load odometry
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Main loop
    for i, ticks in enumerate(logfile.motor_ticks):
        control = np.array(ticks) * ticks_to_mm
        ekf.predict(control)

        # Add new pose
        graph_slam.add_pose(ekf.state.copy())

        # Add constraint from previous pose to current
        z_ij = ekf.get_last_relative_motion()
        Sigma_ij = ekf.get_last_motion_covariance()
        Omega_ij = inv(Sigma_ij)  # Information matrix
        graph_slam.add_constraint(i, i + 1, z_ij, Omega_ij)

    # Write result to file
    with open("graph_slam_prediction.txt", "w") as f:
        for x, y, θ in graph_slam.poses:
            f.write(f"F {x} {y} {θ}\n")

    print(f"Wrote {len(graph_slam.poses)} poses and constraints to graph_slam_prediction.txt")

if __name__ == '__main__':
    os.chdir("Unit_H")
    print("\n*****GRAPH SLAM OBJECT TEST******")
    test_object_creation()
    test_jacobians()
    test_linear_system()
    test_solver()

##################################################################
#     DEPRICATED CODE
##################################################################

    # def add_constraint(self, i, j, z_ij, Omega_ij):
    #     """
    #     Add a relative pose constraint between pose i and pose j.
    #     z_ij: expected relative motion from i to j
    #     Omega_ij: 3x3 information matrix
    #     """
    #     self.constraints.append((i, j, np.array(z_ij), np.array(Omega_ij)))


    # def compute_information_matrix(self, Sigma):
    #     """
    #     The robot motion model results in a covariance matrix with perfect correlation, 
    #     because robot rotation only results from unequal rotation between right and left tracks. 
    #     This correlation in the covariance matrix results in a singular matrix that cannot be inverted. 
    #     The following code increases all the variances of the covariance matrix by 20% providing 
    #     imperfect correlation resulting in a more realistic robot motion model and an invertable matrix.
    #     """
    #     # rng = np.arange(Sigma.shape[0])
    #     # Sigma[rng,rng] *= 1.2
    #     #Lambda =  1.0e-3
    #     #Sigma = (Sigma + Lambda * np.eye(Sigma.shape[0]))/(1 + Lambda)
    #     #Sigma = Sigma + Lambda * np.eye(Sigma.shape[0])
    #     #Sigma += self.motion_noise

    #     Omega = np.linalg.inv(Sigma + self.motion_noise)
    #     return Omega


# def test_prediction_old():
#     print("\n*****GRAPH SLAM PREDICTION TEST******")
#     initial_state = np.array([1850.0, 1897.0, 213.0 / 180.0 * pi])
#     initial_covariance = np.diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
#     ticks_to_mm = 0.349
#     robot_width = 155.0
#     motion_factor = 0.35
#     turn_factor = 0.6

#     ekf = ExtendedKalmanFilter(
#         initial_state,
#         initial_covariance,
#         robot_width,
#         motion_factor,
#         turn_factor
#     )

#     # Load odometry
#     logfile = LegoLogfile()
#     logfile.read("robot4_motors.txt")

#     # Output file
#     with open("graph_slam_prediction.txt", "w") as f:
#         for ticks in logfile.motor_ticks:
#             control = np.array(ticks) * ticks_to_mm
#             ekf.predict(control)
#             x, y, θ = ekf.state
#             line = f"F {x} {y} {θ}\n"
#             f.write(line)


    # def solve3(self, debug=False):
    #     """
    #     Levenberg-Marquardt solver for Graph-Based SLAM.
    #     Efficient implementation: only modifies diagonal of H and restores it if update is rejected.
    #     """
    #     N = len(self.poses)
    #     self.H = np.zeros((3 * N, 3 * N))
    #     self.b = np.zeros((3 * N,))
    #     lm = LevenbergMarquardt()

    #     if debug:
    #         self.print_iteration(-1, 0, 0)
    #         #self.plot_comparison()

    #     for iteration in range(lm.get_max_iterations()):

    #         # Recompute Jacobian and residuals if last update was accepted
    #         if lm.last_update_was_accepted():
    #             self.compute_H_b_of_controls()
    #             self.compute_H_b_of_observations()
    #             self.anchor_pose(0)

    #         # Apply Marquardt damping to H by modifying only the diagonal
    #         H_damped = self.H.copy()
    #         diag_indices = np.diag_indices_from(self.H)
    #         #original_diag = self.H[diag_indices].copy()
    #         #self.H[diag_indices] *= (1.0 + lm.get_lambda())
    #         H_damped[diag_indices] += lm.get_lambda()

    #         # Solve the damped linear system
    #         delta_x = np.linalg.solve(self.H, -self.b)
    #         delta_x = self.delta_x_angle_unscaling(delta_x)
    #         cost = np.linalg.norm(delta_x)

    #         # Evaluate new delta_x and update LM logic
    #         lm.set_cost(cost)

    #         # # Restore diagonal if update was not accepted
    #         # if not lm.last_update_was_accepted():
    #         #     self.H[diag_indices] = original_diag

    #         # Debug output
    #         if debug:
    #             self.print_iteration(iteration, cost, lm.get_lambda())
    #             #self.plot_comparison()

    #         # Stop if converged
    #         if lm.solver_has_converged():
    #             print(f"Converged in {iteration + 1} iterations.")
    #             break

    #         # Apply Δx to poses if accepted
    #         if lm.last_update_was_accepted():
    #             for i in range(N):
    #                 idx = slice(3 * i, 3 * i + 3)
    #                 self.poses[i] += delta_x[idx]


    # def solve2(self, debug = False):
    #     """
    #     Runs the Levenberg Marquardt Solver algorithm to optimize the poses in the graph.
    #     """
    #     # INITIALIZE
    #     N = len(self.poses)
    #     self.H = np.zeros((3*N, 3*N))
    #     self.b = np.zeros((3*N, )) 
    #     lm = LevenbergMarquardt() # LM object handles logic for Levenberg Marquardt algorithm

    #     if debug:
    #         self.print_iteration(-1, 0,0)
    #         self.plot_comparison() # Show plot of initial data before optimization

    #     #MAIN SOLVER LOOP
    #     for iteration in range(lm.max_iterations):

    #         # Recompute Jacobian and residual
            
    #         if lm.last_update_was_accepted():
    #             self.compute_H_b_of_controls()
    #             self.compute_H_b_of_observations()
    #             self.anchor_pose(0) # anchor the first pose

    #         # Solve Linear System of Levenberg Marquardt: (H + λI) * Δx = -b
    #         H, L, I, b = self.H, lm.get_lambda(), np.eye(3 * N), self.b
    #         delta_x = np.linalg.solve(H + L * I, -b)  
    #         delta_x = self.delta_x_angle_unscaling(delta_x) # Unscale the angles in delta x
    #         cost = np.linalg.norm(delta_x)  # Compute the optimization cost
    #         lm.set_cost(cost)  # Recompute LM logic based on the new cost

    #         # Output debug information
    #         if debug:
    #             self.print_iteration(iteration, cost,lm.get_lambda())
    #             if iteration >= 0:
    #                 #self.plot_x_y_theta(delta_x,"Delta X")
    #                 #self.plot_x_y_theta(-self.b," -b ")
    #                 self.plot_comparison()

    #         # Stop the solver if LM has converged
    #         if lm.solver_has_converged():
    #             print(f"Converged in {iteration+1} iterations.")
    #             break

    #         # Apply delta_x updates to poses
    #         if lm.last_update_was_accepted():
    #             for i in range(N):
    #                 idx = slice(3 * i, 3 * i + 3)
    #                 dx_i = delta_x[idx]
    #                 self.poses[i] += dx_i



    # def solve1(self, debug = False, max_iterations=10):
    #     """
    #     Runs the Levenberg Marquardt Solver algorithm to optimize the poses in the graph.
    #     """
    #     # INITIALIZE
    #     N = len(self.poses)
    #     self.H = np.zeros((3*N, 3*N))
    #     self.b = np.zeros((3*N, )) 
    #     lm = LevenbergMarquardt() # LM object handles logic for Levenberg Marquardt algorithm

    #     if debug:
    #         self.print_iteration(-1, 0,0)
    #         self.plot_comparison() # Show plot of initial data before optimization

    #     #MAIN SOLVER LOOP
    #     for iteration in range(lm.max_iterations):

    #         # Recompute Jacobian and residual
    #         if lm.jacobian_and_residual_is_recomputed():
    #             self.compute_H_b_of_controls()
    #             self.compute_H_b_of_observations()
    #             self.anchor_pose(0) # anchor the first pose

    #         # Solve Linear System of Levenberg Marquardt: (H + λI) * Δx = -b
    #         H, L, I, b = self.H, lm.get_lambda(), np.eye(3 * N), self.b
    #         delta_x = np.linalg.solve(H + L * I, -b) 
    #         delta_x = self.delta_x_angle_unscaling(delta_x) # Unscale the angles in delta x
    #         cost = np.linalg.norm(delta_x)  # Compute the optimization cost
    #         lm.set_cost(cost)  # Recompute LM logic based on the new cost

    #         # Output debug information
    #         if debug:
    #             self.print_iteration(iteration, cost,lm.get_lambda())
    #             if iteration >= 0:
    #                 #self.plot_x_y_theta(delta_x,"Delta X")
    #                 #self.plot_x_y_theta(-self.b," -b ")
    #                 self.plot_comparison()

    #         # Stop the solver if LM has converged
    #         if lm.solver_has_converged():
    #             print(f"Converged in {iteration+1} iterations.")
    #             break

    #         # Apply delta_x updates to poses
    #         if lm.delta_x_is_applied():
    #             for i in range(N):
    #                 idx = slice(3 * i, 3 * i + 3)
    #                 dx_i = delta_x[idx]
    #                 self.poses[i] += dx_i

    # def test_linear_system():
    # """
    # Unit test: Verify correct construction of H and b.
    # This test simply runs the function and prints the outputs for inspection.
    # """

    # # Build Graph SLAM object
    # gs = test_object_creation()

    # # Build system
    # H, b = gs.controls_linear_system()

    # # Print system matrices for inspection
    # print("\n\n*********** LINEAR SYSTEM TEST ***********")
    # print("Information matrix H:")
    # print(H)
    # print("\nRight-hand side vector b:")
    # print(b)

    # class LevenbergMarquardt_Old:
    # def __init__(self):
    #     """
    #     Class to encapsulate all the generic logic for a Levenberg Marquardt solver.
    #     It does not contain any information for a specific solver application.
    #     """
    #     self.lambda_max = 1e+6 
    #     self.lambda_min = 1e-7
    #     self.lambda_init = 1e-4  # Initial lambda value
    #     self.cost_tol = 1e-3  # Cost tolerance criteria for convergence
    #     self.delta_cost_tol = 1e-4  # Change in cost tolerance criteria for convergence
    #     self.prev_cost = 1e+10  # Previous cost value to compare against for computing delta_cost
    #     self.lamb = self.lambda_init # Current lambda value
    #     self.max_iterations = 100  # Maximum number of solver iterations
    #     self.update = True  # Use the computed delta x, so need to also recompute jacobians
    #     self.converged = False  # Solver has converged to a solution so stop solver
    #     self.lambda_incr_factor = 10  # Factor to increase lambda when cost increases
    #     self.lambda_decr_factor = 0.5  # Factor to decrease lambda when cost decreases

    #     def set_cost(self, cost):
    #     delta_cost = cost - self.prev_cost

    #     # Cost is decreasing
    #     if delta_cost < 0:
    #         # Use the computed delta x value and decrease lambda to converge faster
    #         self.lamb = max(self.lambda_min, self.lamb * self.lambda_decr_factor) 
    #         self.update = True

    #         # Check stopping criteria
    #         if ((cost < self.cost_tol) or (abs(delta_cost) < self.delta_cost_tol)):
    #             # Solver has converged to a solution, so stop solver
    #             self.converged = True
    #             self.update = False

    #     # Cost is inreasing
    #     else:
    #         # Don't accept current update,
    #         # and increase lambda to prevent the overshoot that is increaing cost
    #         self.lamb = min(self.lambda_max, self.lamb * self.lambda_incr_factor)
    #         self.update = False

    #     # Only if the new update is accepted, save the current cost as the previous cost for next iteration.
    #     # We don't want to store the previous cost for bad updates.
    #     if self.update:
    #         self.prev_cost = cost

    # def get_lambda(self):
    #     return self.lamb

    # def get_max_iterations(self):
    #     return self.max_iterations

    # def last_update_was_accepted(self):
    #     return self.update

    # def cost_met_stop_criteria(self):
    #     return self.converged
