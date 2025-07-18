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
from angle_scaler import AngleScaler

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
        self.angle_scaler = AngleScaler(1000)
        self.motion_noise = np.diag([5**2, 5**2, (1.0*pi/180)**2])


   
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

        The robot’s kinematic model produces a deterministic relationship between wheel motions
        and pose updates. As a result, the computed motion covariance matrix is rank-deficient—
        it encodes perfect correlation between translation and rotation, particularly in cases
        where rotation arises solely from differential wheel movement.

        This perfect correlation leads to a singular (non-invertible) matrix, which cannot be used
        in downstream calculations such as information matrix construction.

        To resolve this, we add a small isotropic noise term (motion_noise) to the diagonal of
        the covariance matrix. This ensures numerical invertibility and more realistically reflects
        uncertainty in real-world robot motion, such as wheel slippage or encoder quantization.
        """
        x_ij = self.compute_relative_pose( x_i, x_j)
        
        #Compute the information matrix by inverting the covariance matrix
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
        Add an observation constraint between a robot pose and a landmark.

        This function encodes a relative observation z_ij = [range, bearing]
        made from a given robot pose (indexed by pose_index) to a known landmark
        in the global frame (x_k = [x, y]).

        Parameters:
            pose_index : int
                Index of the robot pose at the time the observation was made.
            measurement : array-like of shape (2,)
                The observed range and bearing to the landmark, relative to the robot pose.
            landmark : array-like of shape (2,)
                The [x, y] position of the landmark in the global coordinate frame.
            Sigma_ij : array-like of shape (2, 2)
                Covariance matrix of the measurement [range, bearing].

        Notes:
            The measurement covariance Σ_ij is converted into an information matrix Ω_ij
            (the inverse of Σ_ij), which is stored along with the observation data.
        """
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
        These are the derivatives of the pose error vector e_ij with respect to the corresponding poses xi and xj.
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

    def compute_H_b_of_controls(self):
        """
        Build the linear system H and b for Graph-Based SLAM.
        Follows directly from the derivation in the README.md
        """

        for constraint in self.constraints:
            i, j, z_ij, Omega_ij = constraint

            # Get current poses
            xi = self.poses[i]
            xj = self.poses[j]

            # Compute error and Jacobians
            e_ij = self.compute_error(constraint)
            A_ij, B_ij = Graph.compute_jacobians(xi, xj)

            # Scale the angle to provide better condition of the H matrix
            e_ij, A_ij, B_ij, Omega_ij = self.angle_scaler.scale_controls(e_ij, A_ij, B_ij, Omega_ij)

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

            # Scale the angle to provide better condition of the H matrix
            e_ik, J_i, Omega_ik = self.angle_scaler.scale_observation(e_ik, J_i, Omega_ik)


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

    def add_delta_to_poses(self, delta):
        N = len(self.poses)
        for i in range(N):
            idx = slice(3 * i, 3 * i + 3)
            self.poses[i] += delta[idx]
            self.poses[i][2] = (self.poses[i][2] + pi) % (2 * pi) - pi

    def solve(self, debug=False):
        """
        Levenberg-Marquardt solver for Graph-Based SLAM.
        Efficient implementation: modifies only diagonal of H and restores it properly.
        """
        lm = LevenbergMarquardt()
        N = len(self.poses)
        self.H = np.zeros((3 * N, 3 * N))
        self.b = np.zeros((3 * N,))

        if debug:
            self.plot_comparison()

        for iteration in range(lm.get_max_iterations()):

            # Recompute Jacobian and residuals only if the cost is decreasing
            if lm.cost_is_decreasing():
                self.compute_H_b_of_controls()
                self.compute_H_b_of_observations()
                self.anchor_pose(0)
                lm.set_matrix(self.H)

            # Solve linear system with matrix damped using the levenberg-marquardt method
            delta_x = lm.solve_damped(-self.b)

            # Unscale the angle part of delta_x. Angle scaling is done to improve the condition of the H matrix.
            delta_x = self.angle_scaler.unscale_delta_x(delta_x)
            
            # Compute cost and evaluate the cost using levenberg-marquardt method
            cost = np.linalg.norm(delta_x)
            lm.set_cost(cost)

            # Debug output
            if debug:
                print( lm.get_iteration_info(iteration, cost))
                #self.plot_comparison()

            # Stop if converged
            if lm.cost_met_stop_criteria():
                print(f"Converged in {iteration + 1} iterations.")
                b = self.b
                self.plot_x_y_theta(b, "b")
                self.plot_x_y_theta(delta_x, "delta_x")
                self.plot_comparison()
                break

            # Apply Δx to poses only if cost is decreasing 
            if lm.cost_is_decreasing():
                self.add_delta_to_poses(delta_x)
        
        
        if debug:
            summary = lm.get_summary()
            return summary

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
        for i in range(len(theta)):
            theta[i] = ((theta[i] + pi) % (2 * pi) - pi) * 180 / pi

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
        ax2.plot(time_step, theta, 'r--', label='Theta')
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