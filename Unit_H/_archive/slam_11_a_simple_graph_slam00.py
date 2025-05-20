"""
GraphSLAM class for simple 2D graph-based SLAM testing.

This version supports adding fixed 2D poses and relative pose constraints,
computing residuals, and preparing for Gauss-Newton optimization.
"""

import numpy as np

class GraphSLAM:
    def __init__(self):
        self.poses = []               # List of 2D poses: [x, y, theta]
        self.constraints = []         # List of tuples: (i, j, z_ij, Omega_ij)

    def add_pose(self, pose):
        """Add a new pose to the graph."""
        self.poses.append(np.array(pose))

    def add_constraint(self, i, j, z_ij, Omega_ij):
        """
        Add a relative pose constraint between pose i and pose j.
        z_ij: expected relative motion from i to j
        Omega_ij: 3x3 information matrix
        """
        self.constraints.append((i, j, np.array(z_ij), np.array(Omega_ij)))

    def compute_error(self, i, j, z_ij):
        """
        Compute the residual e_ij = z_ij - z_hat_ij
        where z_hat_ij is the predicted relative pose from i to j
        """
        xi = self.poses[i]
        xj = self.poses[j]

        # Compute predicted relative pose from xi to xj
        dx = xj[0] - xi[0]
        dy = xj[1] - xi[1]
        dtheta = xj[2] - xi[2]

        # Rotate into local frame of xi
        cos_theta = np.cos(-xi[2])
        sin_theta = np.sin(-xi[2])
        rot = np.array([[cos_theta, -sin_theta],
                        [sin_theta,  cos_theta]])
        trans = np.dot(rot, np.array([dx, dy]))

        z_hat_ij = np.array([trans[0], trans[1], dtheta])
        # Normalize angle
        z_hat_ij[2] = (z_hat_ij[2] + np.pi) % (2 * np.pi) - np.pi

        error = z_ij - z_hat_ij
        error[2] = (error[2] + np.pi) % (2 * np.pi) - np.pi
        return error

    def print_summary(self):
        print("Poses:")
        for idx, p in enumerate(self.poses):
            print(f"  x{idx} = {p}")
        print("Constraints:")
        for i, j, z, _ in self.constraints:
            print(f"  x{i} → x{j} : {z}")

if __name__ == '__main__':
    # create Graph Slam object
    gs = GraphSLAM()

    #Add Pose
    pose=0
    gs.add_pose(pose)

    #Add constraint
    i=0
    j=0
    z_ij=0
    Omega_ij = 0
    gs.add_constraint(i, j, z_ij, Omega_ij)

    #Compute Error 
    i=0
    j=0
    z_ij = 0
    gs.compute_error(i, j, z_ij)

    #print Results
    gs.print_summary()
