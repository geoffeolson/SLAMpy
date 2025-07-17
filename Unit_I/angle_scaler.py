import numpy as np

class AngleScaler:
    def __init__(self, scale=1000.0):
        """
        Class for applying angle scaling to residuals, Jacobians, and delta_x updates.
        The default scale converts radians to milliradians.
        """
        self.scale = scale

    def scale_controls(self, e_ij, A_ij, B_ij, Omega_ij):
        """
        Scale angle component (index 2) in motion residuals and Jacobians.
        Applies:
            - residual: e_ij[2] *= scale
            - Jacobians: A_ij[2, :] *= scale, B_ij[2, :] *= scale
            - Info matrix: S_inv.T @ Omega @ S_inv
        """
        e_ij[2] *= self.scale
        A_ij[2, :] *= self.scale
        B_ij[2, :] *= self.scale

        S_inv = np.diag([1.0, 1.0, 1.0 / self.scale])
        Omega_ij = S_inv.T @ Omega_ij @ S_inv

        return e_ij, A_ij, B_ij, Omega_ij

    def scale_observation(self, e_ik, J_i, Omega_ik):
        """
        Scale bearing component (index 1) in observation residuals and Jacobians.
        Applies:
            - residual: e_ik[1] *= scale
            - Jacobian: J_i[1, :] *= scale
            - Info matrix: S_inv.T @ Omega @ S_inv
        """
        e_ik[1] *= self.scale
        J_i[1, :] *= self.scale

        S_inv = np.diag([1.0, 1.0 / self.scale])
        Omega_ik = S_inv.T @ Omega_ik @ S_inv

        return e_ik, J_i, Omega_ik

    def unscale_delta_x(self, delta_x):
        """
        Unscale angle components in the update vector (every 3rd index starting from 2).
        Converts from milliradians back to radians.
        """
        n = len(delta_x) // 3
        for i in range(n):
            delta_x[3 * i + 2] *= 1.0 / self.scale
        return delta_x
