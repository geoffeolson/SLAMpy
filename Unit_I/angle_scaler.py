import numpy as np

class AngleScaler:
    def __init__(self, scale=1000.0):
        """
        Class for applying angle scaling to residuals, Jacobians, and delta_x updates.

        The default scale converts radians to milliradians, improving the numerical
        conditioning of the system matrix H (especially during LM optimization).
        Scaling ensures that angle-related components have magnitudes similar to
        position-related components, reducing anisotropy in the optimization problem.
        """
        self.scale = scale

    def scale_controls(self, e_ij, A_ij, B_ij, Omega_ij):
        """
        Scale the angle component (index 2) in motion constraints.

        This scales:
            - The angle residual e_ij[2] ← e_ij[2] * scale
            - The corresponding Jacobian rows A_ij[2,:] and B_ij[2,:]
            - The information matrix by transforming it with S⁻¹ᵀ * Ω * S⁻¹,
              where S⁻¹ is a diagonal scaling matrix

        This improves numerical stability when angles are measured in radians
        and typically have smaller magnitudes than x/y positions.
        """
        e_ij[2] *= self.scale
        A_ij[2, :] *= self.scale
        B_ij[2, :] *= self.scale

        S_inv = np.diag([1.0, 1.0, 1.0 / self.scale])
        Omega_ij = S_inv.T @ Omega_ij @ S_inv

        return e_ij, A_ij, B_ij, Omega_ij

    def scale_observation(self, e_ik, J_i, Omega_ik):
        """
        Scale the angle component (index 1) in observation constraints.

        This scales:
            - The bearing residual e_ik[1] ← e_ik[1] * scale
            - The corresponding Jacobian row J_i[1,:]
            - The information matrix using the same diagonal scaling technique

        This brings the angle-bearing error into the same numeric range as position.
        """
        e_ik[1] *= self.scale
        J_i[1, :] *= self.scale

        S_inv = np.diag([1.0, 1.0 / self.scale])
        Omega_ik = S_inv.T @ Omega_ik @ S_inv

        return e_ik, J_i, Omega_ik

    def unscale_delta_x(self, delta_x):
        """
        Unscale the angle component in the delta_x update vector.

        Every third entry (starting at index 2) represents θ_i in the flattened pose vector.
        This method converts Δθ from milliradians back to radians:
            delta_x[3*i + 2] ← delta_x[3*i + 2] / scale
        """
        n = len(delta_x) // 3
        for i in range(n):
            delta_x[3 * i + 2] *= 1.0 / self.scale
        return delta_x
