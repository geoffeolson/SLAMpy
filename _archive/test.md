### Observation Constraint (Pose-to-Landmark)

Observation constraints relate a robot pose $x_i = (x_i, y_i, \theta_i)$ to a known landmark position $l_k = (l_{kx}, l_{ky})$. The expected observation is the displacement vector in rectangular coordinates from the robot to the landmark, as measured from the scanner frame.

Let the scanner position be:
$$
(x_s, y_s) = (x_i + d \cos(\theta_i),\ y_i + d \sin(\theta_i))
$$

Then the predicted observation $\mathbf{z}_{ik}$ is:
$$
\mathbf{z}_{ik} = \begin{bmatrix}
l_{kx} - x_s \\
l_{ky} - y_s
\end{bmatrix}
=
\begin{bmatrix}
l_{kx} - x_i - d \cos(\theta_i) \\
l_{ky} - y_i - d \sin(\theta_i)
\end{bmatrix}
$$

Let the actual observation be denoted as $\mathbf{z}_{ik}^{\text{meas}}$ (e.g., as returned from the EKF).

The residual error is defined as:
$$
\mathbf{e}_{ik} = \mathbf{z}_{ik}^{\text{meas}} - \mathbf{z}_{ik}
$$

#### Jacobian $A_i = \frac{\partial \mathbf{e}_{ik}}{\partial \mathbf{x}_i}$

This Jacobian describes how the observation error changes with respect to pose $x_i = (x_i, y_i, \theta_i)$. It is a 2×3 matrix:

$$
A_i = \begin{bmatrix}
-1 & 0 & d \sin(\theta_i) \\
0 & -1 & -d \cos(\theta_i)
\end{bmatrix}
$$

#### Information Matrix $H$ and Vector $b$

For a single observation constraint, the contribution to the system is:

- Jacobian matrix: $A_i$
- Information matrix: $H_i = A_i^\top \Omega A_i$
- Information vector: $b_i = A_i^\top \Omega \mathbf{e}_{ik}$

Where $\Omega = \Sigma^{-1}$ is the inverse of the observation covariance matrix $\Sigma$.

#### Summary

- Observation constraints link a single pose to a known landmark.
- Measurements are in **rectangular coordinates**: a 2D displacement vector from the scanner to the landmark.
- The error $\mathbf{e}_{ik}$ is the difference between the actual and predicted displacement vectors.
- The Jacobian $A_i$ is only with respect to one pose and is simpler than in motion constraints.
