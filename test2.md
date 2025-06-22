### Observation Constraint (Pose-to-Landmark)

Observation constraints relate a robot pose $x_i = (x\_i, y\_i, \theta\_i)$ to a known landmark position $l\_k = (l\_{kx}, l\_{ky})$. The expected observation is the displacement vector in rectangular coordinates from the robot to the landmark, as measured from the scanner frame.

Let the scanner position be:

$$
(x\_s, y\_s) = (x\_i + d \cos(\theta\_i),\ y\_i + d \sin(\theta\_i))
$$

Then the predicted observation $\mathbf{z}\_{ik}$ is:

$$
mathbf{z}\_{ik} = \begin{bmatrix}
l\_{kx} - x\_s \\
l\_{ky} - y\_s
\end{bmatrix}
\=
\begin{bmatrix}
l\_{kx} - x\_i - d \cos(\theta\_i) \\
l\_{ky} - y\_i - d \sin(\theta\_i)
\end{bmatrix}
$$

Let the actual observation be denoted as $\mathbf{z}\_{ik}^{\text{meas}}$ (e.g., as returned from the EKF).

The residual error is defined as:

$$
\mathbf{e}\_{ik} = \mathbf{z}\_{ik}^{\text{meas}} - \mathbf{z}\_{ik}
$$

#### Jacobian $A\_i \= \frac{\partial \mathbf{e}\_{ik}}{\partial \mathbf{x}\_i}$

This Jacobian describes how the observation error changes with respect to pose $x\_i = (x\_i, y\_i, \theta\_i)$. It is a 2×3 matrix:

$$
A\_i = \begin{bmatrix}
-1 & 0 & d \sin(\theta\_i) \\
0 & -1 & -d \cos(\theta\_i)
\end{bmatrix}
$$

#### Information Matrix $H$ and Vector $b$

For a single observation constraint, the contribution to the system is:

- Jacobian matrix: $A\_i$
- Information matrix: $H\_i = A\_i^\top \Omega A\_i$
- Information vector: $b\_i = A\_i^\top \Omega \mathbf{e}\_{ik}$

Where $\Omega \= \Sigma^{-1}$ is the inverse of the observation covariance matrix $\Sigma$.

#### Summary

- Observation constraints link a single pose to a known landmark.
- Measurements are in **rectangular coordinates**: a 2D displacement vector from the scanner to the landmark.
- The error $\mathbf{e}\_{ik}$ is the difference between the actual and predicted displacement vectors.
- The Jacobian $A_i$ is only with respect to one pose and is simpler than in motion constraints.
