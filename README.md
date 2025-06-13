## A Simple Graph-Based SLAM

This document explains the derivation of the Jacobians used in a simple Graph-Based SLAM implementation. It includes pose definitions, matrix notation, and the role of Jacobians in constructing the linear system for optimization.

---

### Algorithm Summary

1. **Initialization**

   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**

   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians $A_{ij}$ and $B_{ij}$ with respect to the involved poses $x_i$ and $x_j$.

3. **Construct System**

   - Use Jacobians to populate the information matrix $H$ and vector $b$:

$$
H = J^\top \Omega J \qquad b = J^\top \Omega e
$$
   
   - $\Omega$ is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system $H \delta = -b$ using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: $x \leftarrow x + \delta$

5. **Iterate**

   - Repeat linearization and solving until convergence (change in $\delta$ is small).

---

### Pose Definitions

$$
x_i = 
\begin{bmatrix} 
x_i \\ 
y_i \\ 
\theta_i 
\end{bmatrix}, 
\quad x_j = 
\begin{bmatrix} 
x_j \\ 
y_j \\ 
\theta_j 
\end{bmatrix}, 
\quad \Delta t = 
\begin{bmatrix} 
x_j - x_i \\ 
y_j - y_i 
\end{bmatrix}
$$

Let the rotation matrix of pose $x_i$ be:

$$
R_i \in \mathbb{R}^{2 \times 2}
$$

and define the skew-symmetric matrix:

$$
S = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
$$

---

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

$$
z_{ij} = 
\begin{bmatrix} 
\Delta x_{ij} \\ 
\Delta y_{ij} \\ 
\Delta \theta_{ij} 
\end{bmatrix}
$$

This observation represents the expected transformation from node $i$ to node $j$, measured in the coordinate frame of node $i$.

The predicted relative pose based on current estimates is:

$$
\hat{z}_{ij} = 
\begin{bmatrix} 
R_i^\top (t_j - t_i) \\ 
\theta_j - \theta_i 
\end{bmatrix}
$$

The error function is the difference between the observed and predicted relative pose:

$$
e_{ij} = z_{ij} - \hat{z}_{ij}
$$

This error is used in the least-squares cost function that drives the optimization.

---

### Jacobians

The Jacobians of this function are with respect to the two involved poses $x_i$ and $x_j$.

$$
A_{ij} =
\begin{bmatrix}
-R_i^\top & R_i^\top S \Delta t \\
0_{1 \times 2} & -1
\end{bmatrix}
$$

$$
B_{ij} =
\begin{bmatrix}
R_i^\top & 0_{2 \times 1} \\
0_{1 \times 2} & 1
\end{bmatrix}
$$

---

## 6. Linear System Construction

We now describe how to build the linear system for optimization, following the Gauss-Newton method.

At each iteration, we linearize the error function around the current estimate and solve:  
$H \Delta x = -b$

where:

- $\Delta x$ is the update vector for all poses.
- $H$ is the system matrix (information matrix).
- $b$ is the right-hand-side vector.

---

### 6.1 Contribution from Each Constraint

For each constraint $(i, j, z\_{ij}, \Omega\_{ij})$, we compute:

- The error: $e\_{ij} = z\_{ij} - \hat{z}\_{ij}$
- The Jacobians: $A\_{ij} = \frac{\partial e\_{ij}}{\partial x\_i}, \quad B\_{ij} = \frac{\partial e\_{ij}}{\partial x\_j}$

The contribution of this constraint to the linear system is:

- $H\_{ii} += A\_{ij}^\top \Omega\_{ij} A\_{ij}$
- $H\_{ij} += A\_{ij}^\top \Omega\_{ij} B\_{ij}$
- $H\_{ji} += B\_{ij}^\top \Omega\_{ij} A\_{ij}$
- $H\_{jj} += B\_{ij}^\top \Omega\_{ij} B\_{ij}$
- $b\_i += A\_{ij}^\top \Omega\_{ij} e\_{ij}$
- $b\_j += B\_{ij}^\top \Omega\_{ij} e\_{ij}$

---

### 6.2 Global Assembly

We initialize:

- $H$ as a zero matrix of size $3N \times 3N$ where $N$ is the number of poses.
- $b$ as a zero vector of size $3N$.

For each constraint, the corresponding blocks of $H$ and $b$ are updated at the appropriate positions:

- Pose $i$ corresponds to block index $3i : 3i+3$.
- Pose $j$ corresponds to block index $3j : 3j+3$.

The full system accumulates all contributions from all constraints.

---

### 6.3 Notes

- $\Omega\_{ij}$ is the information matrix of the constraint (usually diagonal).
- The structure of $H$ remains sparse because each constraint only affects two poses.
- After building $H$ and $b$, we solve: $H \Delta x = -b$ using sparse linear solvers (e.g. Cholesky).


### Summary

- These Jacobians are used to populate the sparse system matrix $H$ and vector $b$ in SLAM optimization.
- The rotation matrix $R_i^\top$ transforms global coordinates to local frame $i$.
- The skew-symmetric matrix $S$ arises from the derivative of a rotation operation.
