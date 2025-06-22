$\mathrel{\+\=}$
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

### Summary

- These Jacobians are used to populate the sparse system matrix $H$ and vector $b$ in SLAM optimization.
- The rotation matrix $R_i^\top$ transforms global coordinates to local frame $i$.
- The skew-symmetric matrix $S$ arises from the derivative of a rotation operation.

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

---

## 7. Solving the Linear System

Once the linear system $H \Delta x = -b$ is constructed, we solve for the pose update $\Delta x$.

### 7.1 Gauge Freedom

The SLAM problem has gauge freedom — it is underdetermined because the full trajectory can be translated and rotated without affecting relative measurements.  
To eliminate this ambiguity, we fix one pose (typically the first pose $x_0$) to an arbitrary value. This is called **anchoring**.

We apply this constraint directly to the linear system by modifying $H$ and $b$:

- Set rows and columns corresponding to pose $x_0$ to identity:

$$
H\_{0:3, 0:3} = I\_{3 \times 3}
$$

- Set the corresponding entries in $b$ to zero:

$$
b\_{0:3} = 0
$$

This pins the first pose to its initial estimate.

---

### 7.2 Solving the Linear System

After anchoring, we solve the linear system:

$$
H \Delta x = -b
$$

The update step is:

$$
\Delta x = - H^{-1} b
$$

In practice, $H$ is sparse and symmetric positive definite, so we use sparse solvers such as Cholesky factorization to efficiently compute $\Delta x$.

---

### 7.3 Applying the Pose Updates

Once the increment $\Delta x$ is computed, we apply it to update the poses:

For each pose $x_i$:

$$
x_i \leftarrow x_i + \Delta x_i
$$

where $\Delta x_i$ is the corresponding 3-element subvector extracted from $\Delta x$:

- Pose $i$ corresponds to indices:

$$
\Delta x_i = \Delta x\_{3i : 3i+3}
$$

The updated poses are then used in the next Gauss-Newton iteration.

---

### 7.4 Convergence Criteria

We iterate:

1. Build $H$ and $b$ from current estimates.
2. Solve $H \Delta x = -b$.
3. Apply pose updates.

Until one of the following is satisfied:

- The norm of $\Delta x$ becomes sufficiently small.
- The maximum number of iterations is reached.


### 8.0 Observation Constraints

Observation constraints are added when the robot observes a known landmark. These constraints relate a single robot pose to a landmark position using a measurement model.

Each observation provides a constraint between a robot pose $\mathbf{x}\_i$ and a known landmark position $\mathbf{l}\_k$. The error function compares the expected observation (based on the pose and the landmark) with the actual measurement.

#### Definitions

Let this be the robot pose at time $i$:

$$
\mathbf{x}\_i = \begin{bmatrix} 
x\_i \\ 
y\_i \\ 
\theta\_i 
\end{bmatrix}
$$

let this be the position of landmark $k$

$$
\mathbf{l}\_k = \begin{bmatrix} 
x\_k \\ 
y\_k 
\end{bmatrix}
$$ 

Let this be the observed relative position from pose $i$ to landmark $k$, in the robot's reference frame:

$$
\mathbf{z}\_{ik} = 
\begin{bmatrix} 
\Delta x \\ 
\Delta y 
\end{bmatrix}
$$ 

The expected observation $\hat{\mathbf{z}}\_{ik}$ is computed by:

$$
\hat{\mathbf{z}}\_{ik} = R(\theta\_i)^\top \cdot \left( \mathbf{l}\_k - \mathbf{t}\_i \right)
$$

where: 

$$
\mathbf{t}\_i = \begin{bmatrix} x\_i \\ 
y\_i \end{bmatrix}
$$ 

is the translation part of pose $\mathbf{x}\_i$ and $R(\theta\_i)$ is the 2D rotation matrix:

$$
R(\theta\_i) = 
\begin{bmatrix}
\cos\theta\_i & -\sin\theta\_i \\
\sin\theta\_i & \cos\theta\_i
\end{bmatrix}
$$

#### Error Function

The error is defined as the difference between the predicted and observed measurements:

$$
\mathbf{e}\_{ik} = \hat{\mathbf{z}}\_{ik} - \mathbf{z}\_{ik}
$$

#### Jacobian

The Jacobian $\mathbf{J}\_{ik}$ is the derivative of the error function with respect to the pose $\mathbf{x}\_i$.

Let:

- $\Delta x = x\_k - x\_i$
- $\Delta y = y\_k - y\_i$
- $c = \cos\theta\_i$
- $s = \sin\theta\_i$

Then the Jacobian is:

$$
\mathbf{J}\_{ik} = \frac{\partial \mathbf{e}\_{ik}}{\partial \mathbf{x}\_i} = \begin{bmatrix}
\-c & - s & - s \cdot \Delta x + c \cdot \Delta y \\
s & - c & - c \cdot \Delta x - s \cdot \Delta y
\end{bmatrix}
$$

Note that this is a $2 \times 3$ matrix since the observation error is 2D, and the pose has 3 parameters.

#### Linear System Contribution

Each observation constraint contributes to the linear system:

- Error vector $\mathbf{e}\_{ik}$ is added to the stacked residual vector $\mathbf{b}$
- Jacobian $\mathbf{J}\_{ik}$ is used to form the local information matrix:

$$
\mathbf{H}\_{ii} \mathrel{\+\=} \mathbf{J}\_{ik}^\top \, \Omega\_{ik} \, \mathbf{J}\_{ik}
$$

- And the right-hand side vector:

$$
\mathbf{b}\_i \mathrel{\+\=} \mathbf{J}\_{ik}^\top \, \Omega\_{ik} \, \mathbf{e}\_{ik}
$$

Here $\Omega\_{ik} = \Sigma\_{ik}^{-1}$ is the information matrix associated with the measurement noise.

#### Summary

- Observation constraints connect one robot pose $\mathbf{x}\_i$ to one known landmark $\mathbf{l}\_k$
- The error is defined in rectangular (x, y) coordinates
- Only one block in $\mathbf{H}$ (the pose block) is updated, since the landmark positions are assumed fixed
- This formulation avoids dependencies between multiple robot poses, unlike motion constraints
