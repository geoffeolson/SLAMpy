### Algorithm Summary

1. **Initialization**

   - Initialize all poses (nodes) with estimates.
   - Construct a factor graph where each edge encodes a relative pose constraint between two poses.

2. **Linearization**

   - For each constraint (edge), compute the error between the predicted and observed relative pose.
   - Linearize the error function using Jacobians \( A_{ij} \) and \( B_{ij} \) with respect to the involved poses \( x_i \) and \( x_j \).

3. **Construct System**

   - Use Jacobians to populate the information matrix \( H \) and vector \( b \):

     \[
     H = J^\top \Omega J \qquad b = J^\top \Omega e
     \]

   - \( \Omega \) is the information (inverse covariance) matrix of the measurement.

4. **Solve**

   - Solve the linear system \( H \delta = -b \) using a sparse solver (e.g., Cholesky decomposition).
   - Update the poses: \( x \leftarrow x + \delta \)

5. **Iterate**

   - Repeat linearization and solving until convergence (change in \( \delta \) is small).

### Pose Definitions

\[
x_i = \begin{bmatrix} 
x_i \\ 
y_i \\ 
\theta_i 
\end{bmatrix},
\quad
x_j = \begin{bmatrix} 
x_j \\ 
y_j \\ 
\theta_j 
\end{bmatrix}, 
\quad
\Delta t = \begin{bmatrix} 
x_j - x_i \\ 
y_j - y_i 
\end{bmatrix}
\]

Let the rotation matrix of pose \( x_i \) be:

\[
R_i \in \mathbb{R}^{2 \times 2}
\]

and define the skew-symmetric matrix:

\[
S = \begin{bmatrix}
0 & -1 \\
1 & 0
\end{bmatrix}
\]

### Error Function and Observations

Each edge in the graph represents a constraint based on a relative pose observation:

\[
\mathbf{z}_{ij} = 
\begin{bmatrix} 
\Delta\mathbf{x}_{ij} & \Delta\mathbf{y}_{ij} \\ 
\Delta y_{ij} & \Delta\theta_{ij} 
\end{bmatrix}
\]

This observation represents the expected transformation from node \( i \) to node \( j \), measured in the coordinate frame of node \( i \).

The predicted relative pose based on current estimates is:

\[
\hat{z}_{ij} = \begin{bmatrix} 
R_i^\top (t_j - t_i) \\ 
\theta_j - \theta_i 
\end{bmatrix}
\]

The error function is the difference between the observed and predicted relative pose:

\[
\mathbf{e}_{ij} = \mathbf{z}_{ij} - \hat{z}_{ij}
\]

This error is used in the least-squares cost function that drives the optimization.

### Jacobians

The Jacobians of this function are with respect to the two involved poses \( x_i \) and \( x_j \).

\[
A_{ij} =
\begin{bmatrix}
-R_i^\top & R_i^\top S \Delta t \\
0_{1 \times 2} & -1
\end{bmatrix}
\]

\[
B_{ij} =
\begin{bmatrix}
R_i^\top & 0_{2 \times 1} \\
0_{1 \times 2} & 1
\end{bmatrix}
\]

### Summary

- These Jacobians are used to populate the sparse system matrix \( H \) and vector \( b \) in SLAM optimization.
- The rotation matrix \( R_i^\top \) transforms global coordinates to local frame \( i \).
- The skew-symmetric matrix \( S \) arises from the derivative of a rotation operation.
