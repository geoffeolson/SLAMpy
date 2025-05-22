# Graph-Based SLAM: Gauss-Newton Optimization Algorithm (Simplified and Complete)

## Overview

This section summarizes the algorithmic flow of Graph-Based SLAM as described in the original tutorial.

# Graph-Based SLAM: Gauss-Newton Optimization Algorithm (Simplified and Complete)
## Overview
This document describes the iterative optimization algorithm used for Graph-Based SLAM, based on the Grisetti et al. (2010) tutorial. The algorithm estimates robot poses that minimize the total squared error from relative motion and measurement constraints. The original description (Algorithm 1) lacked some practical implementation details, which are included here.
<img src="https://latex.codecogs.com/svg.latex?\Large&space;x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}" title="\Large x=\frac{-b\pm\sqrt{b^2-4ac}}{2a}" />
---
## Inputs
- Initial guess of robot poses: `x = [x1, x2, ..., xn]`
- Set of constraints `C = [(i, j, z_ij, Ω_ij)]` where:
  - `i`, `j`: indices of connected poses
  - `z_ij`: measured relative pose from node i to j
  - `Ω_ij`: information matrix (inverse of covariance) of the measurement
> Note: These inputs are fixed throughout the optimization. Only the pose estimates `x` change during iterations.
---
## Algorithm (Gauss-Newton Optimization)
```
repeat until convergence:
    1. Initialize H = zero sparse matrix (size: total_state_dim x total_state_dim)
       Initialize b = zero vector (size: total_state_dim)
       Initialize total_error = 0
    2. For each constraint (i, j, z_ij, Ω_ij):
        a. Predict measurement:      z_hat_ij = compute_relative_pose(x[i], x[j])
        b. Compute residual:         e_ij = z_ij - z_hat_ij
        c. Normalize angles (if 2D or 3D)
        d. Compute Jacobians:        A_ij = ∂e/∂x_i,  B_ij = ∂e/∂x_j
        e. Accumulate system:
           H_ii += A_ij.T * Ω_ij * A_ij
           H_ij += A_ij.T * Ω_ij * B_ij
           H_ji += B_ij.T * Ω_ij * A_ij
           H_jj += B_ij.T * Ω_ij * B_ij
           b_i  += A_ij.T * Ω_ij * e_ij
           b_j  += B_ij.T * Ω_ij * e_ij
           total_error += e_ij.T * Ω_ij * e_ij
    3. Apply gauge constraint:
       Fix the first pose (e.g., set H[0:3, 0:3] += large I or set Δx[0:3] = 0)
    4. Solve linear system: H * Δx = -b
       (Use sparse Cholesky decomposition)
    5. Update poses: x[i] += Δx[i] for all i
       Normalize angles if needed
    6. Check convergence:
       If norm(Δx) < threshold or error drop < threshold: break
```
---
## Function: `compute_jacobians()`
### 1. Purpose
Computes the Jacobians of the residual error function with respect to two poses connected by a constraint. Used in constructing the system matrix `H` and vector `b` for linear optimization.
### 2. Inputs and Outputs
- **Input**: A constraint tuple `(i, j, z_ij, Ω_ij)`
- **Output**: Matrices `A_ij = ∂e/∂x_i` and `B_ij = ∂e/∂x_j`, each 3×3
### 3. Equation
Let:
- \( x_i = [x_i, y_i, 	heta_i]^T \)
- \( x_j = [x_j, y_j, 	heta_j]^T \)
- \( \Delta t = [x_j - x_i, y_j - y_i]^T \)
- \( R_i \): 2×2 rotation matrix of pose \( i \)
Residual:
\[
\mathbf{e}_{ij} = 	ext{t2v}(T_i^{-1} T_j) - \mathbf{z}_{ij}
\]
### 4. Breakdown
- Compute transforms: \( T_i = 	ext{v2t}(x_i), T_j = 	ext{v2t}(x_j) \)
- Compute rotation: \( R_i \in SO(2) \)
- Derive Jacobians:
\[
A_{ij}[0:2, 0:2] = -R_i^T \
A_{ij}[0:2, 2] = R_i^T egin{bmatrix} 0 & -1 \ 1 & 0 \end{bmatrix} \Delta t \
A_{ij}[2, 2] = -1
\]
\[
B_{ij}[0:2, 0:2] = R_i^T, \quad B_{ij}[2, 2] = 1
\]
### 5. Notes
Each constraint is structured as:
```python
(i, j, z_ij, Omega_ij)
```
Used to compute residuals and derivatives required for optimization.
---
## Numerical Jacobian Testing
To verify the correctness of analytical Jacobians, numerical derivatives are computed as follows:
1. Perturb each variable of \( x_i \) and \( x_j \) by \( \epsilon \)
2. Recompute residual \( e_{ij} \)
3. Approximate Jacobians via:
\[
rac{\partial e}{\partial x_k} pprox 
rac{e(x + \epsilon) - e(x)}{\epsilon}
\]
This provides a test for each column of the Jacobian.
---
## Output
- Optimized poses `x*`
- Final information matrix `H` (optional)
---
## Notes and Enhancements
- Residuals and Jacobians must be recomputed at each iteration.
- Use angle normalization to keep headings within valid range.
- Gauge freedom must be removed to ensure a unique solution.
- Loop termination can be based on pose update norm or change in total error.
This document will be updated as the implementation progresses.



## Overview
This document describes the iterative optimization algorithm used for Graph-Based SLAM, based on the Grisetti et al. (2010) tutorial. The algorithm estimates robot poses that minimize the total squared error from relative motion and measurement constraints. The original description (Algorithm 1) lacked some practical implementation details, which are included here.

---

## Function: `compute_jacobians()`

### 1. Purpose
Computes the Jacobians of the residual error function with respect to two poses connected by a constraint. Used in constructing the system matrix `H` and vector `b` for linear optimization.

### 2. Inputs and Outputs
- **Input**: A constraint tuple `(i, j, z_ij, Ω_ij)`
- **Output**: Matrices `A_ij = ∂e/∂x_i` and `B_ij = ∂e/∂x_j`, each 3×3

### 3. Equation

<img src="https://latex.codecogs.com/svg.latex?\mathbf{e}_{ij}%20=%20\mathrm{t2v}(T_i^{-1}%20T_j)%20-%20\mathbf{z}_{ij}" title="e_ij = t2v(Ti^{-1} Tj) - z_ij" />

### 4. Breakdown

Let:

<img src="https://latex.codecogs.com/svg.latex?x_i%20=%20\begin{bmatrix}x_i%20\\%20y_i%20\\%20\Theta_i\end{bmatrix},%20\quad%20x_j%20=%20\begin{bmatrix}x_j%20\\%20y_j%20\\%20\Theta_j\end{bmatrix},%20\quad%20\Delta%20t%20=%20\begin{bmatrix}x_j%20-%20x_i%20\\%20y_j%20-%20y_i\end{bmatrix}" />


Then compute Jacobians:

**A_ij:**

<img src="https://latex.codecogs.com/svg.latex?A_{ij}[0:2,0:2]=-R_i^{op},\quad A_{ij}[0:2,2]=R_i^{op}\begin{bmatrix}0&-1\\1&0\end{bmatrix}\Delta%20t,\quad A_{ij}[2,2]=-1" title="A_ij Jacobian equations" />


**B_ij:**

<img src="https://latex.codecogs.com/svg.latex?B_{ij}[0:2,0:2]=R_i^{op},\quad%20B_{ij}[2,2]=1" title="B_ij Jacobian equations" />

---

## Numerical Jacobian Testing

To verify the correctness of analytical Jacobians, numerical derivatives are computed as follows:

<img src="https://latex.codecogs.com/svg.latex?rac{\partial{e}}{\partial{x_k}}%20pprox%20
rac{e(x+\epsilon)-e(x)}{\epsilon}" title="Numerical derivative formula" />

---

## Notes
Each constraint is structured as:
```python
(i, j, z_ij, Omega_ij)
```
Used to compute residuals and derivatives required for optimization.