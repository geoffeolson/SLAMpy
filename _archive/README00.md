# Graph-Based SLAM: Gauss-Newton Optimization Algorithm (Simplified and Complete)

## Overview
This document describes the iterative optimization algorithm used for Graph-Based SLAM, based on the Grisetti et al. (2010) tutorial. The algorithm estimates robot poses that minimize the total squared error from relative motion and measurement constraints. The original description (Algorithm 1) lacked some practical implementation details, which are included here.

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
