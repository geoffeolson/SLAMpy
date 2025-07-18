import numpy as np
import json

class LevenbergMarquardt:
    def __init__(self):
        """
        Levenberg-Marquardt solver control logic for nonlinear least squares optimization.

        This class implements the generic control strategy for the Levenberg-Marquardt (LM) method,
        using Marquardt-style diagonal damping to adaptively blend Gauss-Newton and gradient descent.

        Damping Strategy:
        - The internal system matrix A (typically the Gauss-Newton Hessian approximation) is modified in-place
          by scaling only its diagonal entries: A_ii ← A_ii * (1 + λ), where λ is the damping factor.
        - This shifts the solution method between:
              • Gauss-Newton (λ → 0): faster convergence but sensitive to local minima
              • Gradient descent (λ → ∞): slower but more stable
        - Damping also improves numerical conditioning in ill-posed systems.

        Execution Control:
        - After each solve, the user must call `set_cost(cost)` with the step norm or cost.
        - This triggers adaptive updates to λ and sets two decision flags:
              • cost_is_decreasing(): 
                    True if the cost decreased — accept Δx and recompute A and b
              • cost_met_stop_criteria():
                    True if convergence is achieved — terminate the outer loop

        Matrix Access:
        - `solve_damped()` solves AΔx = b using the damped matrix.
        - Advanced users may call `dampen_matrix()` and `restore_damped_matrix()` directly
          to integrate with external solvers (e.g., sparse matrix solvers).
        """
        # Solver parameters
        self.lambda_init = 1e-4
        self.lambda_max = 1e+6 
        self.lambda_min = 1e-7
        self.lambda_incr_factor = 10.0
        self.lambda_decr_factor = 0.5
        self.cost_tol = 1e-3
        self.delta_cost_tol = 1e-4
        self.max_iterations = 100

        # Solver state variables
        self.prev_cost = 1e+10
        self.lamb = self.lambda_init
        self.cost_is_decreasing_ = True
        self.cost_met_stop_criteria_ = False

        # Matrix references
        self.A = None             # System matrix A in AΔx = b
        self.b = None             # Right-hand side vector
        self.diag_indices = None  # Cached diagonal indices of A
        self.original_diag = None # Cached diagonal values for restoring damping

    # --------------------------------------------------
    # Matrix setup and damped solve
    # --------------------------------------------------

    def set_matrix(self, A):
        """
        Set the system matrix A to be damped in-place.
        Stores a reference and caches the diagonal for restoration.
        """
        assert A.shape[0] == A.shape[1], "Matrix must be square"
        self.A = A
        self.original_diag = np.diag(A).copy()
        self.diag_indices = np.diag_indices_from(A)

    def solve_damped(self, b):
        """
        Solve the damped system (A + λI)Δx = b.

        Applies damping in-place to the diagonal of A, solves the linear system,
        and restores the original diagonal after solving.
        """
        self.dampen_matrix()
        delta_x = np.linalg.solve(self.A, b)
        self.restore_damped_matrix()
        return delta_x

    def dampen_matrix(self):
        """
        Apply diagonal damping: A_ii ← A_ii * (1 + λ)

        This operation modifies A in-place. It is intended for advanced users
        who want to use an external solver while maintaining the LM damping logic.
        """
        self.A[self.diag_indices] = self.original_diag * (1.0 + self.lamb)

    def restore_damped_matrix(self):
        """
        Restore the original (undamped) diagonal of A.

        This reverses `dampen_matrix()` and should be called after solving
        to return A to its pristine state.
        """
        self.A[self.diag_indices] = self.original_diag

    # --------------------------------------------------
    # Solver update logic
    # --------------------------------------------------

    def set_cost(self, cost):
        """
        Update the damping factor λ based on cost behavior.

        - If cost decreased: accept the update and reduce λ.
        - If cost increased: reject the update and increase λ.
        - If cost is sufficiently small or stable, declare convergence.
        """
        delta_cost = cost - self.prev_cost

        if delta_cost < 0:
            self.lamb = max(self.lambda_min, self.lamb * self.lambda_decr_factor)
            self.cost_is_decreasing_ = True

            if (cost < self.cost_tol) or (abs(delta_cost) < self.delta_cost_tol):
                self.cost_met_stop_criteria_ = True
                self.cost_is_decreasing_ = False
        else:
            self.lamb = min(self.lambda_max, self.lamb * self.lambda_incr_factor)
            self.cost_is_decreasing_ = False

        if self.cost_is_decreasing_:
            self.prev_cost = cost

    def cost_is_decreasing(self):
        """
        Return True if the last step reduced cost and should be accepted.
        """
        return self.cost_is_decreasing_

    def cost_met_stop_criteria(self):
        """
        Return True if convergence was achieved based on cost value or stability.
        """
        return self.cost_met_stop_criteria_

    def get_lambda(self):
        """Return the current value of λ."""
        return self.lamb

    def get_max_iterations(self):
        """Return the configured maximum number of solver iterations."""
        return self.max_iterations

    # --------------------------------------------------
    # Debug and reporting
    # --------------------------------------------------

    def get_iteration_info(self, iteration, cost):
        """
        Return a formatted string summarizing the current iteration.
        """
        return (
            f"iter:{iteration:<3} "
            f"cost:{cost:.6f}, "
            f"λ:{self.lamb:.2e}, "
            f"{'✓' if self.cost_is_decreasing_ else '✗'}"
        )

    def get_summary(self):
        """
        Return a summary dictionary with final solver state.
        """
        return {
            "final_lambda": self.lamb,
            "final_cost": self.prev_cost,
            "converged": self.cost_met_stop_criteria_,
            "max_iterations": self.max_iterations
        }

    def get_summary_string(self):
        """
        Return a human-readable string summarizing the final solver status.
        """
        return (
            f"Levenberg-Marquardt Summary:\n"
            f"  Converged: {self.cost_met_stop_criteria_}\n"
            f"  Final λ: {self.lamb:.2e}\n"
            f"  Final Cost: {self.prev_cost:.6f}\n"
            f"  Max Iterations: {self.max_iterations}"
        )

    # --------------------------------------------------
    # Serialization
    # --------------------------------------------------

    def write_json(self):
        """Export solver configuration as a JSON-serializable dictionary."""
        return {
            "lambda_init": self.lambda_init,
            "lambda_max": self.lambda_max,
            "lambda_min": self.lambda_min,
            "lambda_incr_factor": self.lambda_incr_factor,
            "lambda_decr_factor": self.lambda_decr_factor,
            "cost_tol": self.cost_tol,
            "delta_cost_tol": self.delta_cost_tol,
            "max_iterations": self.max_iterations
        }

    def read_json(self, json_obj):
        """
        Load solver configuration from a JSON-compatible dictionary.
        Resets the current λ to its initial value.
        """
        self.lambda_init = json_obj.get("lambda_init", self.lambda_init)
        self.lambda_max = json_obj.get("lambda_max", self.lambda_max)
        self.lambda_min = json_obj.get("lambda_min", self.lambda_min)
        self.lambda_incr_factor = json_obj.get("lambda_incr_factor", self.lambda_incr_factor)
        self.lambda_decr_factor = json_obj.get("lambda_decr_factor", self.lambda_decr_factor)
        self.cost_tol = json_obj.get("cost_tol", self.cost_tol)
        self.delta_cost_tol = json_obj.get("delta_cost_tol", self.delta_cost_tol)
        self.max_iterations = json_obj.get("max_iterations", self.max_iterations)
        self.lamb = self.lambda_init

    def save_to_file(self, filename):
        """Save the solver configuration to a JSON file."""
        with open(filename, 'w') as f:
            json.dump(self.write_json(), f, indent=2)

    def load_from_file(self, filename):
        """Load the solver configuration from a JSON file."""
        with open(filename, 'r') as f:
            self.read_json(json.load(f))


# class LevenbergMarquardt_old:
#     def __init__(self):
#         """
#         Levenberg-Marquardt solver control logic for nonlinear least squares optimization.

#         This class encapsulates the generic strategy of the Levenberg-Marquardt (LM) method,
#         using Marquardt-style diagonal damping to adaptively combine Gauss-Newton and gradient descent steps.

#         Damping Strategy:
#         - The input matrix A (usually the Hessian approximation) is modified in-place during solve
#           by scaling only its diagonal entries: A_ii ← A_ii * (1 + λ), where λ is the damping factor.
#         - This technique interpolates between Gauss-Newton (λ → 0) and gradient descent (λ → ∞),
#           preventing overshoot of the agressive Gauss-Newton method or slow convergence of gradient descent.  
#         - Damping can also improving numerical stability in ill-conditioned systems.

#         Convergence Logic:
#         - After each iteration, `set_cost(cost)` must be called with the norm of the update step
#           (or objective cost), and the algorithm adapts λ accordingly:
#               • If cost decreases: λ is reduced and the update is accepted.
#               • If cost increases: λ is increased and the update is rejected.
#         - The solver uses two flags to control program flow in the higher-level solver:
#               • cost_is_decreasing() 
#                 returns True if the last update decreased the cost. Here the linear system AΔx = b must be updated: 
#                     x must be updated with the solution vector Δx
#                     A, b must also be recomputed,
#               • cost_met_stop_criteria() 
#                 returns True if the solver has converged to a solution when the cost or change in cost is sufficiently small.
#         """
#         # Solver parameters
#         self.lambda_init = 1e-4
#         self.lambda_max = 1e+6 
#         self.lambda_min = 1e-7
#         self.lambda_incr_factor = 10.0
#         self.lambda_decr_factor = 0.5
#         self.cost_tol = 1e-3
#         self.delta_cost_tol = 1e-4
#         self.max_iterations = 100

#         # Solver state variables
#         self.prev_cost = 1e+10
#         self.lamb = self.lambda_init
#         self.cost_is_decreasing_ = True
#         self.cost_met_stop_criteria_ = False

#         # Matrix references and cached diagonal
#         self.A = None  # The matrix of the damped linear system Ax = b
#         self.b = None # The vector (right-hand side) of the linear system Ax = b
#         self.diag_indices = None  # Indices of the diagonal elements in A
#         self.original_diag = None  # Cached original diagonal of A for restoring after solve

#     # --------------------------------------------------
#     # Matrix setup and damped solve
#     # --------------------------------------------------

#     def set_matrix(self, A):
#         """
#         Set the matrix A of the linear system.
#         """
#         assert A.shape[0] == A.shape[1], "Matrix must be square"
#         self.A = A
#         self.original_diag = np.diag(A).copy()
#         self.diag_indices = np.diag_indices_from(A)

#     def solve_damped(self, b):
#         """
#         Solve (A + λI)Δx = b by modifying the diagonal of A in place.
#         The original diagonal is restored after solving.
#         """
#         self.dampen_matrix()
#         delta_x = np.linalg.solve(self.A, b)
#         self.restore_damped_matrix()
#         return delta_x

#     def dampen_matrix(self):
#         """
#         Appllies the damping factor λ to the internal matrix A. 
#         WARNING: this operation modifies the diagonal entries of A in place. This is an advanced 
#         method that can be used to modify the matrix A, so that it may be used in an external solver.
#         """
#         self.A[self.diag_indices] = self.original_diag * (1.0 + self.lamb)

#     def restore_damped_matrix(self):
#         """
#         Restores the internal matrix A to the origional, which removes damping. 
#         WARNING: this operation modifies the diagonal entries of A in place. This is an advanced 
#         method that can be used to modify the matrix A, so that it may be used in an external solver.
#         """
#         self.A[self.diag_indices] = self.original_diag

#     # --------------------------------------------------
#     # Solver update logic
#     # --------------------------------------------------

#     def set_cost(self, cost):
#         """
#         Update the cost and adjust λ accordingly:
#         - If cost decreased: accept update and reduce λ.
#         - If cost increased: reject update and increase λ.
#         Also sets convergence flag if stop criteria are met.
#         """
#         delta_cost = cost - self.prev_cost

#         if delta_cost < 0:
#             self.lamb = max(self.lambda_min, self.lamb * self.lambda_decr_factor)
#             self.cost_is_decreasing_ = True

#             if (cost < self.cost_tol) or (abs(delta_cost) < self.delta_cost_tol):
#                 self.cost_met_stop_criteria_ = True
#                 self.cost_is_decreasing_ = False

#         else:
#             self.lamb = min(self.lambda_max, self.lamb * self.lambda_incr_factor)
#             self.cost_is_decreasing_ = False

#         if self.cost_is_decreasing_:
#             self.prev_cost = cost

#     def cost_is_decreasing(self):
#         """
#         Return True if the last update was accepted (cost decreased).
#         """
#         return self.cost_is_decreasing_

#     def cost_met_stop_criteria(self):
#         """
#         Return True if convergence criteria were met (low or stable cost).
#         """
#         return self.cost_met_stop_criteria_

#     def get_lambda(self):
#         """
#         Return the current value of λ.
#         """
#         return self.lamb

#     def get_max_iterations(self):
#         """
#         Return the configured maximum number of solver iterations.
#         """
#         return self.max_iterations

#     # --------------------------------------------------
#     # Debug and reporting
#     # --------------------------------------------------

#     def get_iteration_info(self, iteration, cost):
#         """
#         Return a string summarizing the current iteration for debugging.
#         Format: iter:cost λ ✓ or ✗
#         """
#         return (
#             f"iter:{iteration:<3} "
#             f"cost:{cost:.6f}, "
#             f"λ:{self.lamb:.2e}, "
#             f"{'✓' if self.cost_is_decreasing_ else '✗'}"
#         )

#     def get_summary(self):
#         """
#         Return a dictionary summarizing the solver outcome.
#         Useful for logs or structured output.
#         """
#         return {
#             "final_lambda": self.lamb,
#             "final_cost": self.prev_cost,
#             "converged": self.cost_met_stop_criteria_,
#             "max_iterations": self.max_iterations
#         }

#     def get_summary_string(self):
#         """
#         Return a formatted summary string of the solver result.
#         Useful for printing at the end of a solve.
#         """
#         return (
#             f"Levenberg-Marquardt Summary:\n"
#             f"  Converged: {self.cost_met_stop_criteria_}\n"
#             f"  Final λ: {self.lamb:.2e}\n"
#             f"  Final Cost: {self.prev_cost:.6f}\n"
#             f"  Max Iterations: {self.max_iterations}"
#         )

#     # --------------------------------------------------
#     # Serialization
#     # --------------------------------------------------

#     def write_json(self):
#         """
#         Export solver configuration as a JSON-serializable dictionary.
#         """
#         return {
#             "lambda_init": self.lambda_init,
#             "lambda_max": self.lambda_max,
#             "lambda_min": self.lambda_min,
#             "lambda_incr_factor": self.lambda_incr_factor,
#             "lambda_decr_factor": self.lambda_decr_factor,
#             "cost_tol": self.cost_tol,
#             "delta_cost_tol": self.delta_cost_tol,
#             "max_iterations": self.max_iterations
#         }

#     def read_json(self, json_obj):
#         """
#         Load solver configuration from a JSON dictionary.
#         Resets the current λ to the initial value.
#         """
#         self.lambda_init = json_obj.get("lambda_init", self.lambda_init)
#         self.lambda_max = json_obj.get("lambda_max", self.lambda_max)
#         self.lambda_min = json_obj.get("lambda_min", self.lambda_min)
#         self.lambda_incr_factor = json_obj.get("lambda_incr_factor", self.lambda_incr_factor)
#         self.lambda_decr_factor = json_obj.get("lambda_decr_factor", self.lambda_decr_factor)
#         self.cost_tol = json_obj.get("cost_tol", self.cost_tol)
#         self.delta_cost_tol = json_obj.get("delta_cost_tol", self.delta_cost_tol)
#         self.max_iterations = json_obj.get("max_iterations", self.max_iterations)
#         self.lamb = self.lambda_init

#     def save_to_file(self, filename):
#         """
#         Save solver configuration to a JSON file.
#         """
#         with open(filename, 'w') as f:
#             json.dump(self.write_json(), f, indent=2)

#     def load_from_file(self, filename):
#         """
#         Load solver configuration from a JSON file.
#         """
#         with open(filename, 'r') as f:
#             self.read_json(json.load(f))
