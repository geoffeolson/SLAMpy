import numpy as np
import json

class LevenbergMarquardt:
    def __init__(self):
        """
        Levenberg-Marquardt solver control logic for nonlinear least squares optimization.

        This class encapsulates the generic strategy of the Levenberg-Marquardt (LM) method,
        using Marquardt-style diagonal damping to adaptively combine Gauss-Newton and gradient descent steps.

        Damping Strategy:
        - The input matrix H (usually the Hessian approximation) is modified in-place during solve
          by scaling only its diagonal entries: H_ii ← H_ii * (1 + λ), where λ is the damping factor.
        - This technique interpolates between Gauss-Newton (λ → 0) and gradient descent (λ → ∞),
          improving numerical stability in ill-conditioned systems.

        Convergence Logic:
        - After each iteration, `set_cost(cost)` must be called with the norm of the update step
          (or objective cost), and the algorithm adapts λ accordingly:
              • If cost decreases: λ is reduced and the update is accepted.
              • If cost increases: λ is increased and the update is rejected.
        - The solver uses two flags to guide control flow in the higher-level solver:
              • `cost_is_decreasing()` returns True if the last update step was accepted.
              • `cost_met_stop_criteria()` returns True if the cost is sufficiently small or stable.

        Key Methods:
            - `set_matrix(H)`: Sets the matrix to be damped and caches its diagonal.
            - `solve_damped(rhs)`: Applies in-place diagonal damping, solves Hx = rhs, then restores H.
            - `get_iteration_info(...)`: Returns a debug string for external logging.
            - `write_json()` / `read_json(obj)`: Serialize/deserialize solver parameters.
            - `get_summary()` / `get_summary_string()`: Return final solver diagnostics.
        """
        # Initialize solver parameters
        self.M = None  # Matrix to be damped
        self.lambda_max = 1e+6 
        self.lambda_min = 1e-7
        self.lambda_init = 1e-4
        self.cost_tol = 1e-3
        self.delta_cost_tol = 1e-4
        self.prev_cost = 1e+10
        self.lamb = self.lambda_init
        self.max_iterations = 100
        self.cost_is_decreasing_ = True
        self.cost_met_stop_criteria_ = False
        self.lambda_incr_factor = 10.0
        self.lambda_decr_factor = 0.5
        self.diag_indices = None
        self.original_diag = None

    def set_cost(self, cost):
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

    def get_lambda(self):
        return self.lamb

    def get_max_iterations(self):
        return self.max_iterations

    def cost_is_decreasing(self):
        return self.cost_is_decreasing_

    def cost_met_stop_criteria(self):
        return self.cost_met_stop_criteria_

    def set_matrix(self, matrix):
        assert matrix.shape[0] == matrix.shape[1], "Matrix must be square"
        self.M = matrix
        self.original_diag = np.diag(matrix).copy()
        self.diag_indices = np.diag_indices_from(matrix)

    def solve_damped(self, rhs):
        self.M[self.diag_indices] = self.original_diag * (1.0 + self.lamb)
        x = np.linalg.solve(self.M, rhs)
        self.M[self.diag_indices] = self.original_diag
        return x

    def get_iteration_info(self, iteration, cost):
        """
        Returns a formatted string with iteration info for debug output.
        """
        return (
            f"iter:{iteration:<3}"
            f"cost:{cost:.6f}, "
            f"λ:{self.lamb:.2e}, "
            f"{'✓' if self.cost_is_decreasing_ else '✗'}"
        )

    def get_summary(self):
        """
        Return a dictionary summary of solver result.
        """
        return {
            "final_lambda": self.lamb,
            "final_cost": self.prev_cost,
            "converged": self.cost_met_stop_criteria_,
            "max_iterations": self.max_iterations
        }

    def get_summary_string(self):
        """
        Return a human-readable string summary of solver result.
        """
        return (
            f"Levenberg-Marquardt Summary:\n"
            f"  Converged: {self.cost_met_stop_criteria_}\n"
            f"  Final λ: {self.lamb:.2e}\n"
            f"  Final Cost: {self.prev_cost:.6f}\n"
            f"  Max Iterations: {self.max_iterations}"
        )

    def write_json(self):
        """
        Export solver parameters as a JSON-serializable dictionary.
        """
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
        Load solver parameters from a JSON dictionary.
        """
        self.lambda_init = json_obj.get("lambda_init", self.lambda_init)
        self.lambda_max = json_obj.get("lambda_max", self.lambda_max)
        self.lambda_min = json_obj.get("lambda_min", self.lambda_min)
        self.lambda_incr_factor = json_obj.get("lambda_incr_factor", self.lambda_incr_factor)
        self.lambda_decr_factor = json_obj.get("lambda_decr_factor", self.lambda_decr_factor)
        self.cost_tol = json_obj.get("cost_tol", self.cost_tol)
        self.delta_cost_tol = json_obj.get("delta_cost_tol", self.delta_cost_tol)
        self.max_iterations = json_obj.get("max_iterations", self.max_iterations)
        self.lamb = self.lambda_init  # Reset current lambda

    def save_to_file(self, filename):
        """
        Save solver configuration to a JSON file.
        """
        with open(filename, 'w') as f:
            json.dump(self.write_json(), f, indent=2)

    def load_from_file(self, filename):
        """
        Load solver configuration from a JSON file.
        """
        with open(filename, 'r') as f:
            self.read_json(json.load(f))

##################################################################
#     DEPRICATED CODE
##################################################################
# class LevenbergMarquardt_old:
#     def __init__(self):
#         """
#         Class to encapsulate all the generic logic for a Levenberg Marquardt solver.
#         It does not contain any information for a specific solver application.
#         """
#         self.M = None  # Matrix to be damped
#         self.lambda_max = 1e+6 
#         self.lambda_min = 1e-7
#         self.lambda_init = 1e-4  # Initial lambda value
#         self.cost_tol = 1e-3  # Cost tolerance criteria for convergence
#         self.delta_cost_tol = 1e-4  # Change in cost tolerance criteria for convergence
#         self.prev_cost = 1e+10  # Previous cost value to compare against for computing delta_cost
#         self.lamb = self.lambda_init # Current lambda value
#         self.max_iterations = 100  # Maximum number of solver iterations
#         self.cost_is_decreasing_ = True  # Use the computed delta x, so need to also recompute jacobians
#         self.cost_met_stop_criteria_ = False  # Solver has converged to a solution so stop solver
#         self.lambda_incr_factor = 10  # Factor to increase lambda when cost increases
#         self.lambda_decr_factor = 0.5  # Factor to decrease lambda when cost decreases
#         self.diag_indices = None  # Indices of the diagonal elements in the matrix
#         self.original_diag = None

#     def set_cost(self, cost):
#         delta_cost = cost - self.prev_cost

#         # Cost is decreasing
#         if delta_cost < 0:
#             # Use the computed delta x value and decrease lambda to converge faster
#             self.lamb = max(self.lambda_min, self.lamb * self.lambda_decr_factor) 
#             self.cost_is_decreasing_ = True

#             # Check if cost met stop criteria
#             if ((cost < self.cost_tol) or (abs(delta_cost) < self.delta_cost_tol)):
#                 # Solver has converged to a solution, so stop solver
#                 self.cost_met_stop_criteria_ = True
#                 self.cost_is_decreasing_ = False

#         # Cost is increasing
#         else:
#             # reject current update,
#             # and increase lambda to prevent the overshoot that is increasing cost
#             self.lamb = min(self.lambda_max, self.lamb * self.lambda_incr_factor)
#             self.cost_is_decreasing_ = False

#         # Save the current cost as the previous cost for next iteration Only if the new update is accepted.
#         # We don't want to store the previous cost for updates that were rejected.
#         if self.cost_is_decreasing_:
#             self.prev_cost = cost

#     def get_lambda(self):
#         return self.lamb

#     def get_max_iterations(self):
#         return self.max_iterations

#     def cost_is_decreasing(self):
#         return self.cost_is_decreasing_

#     def cost_met_stop_criteria(self):
#         return self.cost_met_stop_criteria_

#     def set_matrix(self, matrix):
#         self.M = matrix
#         self.original_diag = np.diag(matrix).copy()
#         self.diag_indices = np.diag_indices_from(matrix)

#     # def get_damped_matrix(self):
#     #     M = self.M.copy()
#     #     M[np.diag_indices_from(M)] = self.origional_diag * ( 1 + self.lamb)
#     #     return M

#     def solve_damped(self, rhs):
#         self.M[self.diag_indices] = self.original_diag * (1.0 + self.lamb)
#         x = np.linalg.solve(self.M, rhs)
#         # Restore the original diagonal
#         self.M[self.diag_indices] = self.original_diag
#         return x