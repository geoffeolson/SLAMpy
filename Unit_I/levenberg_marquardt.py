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
        """
        # Solver hyperparameters and state
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

        # Matrix references and cached diagonal
        self.M = None
        self.diag_indices = None
        self.original_diag = None

    # --------------------------------------------------
    # Matrix setup and damped solve
    # --------------------------------------------------

    def set_matrix(self, matrix):
        """
        Set the matrix to be damped (typically H = JᵀΩJ).
        Stores a reference to the matrix and caches its diagonal for damping.
        """
        assert matrix.shape[0] == matrix.shape[1], "Matrix must be square"
        self.M = matrix
        self.original_diag = np.diag(matrix).copy()
        self.diag_indices = np.diag_indices_from(matrix)

    def solve_damped(self, rhs):
        """
        Solve (H + λI)x = rhs by modifying the diagonal of H in place.
        The original diagonal is restored after solving.
        """
        self.M[self.diag_indices] = self.original_diag * (1.0 + self.lamb)
        x = np.linalg.solve(self.M, rhs)
        self.M[self.diag_indices] = self.original_diag  # Restore original diagonal
        return x

    # --------------------------------------------------
    # Solver update logic
    # --------------------------------------------------

    def set_cost(self, cost):
        """
        Update the cost and adjust λ accordingly:
        - If cost decreased: accept update and reduce λ.
        - If cost increased: reject update and increase λ.
        Also sets convergence flag if stop criteria are met.
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
        Return True if the last update was accepted (cost decreased).
        """
        return self.cost_is_decreasing_

    def cost_met_stop_criteria(self):
        """
        Return True if convergence criteria were met (low or stable cost).
        """
        return self.cost_met_stop_criteria_

    def get_lambda(self):
        """
        Return the current value of λ.
        """
        return self.lamb

    def get_max_iterations(self):
        """
        Return the configured maximum number of solver iterations.
        """
        return self.max_iterations

    # --------------------------------------------------
    # Debug and reporting
    # --------------------------------------------------

    def get_iteration_info(self, iteration, cost):
        """
        Return a string summarizing the current iteration for debugging.
        Format: iter:cost λ ✓ or ✗
        """
        return (
            f"iter:{iteration:<3} "
            f"cost:{cost:.6f}, "
            f"λ:{self.lamb:.2e}, "
            f"{'✓' if self.cost_is_decreasing_ else '✗'}"
        )

    def get_summary(self):
        """
        Return a dictionary summarizing the solver outcome.
        Useful for logs or structured output.
        """
        return {
            "final_lambda": self.lamb,
            "final_cost": self.prev_cost,
            "converged": self.cost_met_stop_criteria_,
            "max_iterations": self.max_iterations
        }

    def get_summary_string(self):
        """
        Return a formatted summary string of the solver result.
        Useful for printing at the end of a solve.
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
        """
        Export solver configuration as a JSON-serializable dictionary.
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
        Load solver configuration from a JSON dictionary.
        Resets the current λ to the initial value.
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
