import numpy as np

class LevenbergMarquardt:
    def __init__(self):
        """
        Class to encapsulate all the generic logic for a Levenberg Marquardt solver.
        It does not contain any information for a specific solver application.
        """
        self.M
        self.lambda_max = 1e+6 
        self.lambda_min = 1e-7
        self.lambda_init = 1e-4  # Initial lambda value
        self.cost_tol = 1e-3  # Cost tolerance criteria for convergence
        self.delta_cost_tol = 1e-4  # Change in cost tolerance criteria for convergence
        self.prev_cost = 1e+10  # Previous cost value to compare against for computing delta_cost
        self.lamb = self.lambda_init # Current lambda value
        self.max_iterations = 100  # Maximum number of solver iterations
        self.cost_decreased = True  # Use the computed delta x, so need to also recompute jacobians
        self.cost_met_stop_criteria = False  # Solver has converged to a solution so stop solver
        self.lambda_incr_factor = 10  # Factor to increase lambda when cost increases
        self.lambda_decr_factor = 0.5  # Factor to decrease lambda when cost decreases

    def set_cost(self, cost):
        delta_cost = cost - self.prev_cost

        # Cost is decreasing
        if delta_cost < 0:
            # Use the computed delta x value and decrease lambda to converge faster
            self.lamb = max(self.lambda_min, self.lamb * self.lambda_decr_factor) 
            self.cost_decreased = True

            # Check if cost met stop criteria
            if ((cost < self.cost_tol) or (abs(delta_cost) < self.delta_cost_tol)):
                # Solver has converged to a solution, so stop solver
                self.cost_met_stop_criteria = True
                self.cost_decreased = False

        # Cost is increasing
        else:
            # reject current update,
            # and increase lambda to prevent the overshoot that is increaing cost
            self.lamb = min(self.lambda_max, self.lamb * self.lambda_incr_factor)
            self.cost_decreased = False

        # Save the current cost as the previous cost for next iteration Only if the new update is accepted.
        # We don't want to store the previous cost for updates that were rejected.
        if self.cost_decreased:
            self.prev_cost = cost

    def get_lambda(self):
        return self.lamb

    def get_max_iterations(self):
        return self.max_iterations

    def cost_decreased(self):
        return self.cost_decreased

    def cost_met_stop_criteria(self):
        return self.cost_met_stop_criteria

    def set_matrix(self, matrix):
        self.M = matrix
        self.diag = np.diag(matrix)

    def get_dampted_matrix(self, matrix):
        H = self.H.copy()
        H[np.diag_indices_from(H)] = self.diag * ( 1 + self.lamb)
        return H