from lego_robot import *
from math import sin, cos, pi, sqrt
import numpy as np
import os
import json
from slam_d_library import get_observations, write_cylinders
from extended_kalman_filter import EKF
from graph_slam import Graph
from lego_robot import LegoLogfile
from slam_d_library import get_observations

class System:
    def __init__(self, ekf, graph_slam, debug=False, sim=False):

        # Dependencies
        self.ekf = ekf
        self.graph_slam = graph_slam

        # Logging containers
        self.states = []
        self.covariances = []
        self.reference_cylinders = []
        self.matched_ref_cylinders = []
        self.observations_log = []
        self.controls_log = []

        # Configuration Data
        self.observations_sim = []
        self.controls_sim = []
        self.logfile = None
        self.debug = debug
        self.sim = sim

    def run(self):
        """
        Run the main system.  
        EKF is used as the front end to build a factor graph for the Graph SLAM backend. 
        Graph SLAM solves the graph to provide optimized poses of the robot.
        """
        # Initialize Logging Containers
        self.states = []
        self.covariances = []
        self.matched_ref_cylinders = []
        self.observations_log = []
        self.controls_log = []

        # Initialize EKF and GraphSLAM
        self.ekf.state = self.ekf.initial_state
        self.ekf.covariance = self.ekf.initial_covariance
        self.graph_slam.add_pose(self.ekf.initial_state)
        self.graph_slam.scanner_displacement = self.ekf.scanner_displacement

        # Run Kalman Filter
        if self.debug: print("******* EKF ********")
        for i in range(self.get_count()):

            # Motion Control (EKF Predict)
            control = self.get_control(i)
            x_i, x_j, sigma_ij = self.ekf.predict(control)
            if i ==0: sigma_ij = self.ekf.initial_covariance
            self.graph_slam.add_motion_constraint(i, i+1, x_i, x_j, sigma_ij)
            if self.debug: self.ekf.print_motion(i)

            # Landmark Observations (EKF Correct)
            observations = self.get_observations(i)
            for j in range(len(observations)):
                measurment, landmark = observations[j]
                Q = self.ekf.correct(measurment, landmark)
                self.graph_slam.add_observation_constraint(i+1, measurment, landmark, Q)
                if self.debug: self.ekf.print_observation(i,j)
            self.graph_slam.add_pose(self.ekf.state)

            # Data Logging
            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            self.matched_ref_cylinders.append([m[1] for m in observations])

        # Run GRAPH SLAM Solver
        if self.debug: print("\n****** GRAPH SLAM ********")
        self.plot_motion_control()
        self.graph_slam.ekf_states = self.states.copy()
        self.graph_slam.ekf_states.insert(0,self.ekf.initial_state)
        info = self.graph_slam.solve(self.debug)

        if self.debug:         
            print(info)
            self.graph_slam.print_summary()
            self.graph_slam.plot_comparison()

    def get_count(self):
        if self.sim:
            count = len(self.controls_sim)
        else:
            count = len(self.logfile.motor_ticks)
        return count

    def get_control(self, time_step):
        if self.sim:
            control = self.controls_sim[time_step]
        else:
            control = np.array(self.logfile.motor_ticks[time_step]) * self.ekf.ticks_to_mm
        self.controls_log.append(control)
        return control

    def get_observations(self, time_step):
        if self.sim:
            observations = self.observations_sim[time_step]
        else:
            observations = get_observations( self.logfile.scan_data[time_step], self.ekf.depth_jump, 
                self.ekf.minimum_valid_distance, self.ekf.cylinder_offset,
                self.ekf.state, self.ekf.scanner_displacement,
                self.reference_cylinders, self.ekf.max_cylinder_distance)
        self.observations_log.append(observations)
        return observations

    def read_logfiles(self):
        self.logfile = LegoLogfile()
        self.logfile.read(os.path.join("robot4_motors.txt"))
        self.logfile.read(os.path.join("robot4_scan.txt"))
        self.logfile.read(os.path.join("robot_arena_landmarks.txt"))
        self.reference_cylinders = [l[1:3] for l in self.logfile.landmarks]
        print('')

    def write_ekf_results(self, filename="Results_EKF.txt"):
        from math import sqrt, cos, sin
        from slam_d_library import write_cylinders
        with open(filename, "w") as f:
            for i in range(len(self.states)):
                x = tuple(self.states[i] + [
                    self.ekf.scanner_displacement * cos(self.states[i][2]),
                    self.ekf.scanner_displacement * sin(self.states[i][2]),
                    0.0])
                f.write(f"F {x[0]} {x[1]} {x[2]}\n")
                e = self.ekf.get_error_ellipse(self.covariances[i])
                q = (e + (sqrt(self.covariances[i][2,2]),))
                f.write(f"E {q[0]} {q[1]} {q[2]} {q[3]}\n")
                write_cylinders(f, "W C", self.matched_ref_cylinders[i])

    def write_graph_slam_results(self, filename="Results_Graph_Slam.txt"):
        with open(filename, "w") as f:
            for x, y, θ in self.graph_slam.poses:
                f.write(f"F {x} {y} {θ}\n")

    def read_json(self, json_obj):
        self.observations_sim = []
        for step in json_obj["observations"]:
            time_step = []
            for obs in step:
                observation = (obs["measure"], obs["landmark"])
                time_step.append(observation)
            self.observations_sim.append(time_step)

        self.controls_sim = []
        for step in json_obj["controls"]:
            time_step = (step["left"], step["right"])
            self.controls_sim.append(time_step)

    def load_json(self, filename):
        with open(filename, 'r') as f:
            self.read_json(json.load(f))

    def plot(self):
        import matplotlib.pyplot as plt
        import numpy as np

        # Create a Figure and a single Axes object
        # plt.subplots() returns a tuple: (Figure object, Axes object or array of Axes objects)
        fig, ax = plt.subplots() 

        p = np.array(self.graph_slam.poses)
        ax.plot(p[:,0], p[:,1], label='GraphSLAM')

        p = np.array(self.states)
        ax.plot(p[:,0], p[:,1], label='EKF')

        # Set properties of the Axes
        ax.set_title('EKF vs Graph SLAM')
        ax.set_xlabel('X-axis')
        ax.set_ylabel('Y-axis')
        ax.legend() # Display the legend based on the 'label' in ax.plot()
        ax.grid(True) # Add a grid to the plot

        # Display the plot
        plt.show()

    def plot_motion_control(self):
        import matplotlib.pyplot as plt
        import numpy as np

        # Setup data
        c = np.array([np.array(control) for control in self.controls_log])
        t = np.arange(c.shape[0])

        # Plot data
        fig, ax = plt.subplots() 
        ax.plot(t, c[:,0], label='right')
        ax.plot(t, c[:,1], label='left')
        ax.set_title('Motion Controls')
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Control Ticks')
        ax.legend()
        ax.grid(True) # Add a grid to the plot

def system_test(debug=False, test_data=False):
    #Build EKF and GraphSLAM Objects
    ekf = EKF()
    ekf.load_json("ekf.json")
    graph_slam = Graph()

    # Run Main Sytyem
    system = System(ekf, graph_slam, debug, test_data)
    system.read_logfiles()
    system.load_json("system.json")
    system.run()
    system.write_ekf_results("results_ekf.txt")
    system.write_graph_slam_results("results_graph_slam.txt")

if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    os.chdir("Data/LegoRobot")
    system_test( debug=True, test_data=False)

