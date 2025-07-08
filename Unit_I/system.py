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
    def __init__(self, ekf, graph_slam, debug=False, test_data=False):

        # Dependencies
        self.ekf = ekf
        self.graph_slam = graph_slam

        # Logging containers
        self.states = []
        self.covariances = []
        self.matched_ref_cylinders = []

        # Configuration Data
        self.observations = None
        self.controls = None
        self.controls2 = None
        self.debug = debug
        self.test_data = test_data

    def run(self, logfile):

        #Run EKF
        if self.debug: print("******* EKF ********")
        reference_cylinders = [l[1:3] for l in logfile.landmarks]
        if self.test_data: logfile.motor_ticks = self.controls
        self.controls2 = []
        for i in range(len(logfile.motor_ticks)):
            # Odometry
            control = np.array(logfile.motor_ticks[i]) * self.ekf.ticks_to_mm
            self.controls2.append(control)
            prev_state = self.ekf.state
            x_i, x_j, sigma_ij = self.ekf.predict(control)
            #motion = self.ekf.state - prev_state
            #################################################
            #if i==35:breakpoint()
            #################################################
            self.graph_slam.add_motion_constraint(i, x_i, x_j, sigma_ij)
            #self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            if self.debug: self.ekf.print_motion(i)

            # Obervations
            observations = get_observations( logfile.scan_data[i], self.ekf.depth_jump, 
                self.ekf.minimum_valid_distance, self.ekf.cylinder_offset,
                self.ekf.state, self.ekf.scanner_displacement,
                reference_cylinders, self.ekf.max_cylinder_distance)
            if self.test_data: observations = self.observations[i]
            for j in range(len(observations)):
                measurment, landmark = observations[j]
                Q = self.ekf.correct(measurment, landmark)
                self.graph_slam.add_observation_constraint(i+1, measurment, landmark, Q)
                if self.debug: self.ekf.print_observation(i,j)
            self.graph_slam.add_pose(self.ekf.state)

            # Log state, covariance, and matched cylinders for later output.
            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            self.matched_ref_cylinders.append([m[1] for m in observations])

        #Run Graph SLAM Solver
        ##################################
        #self.test_print()
        ########################

        if self.debug: print("\n****** GRAPH SLAM ********")
        self.plot_motion_control()
        self.graph_slam.ekf_states = self.states.copy()
        self.graph_slam.solve(self.debug, max_iterations=50, tol=0.001)
        self.graph_slam.print_summary()
        #self.graph_slam.plot()
        #self.plot()

    def read_logfiles(self, filename):
        logfile = LegoLogfile()
        logfile.read(os.path.join("robot4_motors.txt"))
        logfile.read(os.path.join("robot4_scan.txt"))
        logfile.read(os.path.join("robot_arena_landmarks.txt"))

        self.control = [np.array(ctrl) * self.ekf.ticks_to_mm for ctrl in logfile.motor_ticks]

        self.ekf.scanner_displacement = logfile.scanner_displacement
        self.ekf.robot_width = logfile.robot_width

        for i in range(len(logfile.motor_ticks)):
            # Odometry
            control = np.array(logfile.motor_ticks[i]) * self.ekf.ticks_to_mm
            self.controls2.append(control)
            prev_state = self.ekf.state
            x_i, x_j, sigma_ij = self.ekf.predict(control)
            #motion = self.ekf.state - prev_state
            #################################################
            #if i==35:breakpoint()
            #################################################
            self.graph_slam.add_motion_constraint(i, x_i, x_j, sigma_ij)
            #self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            if self.debug: self.ekf.print_motion(i)

            # Obervations
            observations = get_observations( logfile.scan_data[i], self.ekf.depth_jump, 
                self.ekf.minimum_valid_distance, self.ekf.cylinder_offset,
                self.ekf.state, self.ekf.scanner_displacement,
                reference_cylinders, self.ekf.max_cylinder_distance)
            if self.test_data: observations = self.observations[i]
            for j in range(len(observations)):
                measurment, landmark = observations[j]
                Q = self.ekf.correct(measurment, landmark)
                self.graph_slam.add_observation_constraint(i+1, measurment, landmark, Q)
                if self.debug: self.ekf.print_observation(i,j)
            self.graph_slam.add_pose(self.ekf.state)

            # Log state, covariance, and matched cylinders for later output.
            self.states.append(self.ekf.state)
            self.covariances.append(self.ekf.covariance)
            self.matched_ref_cylinders.append([m[1] for m in observations])


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
        self.observations = []
        for step in json_obj["observations"]:
            time_step = []
            for obs in step:
                observation = (obs["measure"], obs["landmark"])
                time_step.append(observation)
            self.observations.append(time_step)

        self.controls = []
        for step in json_obj["controls"]:
            time_step = (step["left"], step["right"])
            self.controls.append(time_step)

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
        ax.set_title('Simple Sine Wave Plot')
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
        c = np.array([np.array(control) for control in self.controls2])
        t = np.arange(c.shape[0])

        # Plot data
        fig, ax = plt.subplots() 
        ax.plot(t, c[:,0], label='right')
        ax.plot(t, c[:,1], label='left')
        ax.set_title('Motion Controls')
        ax.set_xlabel('Time Step')
        ax.set_ylabel('Control Ticks')
        ax.legend() # Display the legend based on the 'label' in ax.plot()
        ax.grid(True) # Add a grid to the plot

#######################################################################
    def test_print(self):
        print("***** EKF States ******")
        for idx, s in enumerate(self.states):
            print(f"  x{idx} = {s[0]:.6f} {s[1]:.6f} {s[2]:.6f}")
        print("***** GraphSLAM States ******")
        for idx, s in enumerate(self.graph_slam.poses):
            print(f"  x{idx} = {s[0]:.6f} {s[1]:.6f} {s[2]:.6f}")
##########################################################################



def lego_robot_test(data_dir, debug=False, test_data=False):

    #Initialize EKF
    ekf = EKF()  # dummy values
    ekf.load_json(os.path.join(data_dir,"ekf.json"))

    #Initialize Graph SLAM
    graph_slam = Graph()
    graph_slam.add_pose(ekf.state.copy())
    graph_slam.scanner_displacement = ekf.scanner_displacement
    
    # Read Robot Data
    logfile = LegoLogfile()
    logfile.read(os.path.join(data_dir,"robot4_motors.txt"))
    logfile.read(os.path.join(data_dir,"robot4_scan.txt"))
    logfile.read(os.path.join(data_dir,"robot_arena_landmarks.txt"))

    # Run Main Sytyem
    system = System(ekf, graph_slam, debug, test_data)
    system.load_json(os.path.join(data_dir,"system.json"))
    system.run(logfile)
    system.write_ekf_results(os.path.join(data_dir,"results_ekf.txt"))
    system.write_graph_slam_results(os.path.join(data_dir,"results_graph_slam.txt"))



if __name__ == '__main__':
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    lego_robot_test("Data/LegoRobot", debug=True, test_data=False)
    ##########################################################################

    # import matplotlib.pyplot as plt
    # import numpy as np

    # # Create a Figure and a single Axes object
    # # plt.subplots() returns a tuple: (Figure object, Axes object or array of Axes objects)
    # fig, ax = plt.subplots() 

    # # Generate some sample data
    # x = np.linspace(0, 10, 100)
    # y = np.sin(x)

    # # Plot data on the Axes
    # ax.plot(x, y, label='Sine Wave')

    # # Set properties of the Axes
    # ax.set_title('Simple Sine Wave Plot')
    # ax.set_xlabel('X-axis')
    # ax.set_ylabel('Y-axis')
    # ax.legend() # Display the legend based on the 'label' in ax.plot()
    # ax.grid(True) # Add a grid to the plot

    # # Display the plot
    # plt.show()

    ################################################################################




##############################################
#   DEPRICATED
##############################################
    # config_json = """
    # {
    #     "ticks_to_mm": 0.349,
    #     "cylinder_offset": 90.0,
    #     "depth_jump": 100.0,
    #     "minimum_valid_distance": 20.0,
    #     "max_cylinder_distance": 300.0,
    #     "robot_width": 155.0,
    #     "scanner_displacement": 30.0,
    #     "control_motion_factor": 0.35,
    #     "control_turn_factor": 0.6,
    #     "measurement_distance_stddev": 200.0,
    #     "measurement_angle_stddev": 0.2617993877991494
    # }
    # """

