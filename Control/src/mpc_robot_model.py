import numpy as np
import math as mt
from scipy.optimize import minimize
from scipy.optimize import Bounds
import datetime
import random

class MPC_Robot_model:
    
    # PARAMETERS FOR THE MPC MODEL
    HORIZON_N = 10
    #values corresponds to the specifications
    break_max = -3
    acceleration_max = 3
    #because the robot rotate itself we fix the maximum rotation to PI which correspond to doing a U-turn
    steering_angle_max = mt.pi
    path_idx = 0

    # WEIGHT OF ERRORS
    q_MSE = 0.7
    q_dRs = 0.3
    q_Rs = 0.0 # WE DO NOT CARE OF THE VALUE ITSELF BUT OF THE DERIVATIVE FOR THE SMOOTHNESS OF THE DRIVE

    #ERROR REFERENCE TO SCALE ERRORS
    max_err_Y = 1 **2
    max_err_X = 1 **2
    max_incr_D = 0.5
    max_incr_steer = mt.pi

    #ERROR MATRIX FOR U AND DU VECTORS
    R_dU = np.array([[0.5 / (max_incr_D * max_incr_D), 0.0],
                    [0.0, 0.5 / (max_incr_steer * max_incr_steer)]])

    R_U = np.array([[0.0, 0],
                    [0, 0.0]])

    delta_time = 0.2

    
    # to start optimization the first, the optimizer needs a "guess" solution which is a null vel
    guess_velocity = [0.0 for i in range(HORIZON_N)]
    guess_angular = [mt.pi/2 for i in range(HORIZON_N)]
    first_guess = guess_velocity + guess_angular
    previous_states = np.array(first_guess)

    
    def __init__(self):
        """Initialization of the module : set up bounds
        """

        self.delta_time = 0.5

        bnds_a = ((self.break_max, self.acceleration_max),) * (self.HORIZON_N)
        bnds_steer = ((-self.steering_angle_max, self.steering_angle_max), ) * (self.HORIZON_N)
        bounds = bnds_a + bnds_steer
        self.bounds = bounds

        print(bounds) 

    def MSE(self, x_k, x_k_ref):
        """Computes mean square error between the given point and the reference point

        Args:
            x_k (pair): "real" point
            x_k_ref (pair): reference point

        Returns:
            double : mean square error
        """

        dX = (x_k[0] - x_k_ref[0]) * (x_k[0] - x_k_ref[0]) / self.max_err_X
        dY = (x_k[1] - x_k_ref[1]) * (x_k[1] - x_k_ref[1]) / self.max_err_Y
        return self.q_MSE * (dX + dY) / 2

    def error_R(self, u, dU):
        """Computes the control input penalization

        Args:
            u (list): control inputs of the car
            dU (list): derivative of the control input

        Returns:
            double: cost of vectors
        """
        return self.q_Rs * np.matmul(np.matmul(u.T, self.R_U), u) + self.q_dRs * np.matmul(np.matmul(dU.T, self.R_dU), dU)

    def f_next_state(self, x_k, u_k):
        """Computes the next state of the car gvien the chosen two-wheel robot model

        Args:
            x_k (list): current state of the car 
            u (list): control inputs of the car
            dU (list): derivative of the control input

        Returns:
            list: next state of the car
        """

        first = x_k[0] + self.delta_time * u_k[0] * mt.cos(x_k[2])
        second = x_k[1] + self.delta_time * u_k[0] * mt.sin(x_k[2])
        third = x_k[2] + self.delta_time * u_k[1]
        to_ret = np.array([first, second, third])
        return to_ret


    def big_fun(self, u_ks: np.ndarray):
        """Function to optimize : given the control inputs, computes the next 'HORIZON_N' states of the car
            and computes the error corresponding to those states

        Args:
            u_ks (np.ndarray): control inputs of the car

        Returns:
            double: associated error
        """

        #reshaping u_ks array
        u_D = u_ks[:self.HORIZON_N]
        u_steering = u_ks[self.HORIZON_N:]
        u_k = np.zeros((self.HORIZON_N,2))
        u_k[:,0] = u_D
        u_k[:,1] = u_steering      

        ## we initialize random dU-vector 
        dU = u_k[1:] - u_k[:-1]
        
        # we calculate the next HORIZON_N states of the car according to those Us
        new_state = np.ones((self.HORIZON_N - 1, 3))
        new_state[0] = self.x0
        for i in range(1, self.HORIZON_N - 1, 1):
            new_state[i] = self.f_next_state(new_state[i - 1], u_k[i])

        self.new_state = new_state

        # given the computed states, we compute the error between the wanted position of the car (path) and those states
        error = 0
        for i in range(self.HORIZON_N - 1):
           error += self.MSE(new_state[i], self.path[i]) + self.error_R(u_k[i], dU[i])
        self.last_error = error
        return error


    def run_MPC(self):
        """Function that run the MPC to get the next control inputs

        Returns:
            (tuple): next control inputs
        """

        result = minimize(self.big_fun, self.previous_states, bounds=self.bounds, method='SLSQP')
        self.previous_states = np.array(result.x).reshape((self.HORIZON_N,2))
        to_return = np.array([result.x[0], result.x[self.HORIZON_N]])
        self.previous_good_state = to_return

        print("Last point error was :", self.last_error)
        print("Next point to go :", self.path[0])


        return to_return

    def acquire_path(self, path):
        """Setter for the path

        Args:
            path (list): list of 2D points
        """

        self.path = path
        self.path_idx = 0

    def set_state(self, x0):
        """Setter for the current state of the car

        Args:
            x0 (list): new state of the car
        """

        self.x0 = x0


    def shift_path(self):
        """When the MPC is run once, we shift the path assuming that the car will be reach a new point of the path
        """
    
        closest_idx = 0
        distance = self.calc_distance(self.path[0], self.x0)
        for i in range(len(self.path)):
            point = self.path[i]
            current = self.calc_distance(point, self.x0)
            if(current < distance):
                distance = current
                closest_idx = i
        
        last = self.path[len(self.path) - 1]
        for i in range(len(self.path) - 1):
            
            if i + closest_idx > len(self.path) - 1:
                self.path[i] = last
            else: 
                self.path[i] = self.path[i + closest_idx]
                
        self.path[len(self.path) - 1] = last
        
    def calc_distance(self, next, curr):
        """Computes the distance between two points

        Args:
            next (pair): end point of vector
            curr (pair): starting point of vector

        Returns:
            double: the distance between the given points
        """
        diffX = (next[0] - curr[0]) * (next[0] - curr[0])
        diffY = (next[1] - curr[1]) * (next[1] - curr[1])
        return mt.sqrt(diffX + diffY)


    def local_to_local(self, path, state):
        """Convert a path in local coordiants to global coordinates
            Here, I do not use self.path nor self.state 
            because I convert the path and set the state 
            after converting the path in the right system of coordinates

        Args:
            path (list of pair): path to convert
            state ([type]): state of the car

        Returns:
            list of pair: path in global coordinates

        """
        new_path = None * len(path)
        for i in range(len(path)):
            current = path[i]
            x_glob = current[0] * mt.cos(state[2]) - current[1] * mt.sin(state[2]) + state[0]
            y_glob = current[0] * mt.sin(state[2]) + current[1] * mt.sin(state[2]) + state[1]
            new_path[i] = (x_glob, y_glob)

        return new_path
