import numpy as np
import math as mt
from scipy.optimize import minimize
from scipy.optimize import Bounds
import datetime
import random

class MPC_car_model:
    
    # PARAMETERS FOR THE MPC MODEL
    HORIZON_N = 10
    break_max = -3
    acceleration_max = 9
    steering_angle_max = mt.pi/9
    path_idx = 0

    # WEIGHT OF ERRORS
    q_MSE = 0.7
    q_dRs = 0.3
    q_Rs = 0.0 # WE DO NOT CARE OF THE VALUE ITSELF BUT OF THE DERIVATIVE FOR THE SMOOTHNESS OF THE DRIVE
    
    # ERROR REFERENCE 
    max_err_Y = 2 ** 2
    max_err_X = 2 ** 2
    max_incr_D = 0.6
    max_incr_steer = mt.pi/4

    #MATRIX ERROR (FOR U AND dU VECTORS)
    R_dU = np.array([[0.5 / (max_incr_D * max_incr_D), 0.0],
                    [0.0, 0.5 / (max_incr_steer * max_incr_steer)]])

    R_U = np.array([[0.0, 0],
                    [0, 0.0]])

    # COEFFICIENT OF KINETIC MODEL (TRAIN)
    C_m = 200
    C_r0 = 0
    C_r2 = 0

    # CAR PARAMETERS (MASS, WHEEL DISTANCE BETWEEN REAR/FRONT AND CENTER OF CAR)
    # THOSE PARAMETERS SHOULD BE CALCULATED WITH IRL TESTS
    m = 223
    l_R = 1
    l_F = 1
    #v_y_factor = l_R/(l_R + l_F)
    #r_factor = 1/(l_R + l_F)
    v_y_factor =  1/2
    r_factor = 1/4

    delta_time = 1

    # x = [X, Y, Phi, V_x, V_y, r] with (X, Y) the car's position, Phi the position and heading in global coordinates,
    # V_x and V_y hor/vert velocity and r the yaw rate r
    # u = [D, steering] with delta the steering angle and D the driving command

    guess_D = [0.0 for i in range(HORIZON_N)]
    guess_steering = [mt.pi/3 for i in range(HORIZON_N)]
    first_guess = guess_D + guess_steering

    previous_states = np.array(first_guess)
    path = np.empty(HORIZON_N)

    """Initialization of the module : set up bounds
    """

    def __init__(self):
        self.delta_time = 0.5

        bnds_a = ((self.break_max, self.acceleration_max),) * (self.HORIZON_N)
        bnds_steer = ((-self.steering_angle_max, self.steering_angle_max), ) * (self.HORIZON_N)
        bounds = bnds_a + bnds_steer
        self.bounds = bounds

    def F_x(self, D, v_x):
        """Calculates the longitudinal force F_x with the motor model, C_m * D, rolling resistance, C_r and drag C_r2 * v_x
    
        Args : 
            D : driving command
            v_x : longitudinal velocity

        Returns :
            double : longitudinal force
        """
        return self.C_m * D - self.C_r0 - self.C_r2 * (v_x ** 2)



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

    def f_next_state(self, x_k, u_k, dU_k):
        """Computes the next state of the car given the chosen model

        Args:
            x_k (list): current state of the car 
            u (list): control inputs of the car
            dU (list): derivative of the control input

        Returns:
            list: next state of the car
        """
        first = x_k[0] + self.delta_time * (x_k[3] * mt.cos(x_k[2]) - x_k[4] * mt.sin(x_k[2]))
        second = x_k[1] + self.delta_time * (x_k[3] * mt.sin(x_k[2]) + x_k[4] * mt.cos(x_k[2]))
        third = x_k[2] + self.delta_time * x_k[5]
        fourth = x_k[3] + self.delta_time * self.F_x(u_k[0], x_k[3]) / self.m
        fifth = x_k[4] + self.delta_time * ((dU_k[1] * x_k[3] + u_k[1] * fourth) * self.v_y_factor)
        sixth = x_k[5] + self.delta_time * ((dU_k[1] * x_k[3] + u_k[1] * fourth) * self.r_factor)
        to_ret = np.array([first, second, third, fourth, fifth, sixth])
        return to_ret

    def f_kinematic(self, x_k, u_k, dU_k):
        first = x_k[3] * mt.cos(x_k[2]) - x_k[4] * mt.sin(x_k[2])
        second = x_k[3] * mt.sin(x_k[2]) + x_k[4] * mt.cos(x_k[2])
        third = x_k[5]
        fourth = self.F_x(u_k[0], x_k[3]) / self.m
        fifth = (dU_k[1] * x_k[3] + u_k[1] * fourth) * self.v_y_factor
        sixth = (dU_k[1] * x_k[3] + u_k[1] * fourth) * self.r_factor
        to_ret = np.array([first, second, third, fourth, fifth, sixth])
        return to_ret

    def calculate_inertia(self):
        return 200

    #force parameters
    D_r = 0
    C_r = 0
    B_r = 0
    D_f = 0
    C_f = 0
    B_f = 0
    P_tv = 0
    v_x_min = 3
    v_x_max = 5

    def calculate_alpha_r(self, x_k):
        return mt.atan((x_k[4] - self.l_R * x_k[5])/x_k[3])

    def calculate_alpha_f(self, x_k, u_k):
        return mt.atan((x_k[4] + self.l_F * x_k[5])/x_k[3]) - u_k[1]

    def calculate_F_R_y(self, alpha_r):
        return self.D_r * mt.sin(self.C_r * mt.atan(self.B_r * alpha_r))

    
    def calculate_F_f_y(self, alpha_f):
        return self.D_f * mt.sin(self.C_f * mt.atan(self.B_f * alpha_f))

    def calculate_t(self, x_k, u_k):
        target = u_k[1] * x_k[3] / (self.l_F + self.l_R)
        return (target - x_k[5]) * self.P_tv
    
    def f_dynamic(self, x_k, u_k, dU_k):
        first = x_k[3] * mt.cos(x_k[2]) - x_k[4] * mt.sin(x_k[2])
        print(first)

        second = x_k[3] * mt.sin(x_k[2]) + x_k[4] * mt.cos(x_k[2])
        print(second)

        third = x_k[5]
        print(third)

        fourth = 1/self.m * (self.f_kinematic(x_k, u_k, dU_k) - self.calculate_F_f_y(self.calculate_alpha_f(x_k, u_k)) * mt.sin(u_k[1]) + self.m * x_k[4] * x_k[5])
        print(fourth)

        fifth = 1/self.m * (self.calculate_F_R_y(self.calculate_alpha_r(x_k)) - self.calculate_F_f_y(self.calculate_alpha_f(x_k, u_k)) * mt.cos(u_k[1]) - self.m * x_k[4] * x_k[5])
        print(fifth)

        sixth = 1/self.calculate_inertia() * (self.calculate_F_f_y(self.calculate_alpha_f(x_k, u_k)) * self.l_F * mt.cos(u_k[1]) - self.calculate_F_R_y(self.calculate_alpha_r(x_k)) * self.l_R + self.calculate_t(x_k, u_k))
        print(sixth)
        to_ret = np.array([first, second, third, fourth, fifth, sixth])
        return to_ret

    def calculate_weight(self, x_k):
        return min(0,max(1, (x_k[3] - self.v_x_min)/ (self.v_x_min - self.v_x_max))) 

    def next_state(self, x_k, u_k, dU_k):
        param = self.calculate_weight(x_k)
        f_kin = self.f_kinematic(x_k, u_k, dU_k)
        f_dyn = self.f_dynamic(x_k, u_k, dU_k)
        delta = np.add(param * f_dyn, (1-param) * f_kin)
        
        delta = self.delta_time * delta
        first = x_k[0] + self.delta_time * delta[0]
        second = x_k[1] + self.delta_time * delta[0]
        third = x_k[2] + self.delta_time * delta[0]
        fourth = x_k[3] + self.delta_time * delta[0]
        fifth = x_k[4] + self.delta_time * delta[0]
        sixth = x_k[5] + self.delta_time * delta[0]
        to_ret = np.array([first, second, third, fourth, fifth, sixth])
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
        new_state = np.ones((self.HORIZON_N - 1, 6))
        new_state[0] = self.x0
        for i in range(1, self.HORIZON_N - 1, 1):
            new_state[i] = self.f_next_state(new_state[i - 1], u_k[i], dU[i])

        self.new_state = new_state

        # given the computed states, we compute the error between the wanted position of the car (path) and those states
        self.error = 0
        for i in range(self.HORIZON_N - 1):
           self.error += self.MSE(new_state[i], self.path[i]) + self.error_R(u_k[i], dU[i])
        print(self.error)
        return self.error


    def run_MPC(self):
        """Function that run the MPC to get the next control inputs

        Returns:
            (tuple): next control inputs
        """
        result = minimize(self.big_fun, self.previous_states, bounds=self.bounds, method='SLSQP')
        self.previous_states = np.array(result.x).reshape((self.HORIZON_N,2))
        to_return = np.array([result.x[0], result.x[self.HORIZON_N]])
        self.previous_good_state = to_return

        reshaped_state = [None] * self.HORIZON_N
        for i in range(self.HORIZON_N):
            reshaped_state[i] = (result.x[i], result.x[self.HORIZON_N + i])

        return to_return, reshaped_state

    def acquire_path(self, path):
        """Setter for the path

        Args:
            path (list): list of 2D points
        """
        self.path = path
        self.last_closest_point = 0

    def set_state(self, x0):
        """Setter for the current state of the car

        Args:
            x0 (list): new state of the car
        """
        self.x0 = x0

    def shift_path(self):
        """When the MPC is run once, we shift the path assuming that the car will be reach next point of the path
        """

        closest_idx = 0
        distance = self.calc_distance(self.path[0], self.x0)
        for i in range(self.last_closest_point, self.HORIZON_N):
            point = self.path[i]
            current = self.calc_distance(point, self.x0)
            if(current < distance):
                distance = current
                closest_idx = i
        
        self.last_closest_point = closest_idx 
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

    def local_to_global(self, path, state):
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




