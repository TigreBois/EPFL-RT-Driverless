import numpy as np
from casadi.casadi import tangent
from do_mpc import controller
from scipy.optimize import minimize_scalar
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

class StanleyPIDController:
    """
        General procedure : At time t_k we specify the state x_k. Then we compute the velocity error v_err_k at time t_k, and the control inputs u_k we want to apply. These will induce a new state x_(k+1) at the next sampling time.
        States : x = [X,Y,phi,v_x]
        Control inputs : u = [a_x, delta]
     """
    # misc
    referencePath: CubicSpline
    v_ref = 5.0  # m/s
    samplingTime = 0.05
    delta_max = np.deg2rad(30.0)

    states: np.ndarray # shape (4,n)
    inputs: np.ndarray # shape (2,n)
    longitudinalErrors: np.ndarray # shape (1,n)

    # PI gains
    K_P = 2.5 # [s^-1]
    K_I = 1   # [s^-2]a
    # Stanley parameters
    k_Delta = 100.0
    k_s = 1.0e-1
    k_d = 1.0e-1

    def currentState(self) -> np.ndarray:
        return self.states[:-1]

    def __init__(self, pathPoints: np.ndarray, initialState: np.ndarray):
        """
            pathPoints : numpy array with 2 columns (one for x coord and one for y coord)
            initialState : numpy array of shape (4,1)
            initialInputs : numpy array of shape (2,1)
         """
        # initialize the reference path
        distance = np.cumsum(
            np.sqrt(np.sum(np.diff(pathPoints, axis=1)**2, axis=0)))
        distance = np.insert(distance, 0, 0)/distance[-1]
        # print("distance=", distance)
        self.referencePath = CubicSpline(distance, pathPoints, axis=1)
        # initialize the states
        self.states = np.zeros((4,1))
        self.states[:,0] = initialState
        self.inputs = np.zeros((2,0))
        # self.inputs[:,0] = initialInputs
        self.longitudinalErrors = np.zeros((1,1))

    def setState(self, newState: np.ndarray):
        assert newState.shape == (4,1)
        self.currentState = np.c_[self.currentState, newState]

    def step(self) -> np.ndarray:
        """
            Computes and returns the new control inputs
         """
        # find new acceleration input
        self.longitudinalErrors = np.c_[self.longitudinalErrors, self.v_ref - self.states[3,-1]]
        new_a_x = self.K_P * self.longitudinalErrors[0,-1] + 0.5 * self.K_I * (self.longitudinalErrors[0,-1] + self.longitudinalErrors[0,-2]) * self.samplingTime
        # find new steering input
        projection = minimize_scalar(lambda theta: (self.referencePath(theta)[0]-self.states[0,-1])**2+(self.referencePath(theta)[1]-self.states[1,-1])**2, bounds=(0.0,1.0), method='Bounded')
        if projection.success:
            tangentVector = np.array(self.referencePath(projection.x, 1))
            e_phi = np.arccos((tangentVector @ np.array([np.cos(self.states[2,-1]), np.sin(self.states[2,-1])])) / np.sqrt(tangentVector @ tangentVector))
            new_delta = e_phi + np.arctan(self.k_Delta*projection.fun/(self.k_s+self.k_d*self.states[3,-1]))
        else:
            new_delta = self.inputs[1,-1]
        
        np.clip(new_delta, -self.delta_max, self.delta_max)
        self.inputs = np.c_[self.inputs, np.array([new_delta, new_a_x])]
        return self.inputs[:,-1]

    def carDynamics(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : MPC input = [ddelta, dD]
            returns : xdot
        """
        assert x.size == 6
        assert u.size == 2
        assert w.size == 2
        m = 230.0
        I_z = 20.0
        l_R = 1.0
        l_F = 1.0
        B_R = 10.0
        C_R = 1.9
        D_R = 1.0
        B_F = 10.0
        C_F = 1.9
        D_F = 1.0
        # C_m = 200.0
        # C_r0 = 0.0
        # C_r2 = 0.0
        P_TV = 0.0
        v_blend_min = 2.0
        v_blend_max = 3.0
        

        # F_x = C_m*u[1]-C_r0-C_r2*(x[3]**2)
        F_F_y = D_F * np.sin(C_F*np.arctan(B_F * np.arctan((x[4] + l_F*x[5]) / x[3])-u[0])) if x[3] != 0 else 0.0
        F_R_y = D_R * np.sin(C_R*np.arctan(B_R * np.arctan((x[4] - l_R*x[5]) / x[3]))) if x[3] != 0 else 0.0
        blendCoef = min(max((x[3]-v_blend_min) / (v_blend_max-v_blend_min), 0), 1)
        return np.array([x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]),
                        x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]),
                        x[5],
                        u[1] - blendCoef*F_F_y * np.sin(u[0]) / m + x[4] * x[5] * blendCoef,
                        blendCoef/m*(F_R_y+F_F_y*np.cos(u[0])-m*x[3]*x[5]) + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])*l_R/(l_R+l_F),
                        blendCoef/I_z*(F_F_y*l_F*np.cos(u[0])-F_R_y*l_R+P_TV*(u[0]*x[3]/(l_F+l_R) - x[5])) + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])/(l_R+l_F),
                        ])

    def computeNextState(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        k1 = self.carDynamics(x, u, w)
        k2 = self.carDynamics(x + self.samplingTime*0.5*k1, u, w)
        k3 = self.carDynamics(x + self.samplingTime*0.5*k2, u, w)
        k4 = self.carDynamics(x + self.samplingTime*k3, u, w)
        return x+self.samplingTime/6*(k1+2*k2+2*k3+k4)

    def evolve(self):
        if self.inputs.size > 2:
            self.states = np.c_[self.states, self.computeNextState(x=self.states[:,-1], u=self.inputs[:,-1], w=self.samplingTime*(self.inputs[:,-1]-self.inputs[:,-2]))]
        else:
            self.states = np.c_[self.states, self.computeNextState(x=self.states[:,-1], u=self.inputs[:,-1], w=np.zeros(2))]




if __name__ == '__main__':
    pathPoints = np.c_[np.linspace(0,250,100), np.zeros(100)].T
    # print("pathPoints=", pathPoints)
    assert pathPoints.shape == (2,100)
    currentState = np.array([0,0,0,0,0,0])
    C = StanleyPIDController(pathPoints=pathPoints, initialState=currentState[:4])

    simulationLength = 450
    for k in range(simulationLength):
        newInputs = C.step()
        if C.inputs.size > 2:
            w=C.samplingTime*(C.inputs[:,-1]-C.inputs[:,-2])
        else:
            w=np.zeros(2)
        currentState = C.computeNextState(x=currentState, u=C.inputs[:,-1], w=w)
        C.states = np.c_[C.states, currentState[:4]]

    statesFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/states.csv', 'w')
    for i in range(simulationLength+1):
        statesFile.write('{},{},{},{},{}\n'.format(i, C.states[0,i], C.states[1,i], C.states[2,i], C.states[3,i]))
    statesFile.close()
    inputsFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/inputs.csv', 'w')
    for i in range(simulationLength):
        inputsFile.write('{},{},{}\n'.format(i, C.inputs[0,i], C.inputs[1,i]))
    inputsFile.close()
    # print("C.states", C.states.T)
    # print("C.inputs", C.inputs.T)
    plt.plot(C.states[0,:], C.states[1,:], 'b+')
    plt.plot(pathPoints[0,:], pathPoints[1, :], 'g-')
    plt.show()
    print('yes')