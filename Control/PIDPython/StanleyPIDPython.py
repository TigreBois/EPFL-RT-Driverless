import numpy as np
from casadi.casadi import tangent
from do_mpc import controller
# from scipy.optimize import fmin
import scipy.optimize as opt
from scipy.interpolate import CubicSpline
import scipy.spatial.distance as dist
import matplotlib.pyplot as plt
import sys



class StanleyPIDController:
    """
        General procedure : At time t_k we specify the state x_k. Then we compute the velocity error v_err_k at time t_k, and the control inputs u_k we want to apply. These will induce a new state x_(k+1) at the next sampling time.
        States : x = [X,Y,phi,v_x]
        Control inputs : u = [a_x, delta]
     """
    # misc
    referencePath: CubicSpline
    v_ref = 3.0  # m/s
    samplingTime = 0.05
    delta_max = np.deg2rad(30.0)

    states: np.ndarray # shape (4,n)
    inputs: np.ndarray # shape (2,n)
    longitudinalErrors: np.ndarray # shape (1,n)
    projections: np.ndarray

    # PI gains
    K_P = 3 # [s^-1]
    K_I = 1   # [s^-2]a
    # Stanley parameters
    k_Delta = 1.0e0
    k_s = 1.0
    k_d = 1.0e1

    # physical parameters of the car model
    m = 223.0
    l_R = 0.895
    l_F = 0.675
    I_z = 360.0
    B_R = 10.0
    C_R = 1.9
    D_R = 1.0
    B_F = 10.0
    C_F = 1.9
    D_F = 1.0
    # C_m = 200.0
    # C_r0 = 0.0
    # C_r2 = 0.0
    v_blend_min = 1.0
    v_blend_max = 2.0

    def currentState(self) -> np.ndarray:
        return self.states[:-1]

    def __init__(self, pathPoints: np.ndarray, initialState: np.ndarray):
        """
            pathPoints : numpy array with 2 rows (one for x coord and one for y coord)
            initialState : numpy array of shape (4,1)
            initialInputs : numpy array of shape (2,1)
         """
        # initialize the reference path
        distance = np.cumsum(
            np.sqrt(np.sum(np.diff(pathPoints, axis=1)**2, axis=0)))
        distance = np.insert(distance, 0, 0)/distance[-1]
        self.referencePath = CubicSpline(distance, pathPoints, axis=1, extrapolate=True)
        splinePointsFile = open("/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/splinePoints.csv", "w")
        for theta in distance:
            splinePointsFile.write(str(theta) + "," + str(self.referencePath(theta)) + "\n")

        splinePointsFile.close()
        # plt.figure(num=2)
        # plt.plot(pathPoints[0,:], pathPoints[1,:], 'r+')
        # plt.plot(self.referencePath(np.linspace(0,1,1000))[0], self.referencePath(np.linspace(0,1,1000))[1], 'b-')
        # plt.show()

        # initialize the states
        self.states = np.zeros((4,1))
        self.states[:,0] = initialState
        self.inputs = np.zeros((2,0))
        self.longitudinalErrors = np.zeros((1,1))
        self.projections = np.zeros((1,0))

        # open documents
        self.pidTermsFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/pidTerms.csv', 'w')
        self.longitudinalErrorsFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/longitudinalErrors.csv', 'w')
        self.steeringErrorsFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/steeringErrors.csv', 'w')

    def __del__(self):
        self.pidTermsFile.close()
        self.longitudinalErrorsFile.close()
        self.steeringErrorsFile.close()

    def setState(self, newState: np.ndarray):
        assert newState.shape == (4,1) or newState.shape == (4,)
        self.currentState = np.column_stack((self.currentState, newState))


    def step(self, k: int) -> np.ndarray:
        """
            Computes and returns the new control inputs
         """
        # find new acceleration input
        actual_v_ref = self.v_ref #if self.states[0,-1] <= 80 else 0
        self.longitudinalErrors = np.column_stack((self.longitudinalErrors, actual_v_ref - self.states[3,-1]))
        self.longitudinalErrorsFile.write('{},{}\n'.format(k, self.longitudinalErrors[0,-1]))
        new_a_x = self.K_P * self.longitudinalErrors[0,-1] + 0.5 * self.K_I * (self.longitudinalErrors[0,-1] + self.longitudinalErrors[0,-2]) * self.samplingTime
        self.pidTermsFile.write('{},{},{}\n'.format(k, self.K_P * self.longitudinalErrors[0,-1],0.5 * self.K_I * (self.longitudinalErrors[0,-1] + self.longitudinalErrors[0,-2]) * self.samplingTime))

        # find new steering input ============================================================
        
        # METHOD 1 : fmin ============================================================
        # def fun(theta):
        #     return dist.euclidean(self.referencePath(theta), self.states[0:2,-1])

        # thetaMin = opt.fminbound(fun, 0.0, 1.0)
        # distMin = dist.euclidean(self.referencePath(thetaMin), self.states[:1,-1])
        # # heading error
        # tangentVector = np.array(self.referencePath(thetaMin, 1))
        # headingVector = np.array([np.cos(self.states[2,-1]), np.sin(self.states[2,-1])])
        # tangentVectorAngle = np.arctan2(tangentVector[1], tangentVector[0])
        # headingVectorAngle = self.states[2,-1]
        # # headingError = e_phi in the stanley report
        # if tangentVectorAngle - headingVectorAngle > np.pi:
        #     headingError = tangentVectorAngle - headingVectorAngle-2*np.pi
        # elif tangentVectorAngle - headingVectorAngle < -np.pi:
        #     headingError = tangentVectorAngle - headingVectorAngle+2*np.pi
        # else:
        #     headingError = tangentVectorAngle - headingVectorAngle
        
        # # cross track error
        # e_delta = np.array([self.referencePath(thetaMin)[0] - self.states[0,-1], self.referencePath(thetaMin)[1] - self.states[1,-1]])
        # crossTrackError = np.arctan(self.k_Delta*distMin/(self.k_s+self.k_d*self.states[3,-1]))
        # e_deltaAngle = np.arctan2(e_delta[1], e_delta[0])
        # difference2 = e_deltaAngle - headingVectorAngle
        # if np.abs(difference2) > np.pi:
        #     sign2 = -np.sign(difference2)
        # else:
        #     sign2 = np.sign(difference2)
        
        # if sign2 > 0:
        #     crossTrackError = np.abs(crossTrackError)
        # else:
        #     crossTrackError = -np.abs(crossTrackError)

        # new_delta = headingError + crossTrackError
        # self.steeringErrorsFile.write('{}, projection index : {}, projection point : {}, projection distance : {}, heading error : {}, cross track error : {}, new_delta : {}\n'.format(k, thetaMin, self.referencePath(thetaMin), distMin, headingError, crossTrackError, new_delta))

        
        # METHOD 2 : minimize_scalar ============================================================
        projection = opt.minimize_scalar(lambda theta: dist.euclidean(self.states[0:2,-1], self.referencePath(theta)), bounds=(0.0,1.0), method='Bounded')
        # projection = opt.minimize_scalar(lambda theta: dist.euclidean(self.states[0:2,-1], self.referencePath(theta)), method='Brent', bracket=(0.0,self.projections[-1] if self.projections.size > 0 else 0.01,1.0))
        # self.projections = np.column_stack((self.projections, projection.x))
        if projection.success:
            # heading error
            tangentVector = np.array(self.referencePath(projection.x, 1))
            headingVector = np.array([np.cos(self.states[2,-1]), np.sin(self.states[2,-1])])
            tangentVectorAngle = np.arctan2(tangentVector[1], tangentVector[0])
            headingVectorAngle = self.states[2,-1]
            # headingError = e_phi in the stanley report
            if tangentVectorAngle - headingVectorAngle > np.pi:
                headingError = tangentVectorAngle - headingVectorAngle-2*np.pi
            elif tangentVectorAngle - headingVectorAngle < -np.pi:
                headingError = tangentVectorAngle - headingVectorAngle+2*np.pi
            else:
                headingError = tangentVectorAngle - headingVectorAngle
            
            # cross track error
            e_delta = np.array([self.referencePath(projection.x)[0] - self.states[0,-1], self.referencePath(projection.x)[1] - self.states[1,-1]])
            crossTrackError = np.arctan(self.k_Delta*projection.fun/(self.k_s+self.k_d*self.states[3,-1]))
            e_deltaAngle = np.arctan2(e_delta[1], e_delta[0])
            difference2 = e_deltaAngle - headingVectorAngle
            if np.abs(difference2) > np.pi:
                sign2 = -np.sign(difference2)
            else:
                sign2 = np.sign(difference2)
            
            if sign2 > 0:
                crossTrackError = np.abs(crossTrackError)
            else:
                crossTrackError = -np.abs(crossTrackError)

            new_delta = headingError + crossTrackError
            
            # print('{}, heading error : {}, cross track error : {}, new_delta : {}'.format(k,headingError, crossTrackError, new_delta))
        else:
            sys.stderr.write('failure of projection\n')
            new_delta = self.inputs[0,-1]

        self.steeringErrorsFile.write('{}, projection index : {}, projection point : {}, projection distance : {}, heading error : {}, cross track error : {}, new_delta : {}\n'.format(k, projection.x, self.referencePath(projection.x), projection.fun, headingError, crossTrackError, new_delta))

        # METHOD 3 : custom made newton raphson ============================================================
        # thetaProj = self.projections[-1] if self.projections.size > 0 else 0.0
        # distancePrime = -2*(self.referencePath(thetaProj, 1)[0]*(self.states[0,-1]-self.referencePath(thetaProj)[0]) + self.referencePath(thetaProj, 1)[1]*(self.states[1,-1]-self.referencePath(thetaProj)[1]))
        # distanceSecond = -2*(self.referencePath(thetaProj,2)[0]*(self.states[0,-1]-self.referencePath(thetaProj)[0]) + self.referencePath(thetaProj,1)[0]*(self.states[0,-1]-self.referencePath(thetaProj, 1)[0]) + self.referencePath(thetaProj,2)[1]*(self.states[1,-1]-self.referencePath(thetaProj)[1]) + self.referencePath(thetaProj,1)[1]*(self.states[1,-1]-self.referencePath(thetaProj, 1)[1]))
        # nbrIterations = 0
        # while np.abs(distancePrime) > 1e-6 and nbrIterations < 100:
        #     thetaProj = thetaProj - distancePrime / distanceSecond
        #     thetaProj = np.clip(thetaProj, 0.0, 1.0)
        #     distancePrime = -2*(self.referencePath(thetaProj, 1)[0]*(self.states[0,-1]-self.referencePath(thetaProj)[0]) + self.referencePath(thetaProj, 1)[1]*(self.states[1,-1]-self.referencePath(thetaProj)[1]))
        #     distanceSecond = -2*(self.referencePath(thetaProj,2)[0]*(self.states[0,-1]-self.referencePath(thetaProj)[0]) + self.referencePath(thetaProj,1)[0]*(self.states[0,-1]-self.referencePath(thetaProj, 1)[0]) + self.referencePath(thetaProj,2)[1]*(self.states[1,-1]-self.referencePath(thetaProj)[1]) + self.referencePath(thetaProj,1)[1]*(self.states[1,-1]-self.referencePath(thetaProj, 1)[1]))
        #     nbrIterations += 1
            
        # self.projections = np.append(self.projections, thetaProj)
        # distance = dist.euclidean(self.states[0:2,-1], self.referencePath(thetaProj))
        # # heading error
        # tangentVector = np.array(self.referencePath(thetaProj, 1))
        # headingVector = np.array([np.cos(self.states[2,-1]), np.sin(self.states[2,-1])])
        # tangentVectorAngle = np.arctan2(tangentVector[1], tangentVector[0])
        # headingVectorAngle = self.states[2,-1]
        # # headingError = e_phi in the stanley report
        # if tangentVectorAngle - headingVectorAngle > np.pi:
        #     headingError = tangentVectorAngle - headingVectorAngle-2*np.pi
        # elif tangentVectorAngle - headingVectorAngle < -np.pi:
        #     headingError = tangentVectorAngle - headingVectorAngle+2*np.pi
        # else:
        #     headingError = tangentVectorAngle - headingVectorAngle
        
        # # cross track error
        # e_delta = np.array([self.referencePath(thetaProj)[0] - self.states[0,-1], self.referencePath(thetaProj)[1] - self.states[1,-1]])
        # crossTrackError = np.arctan(self.k_Delta*distance/(self.k_s+self.k_d*self.states[3,-1]))
        # e_deltaAngle = np.arctan2(e_delta[1], e_delta[0])
        # difference2 = e_deltaAngle - headingVectorAngle
        # if np.abs(difference2) > np.pi:
        #     sign2 = -np.sign(difference2)
        # else:
        #     sign2 = np.sign(difference2)
        
        # if sign2 > 0:
        #     crossTrackError = np.abs(crossTrackError)
        # else:
        #     crossTrackError = -np.abs(crossTrackError)
        # new_delta = headingError + crossTrackError
        # self.steeringErrorsFile.write('{}, projection index : {}, projection point : {}, projection distance : {}, heading error : {}, cross track error : {}, new_delta : {}\n'.format(k, thetaProj, self.referencePath(thetaProj), distance, headingError, crossTrackError, new_delta))

        new_delta = np.clip(new_delta, -self.delta_max, self.delta_max)
        self.inputs = np.column_stack((self.inputs, np.array([new_delta, new_a_x])))
        return self.inputs[:,-1]

    def carDynamics(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : MPC input = [ddelta, dD]
            returns : xdot
        """
        assert x.size == 6 and (x.shape == (6,1) or x.shape == (6,))
        assert u.size == 2 and (u.shape == (2,1) or u.shape == (2,))
        assert w.size == 2 and (w.shape == (2,1) or w.shape == (2,))
        
        # F_x = C_m*u[1]-C_r0-C_r2*(x[3]**2)
        F_F_y = self.D_F * np.sin(self.C_F*np.arctan(self.B_F * np.arctan((x[4] + self.l_F*x[5]) / x[3])-u[0])) if x[3] != 0 else 0.0
        F_R_y = self.D_R * np.sin(self.C_R*np.arctan(self.B_R * np.arctan((x[4] - self.l_R*x[5]) / x[3]))) if x[3] != 0 else 0.0
        blendCoef = 1.0 # min(max((x[3]-self.v_blend_min) / (self.v_blend_max-self.v_blend_min), 0), 1)
        return np.array([x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]),
                        x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]),
                        x[5],
                        u[1] - blendCoef*F_F_y * np.sin(u[0]) / self.m + x[4] * x[5] * blendCoef,
                        blendCoef*(F_R_y+F_F_y*np.cos(u[0])-self.m*x[3]*x[5])/self.m + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])*self.l_R/(self.l_R+self.l_F),
                        blendCoef*(np.sin(u[0])*self.l_F*np.cos(u[0])-F_R_y*self.l_R)/self.I_z + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])/(self.l_R+self.l_F)
                        # blendCoef*(F_F_y*self.l_F*np.cos(u[0])-F_R_y*self.l_R)/self.I_z + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])/(self.l_R+self.l_F)
                        ])
 
    def computeNextState(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        k1 = self.carDynamics(x, u, w)
        k2 = self.carDynamics(x + self.samplingTime*0.5*k1, u, w)
        k3 = self.carDynamics(x + self.samplingTime*0.5*k2, u, w)
        k4 = self.carDynamics(x + self.samplingTime*k3, u, w)
        return x+self.samplingTime/6*(k1+2*k2+2*k3+k4)
        # return x+self.samplingTime*self.carDynamics(x,u,w)

    def computeNextState2(self, x: np.ndarray, u: np.ndarray, w:np.ndarray) -> np.ndarray:
        first = x[0] + self.samplingTime * (x[3] * np.cos(x[2]) - x[4] * np.sin(x[2]))
        second = x[1] + self.samplingTime * (x[3] * np.sin(x[2]) + x[4] * np.cos(x[2]))
        third = x[2] + self.samplingTime * x[5]
        fourth = x[3] + self.samplingTime * u[1]
        fifth = x[4] + self.samplingTime * ((w[0] * x[3] + u[0] * fourth) * self.l_R / (self.l_R+self.l_F))
        sixth = x[5] + self.samplingTime * ((w[0] * x[3] + u[0] * fourth) / (self.l_R+self.l_F))
        # fifth = x[4] + self.samplingTime * ((w[0] * x[3] + u[0] * u[1]) * l_R / (l_R+l_F))
        # sixth = x[5] + self.samplingTime * ((w[0] * x[3] + u[0] * u[1]) / (l_R+l_F))
        return np.array([first, second, third, fourth, fifth, sixth])

    def evolve(self):
        if self.inputs.size > 2:
            self.states = np.c_[self.states, self.computeNextState(x=self.states[:,-1], u=self.inputs[:,-1], w=self.samplingTime*(self.inputs[:,-1]-self.inputs[:,-2]))]
        else:
            self.states = np.c_[self.states, self.computeNextState(x=self.states[:,-1], u=self.inputs[:,-1], w=np.zeros(2))]




if __name__ == '__main__':
    # pathPoints = np.c_[np.linspace(0,250,100), np.zeros(100)].T
    # currentState = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0])

    pathPoints = np.genfromtxt('/Users/tudoroancea/Developer/racing-team/simulation/Track Racelines/Budapest.csv', delimiter=',', skip_header=1, unpack=True)
    pathPoints = pathPoints[:,:300]
    currentState = np.array([-6.075903,-4.259295,np.deg2rad(135),0,0,0])
    # pathPoints = np.genfromtxt('/Users/tudoroancea/Developer/racing-team/simulation/Track Racelines/MexicoCity.csv', delimiter=',', skip_header=1, unpack=True)
    # pathPoints = pathPoints[:,:50]
    # currentState = np.array([-1.058253,8.811078,np.deg2rad(-15),0,0,0])

    C = StanleyPIDController(pathPoints=pathPoints, initialState=currentState[:4])

    statesFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/states.csv', 'w')
    inputsFile = open('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/PIDPython/inputs.csv', 'w')

    statesFile.write('{},{},{},{},{},{},{}\n'.format(0, currentState[0], currentState[1], currentState[2], currentState[3], currentState[4], currentState[5]))

    simulationLength = 1500
    for k in range(simulationLength):
        newInputs = C.step(k)
        if C.inputs.size > 2:
            w=C.samplingTime*(C.inputs[:,-1]-C.inputs[:,-2])
        else:
            w=np.zeros(2)
        currentState = C.computeNextState(x=currentState, u=C.inputs[:,-1], w=w)
        C.states = np.c_[C.states, currentState[:4]]
        
        statesFile.write('{},{},{},{},{},{},{}\n'.format(k+1, currentState[0], currentState[1], currentState[2], currentState[3], currentState[4], currentState[5]))
        inputsFile.write('{},{},{}\n'.format(k, newInputs[0], newInputs[1]))

    statesFile.close()
    inputsFile.close()
    plt.figure(num=1)
    plt.plot(C.states[0,:], C.states[1,:], 'b-')
    plt.plot(pathPoints[0,:], pathPoints[1, :], 'g-')
    plt.show()