import numpy as np
import scipy.spatial.distance as dist
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from enum import Enum
import time

from enums import *
from skidpadGeneration import *

class CarModel:
    # misc
    samplingTime = 0.05
    physicalModel = PhysicalModel.KINEMATIC

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
    v_blend_min = 3.0
    v_blend_max = 5.0
    kinCoef1 = l_R/(l_R+l_F)
    kinCoef2 = 1/(l_R+l_F)

    def __init__(self, samplingTime = 0.05, physicalModel = PhysicalModel.KINEMATIC):
        self.samplingTime = samplingTime
        self.physicalModel = physicalModel

    def kinematicCarDynamics(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ 
            x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : MPC input = [ddelta, dD]
            returns xdot
         """
        assert x.size == 6 and (x.shape == (6, 1) or x.shape == (6,))
        assert u.size == 2 and (u.shape == (2, 1) or u.shape == (2,))
        assert w.size == 2 and (w.shape == (2, 1) or w.shape == (2,))

        return np.array([
            x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]),
            x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]),
            x[5],
            u[1],
            (w[0]*x[3]+u[0]*u[1])*self.kinCoef1,
            (w[0]*x[3]+u[0]*u[1])*self.kinCoef2
        ])

    def dynamicCarDynamics(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ 
            x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : useless in this method but here for consistency with the other dynamics
            returns xdot
         """
        assert x.size == 6 and (x.shape == (6, 1) or x.shape == (6,))
        assert u.size == 2 and (u.shape == (2, 1) or u.shape == (2,))

        F_F_y = self.D_F * np.sin(self.C_F*np.arctan(self.B_F * np.arctan((x[4] + self.l_F*x[5]) / x[3])-u[0])) if x[3] != 0 else 0.0
        F_R_y = self.D_R * np.sin(self.C_R*np.arctan(self.B_R * np.arctan((x[4] - self.l_R*x[5]) / x[3]))) if x[3] != 0 else 0.0

        return np.array([
            x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]),
            x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]),
            x[5],
            u[1] -np.sin(u[0])*F_F_y/self.m+x[4]*x[5],
            (F_R_y + F_F_y*np.cos(u[0] - self.m*x[3]*x[5]))/self.m,
            (F_F_y*self.l_F*np.cos(u[0]) - F_R_y*self.l_R)/self.I_z
        ])

    def computeNextState(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ 
            x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : MPC input = [ddelta, dD]
            returns xdot
         """
        assert x.size == 6 and (x.shape == (6, 1) or x.shape == (6,))
        assert u.size == 2 and (u.shape == (2, 1) or u.shape == (2,))
        assert w.size == 2 and (w.shape == (2, 1) or w.shape == (2,))

        if self.physicalModel == PhysicalModel.KINEMATIC:
            dynamics = self.kinematicCarDynamics
        else:
            dynamics = self.dynamicCarDynamics

        k1 = dynamics(x, u, w)
        k2 = dynamics(x + self.samplingTime*0.5*k1, u, w)
        k3 = dynamics(x + self.samplingTime*0.5*k2, u, w)
        k4 = dynamics(x + self.samplingTime*k3, u, w)
        return x+self.samplingTime/6*(k1+2*k2+2*k3+k4)

class StanleyPIDController:
    """
        General procedure : At time t_k we specify the state x_k. Then we compute the velocity error v_err_k at time t_k, and the control inputs u_k we want to apply. These will induce a new state x_(k+1) at the next sampling time.
        States of the form [X,Y,phi,v_x]
        Control inputs of the form [a_x, delta]
     """
    # misc
    referencePath: np.ndarray
    samplingTime = 0.05 # s
    delta_max = np.deg2rad(30.0) # rad
    a_max = 15.0 # m/s^2
    drivingMode = DrivingMode.DRIVING
    mission = Mission.SKIDPAD

    # last values
    lastState = np.zeros(4)
    lastInput = np.zeros(2)
    lastLongitudinalError = 0.0
    lastProjectionID = 0

    # PI gains
    K_P = 3  # [s^-1]
    K_I = 1   # [s^-2]
    # Stanley parameters
    k_Delta = 3e2
    k_s = 1.0
    k_d = 1.0e2

    def __init__(self, pathPoints: np.ndarray, initialState: np.ndarray, sampligTime = 0.05, mission=Mission.SKIDPAD, outputFile = ''):
        """
            pathPoints : numpy array with 2 rows (one for x coord and one for y coord) and n columns (n points on the path)
            initialState : numpy array of shape (4,1)
         """
        # initialize the reference path
        assert pathPoints.shape[0] == 2 and pathPoints.shape[1] > 1, "pathPoints must be a numpy array with 2 rows and at least 2 columns"
        nbrOfPoints = pathPoints.shape[1]
        self.referencePath = np.zeros((3, nbrOfPoints))
        self.referencePath[0:2, :] = pathPoints
        for i in range(0,nbrOfPoints-1):
            self.referencePath[2, i] = np.arctan2(pathPoints[1, i+1]-pathPoints[1, i], pathPoints[0, i+1]-pathPoints[0, i])
        self.referencePath[2,-1] = self.referencePath[2,-2]

        if outputFile != '':
            referencePathFile = open(outputFile+'referencePath.csv', 'w')
            referencePathFile.write('rank,x,y,tangentAngle\n')
            for i in range(0,nbrOfPoints):
                referencePathFile.write('{},{},{},{}\n'.format(i,self.referencePath[0,i], self.referencePath[1,i], self.referencePath[2,i]))
            referencePathFile.close()        

        # initialize the state
        assert initialState.shape == (4, 1) or initialState.shape == (4,), "initialState must be a numpy vector (1D or 2D array) with 4 variables: X, Y, phi, v_x"
        self.lastState = initialState

        self.samplingTime = sampligTime
        self.mission = mission


    def v_ref(self):
        if self.drivingMode == DrivingMode.DRIVING:
            if self.mission == Mission.SKIDPAD:
                return 12.0
            elif self.mission == Mission.ACCELERATION or self.mission == Mission.INFINITE_STRAIGHT_LINE:
                return 30.0
            else: 
                return 5.0
        else:
            return 0.0
        
    def setState(self, newState: np.ndarray):
        """
            newState : numpy array of shape (4,1) corresponding to [X, Y, phi, v_x]
         """
        assert newState.shape == (4, 1) or newState.shape == (4,), "newState must be a numpy vector (1D or 2D array) with 4 variables: X, Y, phi, v_x"
        self.lastState = newState

    def computeNextInput(self):
        """
            Computes and returns the new control inputs in the form [a_x, delta]
         """
        # find new acceleration input ============================================================
        newLongitudinalError = self.v_ref() - self.lastState[3]
        new_a_x = self.K_P * newLongitudinalError + 0.5 * self.K_I * (newLongitudinalError + self.lastLongitudinalError) * self.samplingTime
        new_a_x = np.clip(new_a_x, -self.a_max, self.a_max)
        self.lastLongitudinalError = newLongitudinalError

        # find new steering input ============================================================
        # find the closest point on the path
        consideredPoints = self.referencePath[0:2,self.lastProjectionID:self.lastProjectionID+10]
        closestPointID = self.lastProjectionID + np.argmin(np.sum(np.square(consideredPoints - self.lastState[:2].reshape(2,1)), axis=0))
        closestPoint = self.referencePath[0:2, closestPointID]
        closestPointDistance = dist.euclidean(self.lastState[:2], closestPoint)
        self.lastProjectionID = closestPointID

        # find the heading error
        tangentVectorAngle = self.referencePath[2, closestPointID]
        headingVectorAngle = self.lastState[2]
        diff1 = float(tangentVectorAngle - headingVectorAngle)
        headingError = (diff1 + np.pi) % (2*np.pi) - np.pi

        # find the cross track error
        crossTrackError = np.arctan(self.k_Delta*closestPointDistance/(self.k_s+self.k_d*self.lastState[3]))
        crossTrackAngle = np.arctan2(closestPoint[1] - self.lastState[1], closestPoint[0] - self.lastState[0]) # angle between e_∆ and the horizontal line in the report on stanley
        diff2 = crossTrackAngle - self.referencePath[2, closestPointID]
        if np.abs(diff2) > np.pi:
            sign2 = -np.sign(diff2)
        else:
            sign2 = np.sign(diff2)
        if sign2 > 0:
            crossTrackError = np.abs(crossTrackError)
        else:
            crossTrackError = -np.abs(crossTrackError)

        newDelta = headingError + crossTrackError
        newDelta = np.clip(newDelta, -self.delta_max, self.delta_max)
        self.lastInput = np.array([newDelta, new_a_x])

        return self.lastInput, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance


def wrapperStanleyControl(referencePath: np.ndarray, currentState: np.ndarray, samplingTime = 0.05, mission=Mission.SKIDPAD) -> np.ndarray:
    """
        referencePath : numpy array with 2 rows (one for x coord and one for y coord) and n columns (n points on the path)
        currentState : numpy array of shape (4,1) or (4,) => (X, Y, phi, v_x)
        samplingTime : sampling time of the controller, ie time between two calls of the controller
        mission : the mission to be performed by the controller (which type of track is followed : Straight line, Skidpad, ...)
        returns the new control inputs
     """
    controller = StanleyPIDController(referencePath, currentState, samplingTime, mission)
    return controller.computeNextInput()

if __name__ == '__main__':
    # General simulation parameters
    samplingTime = 0.05 # [s]
    simulationLength = 500 # iterations

    # CHOOSE THE PATH =====================================================================================

    # SRAIGHT LINE ===========================================================
    # pathPoints = np.c_[np.linspace(0,250,100), np.zeros(100)].T
    # currentState = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0])
    # SKIDPAD ================================================================
    pathPoints = shortSkidpad
    currentState = np.array([0.0,-15.0,np.pi/2,0.0,0.0,0.0])
    # BUDAPEST ================================================================
    # pathPoints = np.genfromtxt('/Users/tudoroancea/Developer/racing-team/simulation/Track Racelines/Budapest.csv', delimiter=',', skip_header=1, unpack=True)
    # pathPoints = pathPoints[:, :300]
    # currentState = np.array([-6.075903, -4.259295, np.deg2rad(135), 0, 0, 0])
    # MEXICO CITY ================================================================
    # pathPoints = np.genfromtxt('/Users/tudoroancea/Developer/racing-team/simulation/Track Racelines/MexicoCity.csv', delimiter=',', skip_header=1, unpack=True)
    # pathPoints = pathPoints[:,:5000]
    # currentState = np.array([-1.058253,8.811078,np.deg2rad(-15),0,0,0])
    
    import os

    outputPath = os.path.dirname(__file__)+'/StanleyPID-outputs/'
    if not os.path.exists(outputPath):
        os.mkdir(outputPath)

    statesFile = open(outputPath+'states.csv', 'w')
    inputsFile = open(outputPath+'inputs.csv', 'w')
    steeringFile = open(outputPath+'steering.csv', 'w')
    statesFile.write('{},{},{},{},{},{},{}\n'.format(0, currentState[0], currentState[1], currentState[2], currentState[3], currentState[4], currentState[5]))
    steeringFile.write('rank, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance\n')

    # DECKARE THE CONTROLLER INSTANCE =====================================================================================
    stanleyController = StanleyPIDController(pathPoints=pathPoints, initialState=currentState[:4], outputFile=outputPath)
    states = np.zeros((6,1))
    states[:,0] = currentState[:]
    inputs = np.zeros((2,0))
    # DECLARE THE PHYSICAL MODEL INSTANCE =====================================================================================
    carModel = CarModel(samplingTime=samplingTime, physicalModel=PhysicalModel.KINEMATIC)

    # IT'S SIMULATION TIME =====================================================================================
    for k in range(0, simulationLength):
        start = time.time()

        newInputs, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance = stanleyController.computeNextInput()
        newInputs += np.array([np.random.normal(0, np.deg2rad(10)),np.random.normal(0,0.5)])
        inputs = np.column_stack((inputs, newInputs))

        w = (inputs[:, -1]-inputs[:, -2])/samplingTime if inputs.shape[1] > 1 else np.zeros(2)
        currentState = carModel.computeNextState(x=currentState, u=newInputs, w=w)
        stanleyController.setState(currentState[:4])
        states = np.column_stack((states, currentState))
        
        stop = time.time()
        # print(1000*(stop-start), 'ms')

        inputsFile.write('{},{},{}\n'.format(k, newInputs[0], newInputs[1]))
        statesFile.write('{},{},{},{},{},{},{}\n'.format(k+1, currentState[0], currentState[1], currentState[2], currentState[3], currentState[4], currentState[5]))
        steeringFile.write('{},{},{},{},{},{}\n'.format(k, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance))

        if stanleyController.mission == Mission.SKIDPAD:
            if closestPointID >= shortSkidpadEnd:
                # we have passed the finish line
                stanleyController.drivingMode = DrivingMode.STOPPING
            if stanleyController.drivingMode == DrivingMode.STOPPING and currentState[3] < 0.1:
                # the car has stopped
                stanleyController.drivingMode = DrivingMode.MISSION_FINISHED
                break


    statesFile.close()
    inputsFile.close()
    steeringFile.close()
    plt.figure()
    plt.plot(states[0, :], states[1, :], 'b-', zorder=2, label="Car Position", lw=1)
    plt.plot(pathPoints[0, :], pathPoints[1, :], 'g-', zorder=1, label="Reference Track")
    plt.axis('equal')
    plt.legend(bbox_to_anchor=(0.5, -0.5), loc='lower center', borderaxespad=0.)
    plt.ylim(top=15, bottom=-15)
    plt.xlim(left=-15, right=15)
    plt.show()
