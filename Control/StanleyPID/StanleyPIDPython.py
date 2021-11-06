import numpy as np
import scipy.spatial.distance as dist
import matplotlib.pyplot as plt
from skidpadGeneration import skidpad

class CarModel:
    # misc
    samplingTime = 0.05

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

    def __init__(self, samplingTime = 0.05):
        self.samplingTime = samplingTime

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

    def dynamicCarDynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        """ 
            x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
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


    def mixedCarDynamics(self, x: np.ndarray, u: np.ndarray, w: np.ndarray) -> np.ndarray:
        """ 
            x : car state = [X, Y, phi, v_x, v_y, r] 
            u : control input state = [delta, D]
            w : MPC input = [ddelta, dD]
            returns : xdot
        """
        assert x.size == 6 and (x.shape == (6, 1) or x.shape == (6,))
        assert u.size == 2 and (u.shape == (2, 1) or u.shape == (2,))
        assert w.size == 2 and (w.shape == (2, 1) or w.shape == (2,))

        F_F_y = self.D_F * np.sin(self.C_F*np.arctan(self.B_F * np.arctan((x[4] + self.l_F*x[5]) / x[3])-u[0])) if x[3] != 0 else 0.0
        F_R_y = self.D_R * np.sin(self.C_R*np.arctan(self.B_R * np.arctan((x[4] - self.l_R*x[5]) / x[3]))) if x[3] != 0 else 0.0
        blendCoef = min(max((x[3]-self.v_blend_min) / (self.v_blend_max-self.v_blend_min), 0), 1)
        return np.array([x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]),
                        x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]),
                        x[5],
                        u[1] - blendCoef*F_F_y *np.sin(u[0]) / self.m + x[4] * x[5] * blendCoef,
                        blendCoef*(F_R_y+F_F_y*np.cos(u[0])-self.m*x[3]*x[5])/self.m + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])*self.l_R/(self.l_R+self.l_F),
                        # blendCoef*(np.sin(u[0])*self.l_F*np.cos(u[0])-F_R_y*self.l_R)/self.I_z + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])/(self.l_R+self.l_F)
                        blendCoef*(F_F_y*self.l_F*np.cos(u[0])-F_R_y*self.l_R)/self.I_z + (1-blendCoef)*(w[0]*x[3]+u[0]*u[1])/(self.l_R+self.l_F)
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

        # Pure RK4 scheme integration of the dynamics ============================================================
        # k1 = self.mixedCarDynamics(x, u, w)
        # k2 = self.mixedCarDynamics(x + self.samplingTime*0.5*k1, u, w)
        # k3 = self.mixedCarDynamics(x + self.samplingTime*0.5*k2, u, w)
        # k4 = self.mixedCarDynamics(x + self.samplingTime*k3, u, w)
        
        # k1 = self.kinematicCarDynamics(x, u, w)
        # k2 = self.kinematicCarDynamics(x + self.samplingTime*0.5*k1, u, w)
        # k3 = self.kinematicCarDynamics(x + self.samplingTime*0.5*k2, u, w)
        # k4 = self.kinematicCarDynamics(x + self.samplingTime*k3, u, w)

        # k1 = self.dynamicCarDynamics(x, u, w)
        # k2 = self.dynamicCarDynamics(x + self.samplingTime*0.5*k1, u, w)
        # k3 = self.dynamicCarDynamics(x + self.samplingTime*0.5*k2, u, w)
        # k4 = self.dynamicCarDynamics(x + self.samplingTime*k3, u, w)

        # return x+self.samplingTime/6*(k1+2*k2+2*k3+k4)

        # Pure Euler scheme integration of the dynamics ============================================================
        # return x+self.samplingTime*self.mixedCarDynamics(x,u,w)
        return x+self.samplingTime*self.kinematicCarDynamics(x,u,w)
        # return x+self.samplingTime*self.dynamicCarDynamics(x,u)


        # kinematic using upgraded values when we can ============================================================
        # new_v_x = x[3] + self.samplingTime * u[1]
        # new_v_y = x[4] + self.samplingTime * (w[0]*new_v_x+u[0]*u[1])*self.kinCoef1
        # new_r = x[5] + self.samplingTime * (w[0]*new_v_x+u[0]*u[1])*self.kinCoef2
        # new_phi = x[2] + self.samplingTime * new_r
        # new_X = x[0] + self.samplingTime*(new_v_x*np.cos(new_phi) - new_v_y*np.sin(new_phi))
        # new_Y = x[1] + self.samplingTime*(new_v_x*np.sin(new_phi) + new_v_y*np.cos(new_phi))
        # new_X = x[0] + self.samplingTime*(x[3]*np.cos(x[2]) - x[4]*np.sin(x[2]))
        # new_Y = x[1] + self.samplingTime*(x[3]*np.sin(x[2]) + x[4]*np.cos(x[2]))
        # return np.array([new_X, new_Y, new_phi, new_v_x, new_v_y, new_r])

        # dynamic using upgraded values when we can ============================================================
        # F_F_y = self.D_F * np.sin(self.C_F*np.arctan(self.B_F * np.arctan((x[4] + self.l_F*x[5]) / x[3])-u[0])) if x[3] != 0 else 0.0
        # new_v_x = x[3] + self.samplingTime * (u[1] - F_F_y*np.sin(u[0]) / self.m + x[4] * x[5]*self.m)
        # F_R_y = self.D_R * np.sin(self.C_R*np.arctan(self.B_R * np.arctan((x[4] - self.l_R*x[5]) / x[3]))) if x[3] != 0 else 0.0


from enum import Enum
class DrivingMode(Enum):
    DRIVING = 0
    STOPPED = 1
    MISSION_FINISHED = 2

class StanleyPIDController:
    """
        General procedure : At time t_k we specify the state x_k. Then we compute the velocity error v_err_k at time t_k, and the control inputs u_k we want to apply. These will induce a new state x_(k+1) at the next sampling time.
        States : x = [X,Y,phi,v_x]
        Control inputs : u = [a_x, delta]
     """
    # misc
    referencePath: np.ndarray
    samplingTime = 0.05 # s
    delta_max = np.deg2rad(30.0)
    drivingMode = DrivingMode.DRIVING

    # last values
    lastState = np.zeros(4)
    lastInput = np.zeros(2)
    lastLongitudinalError = 0.0
    lastProjectionID = 0

    # PI gains
    K_P = 3  # [s^-1]
    K_I = 1   # [s^-2]
    # Stanley parameters
    k_Delta = 6.0e1
    k_s = 1.0
    k_d = 1.0e2

    def __init__(self, pathPoints: np.ndarray, initialState: np.ndarray, sampligTime = 0.05, outputFile = ''):
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
            referencePathFile = open(outputPath+'referencePath.csv', 'w')
            referencePathFile.write('rank,x,y,tangentAngle\n')
            for i in range(0,nbrOfPoints):
                referencePathFile.write('{},{},{},{}\n'.format(i,self.referencePath[0,i], self.referencePath[1,i], self.referencePath[2,i]))
            referencePathFile.close()        

        # initialize the state
        assert initialState.shape == (4, 1) or initialState.shape == (4,), "initialState must be a numpy vector (1D or 2D array) with 4 variables: X, Y, phi, v_x"
        self.lastState = initialState

        self.samplingTime = sampligTime


    def v_ref(self):
        return 7.0 if self.drivingMode == DrivingMode.DRIVING else 0.0
        
    def setState(self, newState: np.ndarray):
        assert newState.shape == (4, 1) or newState.shape == (4,), "newState must be a numpy vector (1D or 2D array) with 4 variables: X, Y, phi, v_x"
        self.lastState = newState

    def computeNextInput(self):
        """
            Computes and returns the new control inputs
         """
        # find new acceleration input ============================================================
        newLongitudinalError = self.v_ref() - self.lastState[3]
        new_a_x = self.K_P * newLongitudinalError + 0.5 * self.K_I * (newLongitudinalError + self.lastLongitudinalError) * self.samplingTime
        self.lastLongitudinalError = newLongitudinalError

        # find new steering input ============================================================
        # find the closest point on the path
        consideredPoints = self.referencePath[0:2,self.lastProjectionID:self.lastProjectionID+10]
        # consideredPoints = self.referencePath[0:2,self.lastProjectionID:min(self.lastProjectionID+10, self.referencePath.shape[1])]
        # consideredPoints = self.referencePath[0:2,:]
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



if __name__ == '__main__':
    # General simulation parameters
    samplingTime = 0.05 # [s]
    simulationLength = 850 # iterations

    # CHOOSE THE PATH =====================================================================================

    # SRAIGHT LINE ===========================================================
    # pathPoints = np.c_[np.linspace(0,250,100), np.zeros(100)].T
    # currentState = np.array([0.0,0.0, 0.0, 0.0, 0.0, 0.0])
    # SKIDPAD ================================================================
    pathPoints = skidpad[:,:]
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
    carModel = CarModel(samplingTime=samplingTime)

    # SIMULATION TIME =====================================================================================
    for k in range(0, simulationLength):
        newInputs, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance = stanleyController.computeNextInput()
        inputs = np.column_stack((inputs, newInputs))

        w = (inputs[:, -1]-inputs[:, -2])/samplingTime if inputs.shape[1] > 1 else np.zeros(2)
        currentState = carModel.computeNextState(x=currentState, u=newInputs, w=w)
        stanleyController.setState(currentState[:4])
        states = np.column_stack((states, currentState))

        inputsFile.write('{},{},{}\n'.format(k, newInputs[0], newInputs[1]))
        statesFile.write('{},{},{},{},{},{},{}\n'.format(k+1, currentState[0], currentState[1], currentState[2], currentState[3], currentState[4], currentState[5]))
        steeringFile.write('{},{},{},{},{},{}\n'.format(k, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance))

        # # display
        # plt.clf()
        # # Display track, start and end
        # plt.plot(currentState[0], currentState[1], zorder=31, label="Pos")


        # #Display path as points
        # #plt.scatter(x_y[0], x_y[1], zorder=1, label="Track")
        # #Display path as line
        # plt.plot(x_y[0], x_y[1], zorder=1, label="Track")
        
        # plt.scatter(points[0][0], points[0][1],color='red', zorder=2, label="Start/finish point")
        # plt.scatter(points[len(points)-1][0], points[len(points)-1][1], color='red', zorder=2)

        # # Display the position
        # plt.scatter(position[0], position[1], c=10, s=20, zorder=2, label="Current position")

        # # Display the predicted position
        # x_y = list(zip(*predicted_pos))
        # plt.plot(x_y[0], x_y[1], '--r', zorder=2, label="Predicted trajectory")


        # # Add legend under the plot
        # plt.legend(bbox_to_anchor=(0.5, -0.5), loc='lower center', borderaxespad=0.) 
        # plt.figure(1).subplots_adjust(bottom=0.3)

        # # Must pause the dispay otherwise the update are too fast for it to be generated
        # plt.pause(samplingTime)

    statesFile.close()
    inputsFile.close()
    steeringFile.close()
    plt.figure(num=1)
    plt.plot(states[0, :], states[1, :], 'b+')
    plt.plot(pathPoints[0, :], pathPoints[1, :], 'g-')
    plt.axis('equal')
    plt.show()
