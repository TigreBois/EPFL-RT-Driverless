from matplotlib import animation
import matplotlib.pyplot as plt
from skidpadGeneration import *
from StanleyPIDPython import StanleyPIDController, CarModel
from common import *

# General simulation parameters
samplingTime = 0.05 # [s]
simulationLength = 500 # iterations

# CHOOSE YOUR PATH =====================================================================================
pathPoints = shortSkidpad
currentState = np.array([0.0,-15.0,np.pi/2,0.0,0.0,0.0])

# DECKARE THE CONTROLLER INSTANCE =====================================================================================
stanleyController = StanleyPIDController(pathPoints=pathPoints, initialState=currentState[:4], mission=Mission.SKIDPAD)
states = np.zeros((6,1))
states[:,0] = currentState[:]
inputs = np.zeros((2,0))
# DECLARE THE PHYSICAL MODEL INSTANCE =====================================================================================
carModel = CarModel(samplingTime=samplingTime, physicalModel=CarModel.PhysicalModel.KINEMATIC, integrator=CarModel.Integrator.RK4)

fig = plt.figure()
trajectoryPlot = fig.add_subplot(autoscale_on=False, xlim=(-21, 21), ylim=(-16, 13))
trajectoryPlot.set_aspect('equal')

# track
trajectoryPlot.plot(pathPoints[0, :], pathPoints[1, :], 'g-', label="Reference Track", zorder=1, lw=1)
trajectoryPlot.plot(skidpadBoundaries[0, :], skidpadBoundaries[1, :], 'ko', label="Skidpad Boundaries", zorder=1, lw=1)

carTraj, = trajectoryPlot.plot([], [], 'b-', label="Car Position", zorder=2, lw=2)
timeString = 'time %.1fs'
timeText = trajectoryPlot.text(0.02, 0.95, '', transform=trajectoryPlot.transAxes)


# IT'S SIMULATION TIME =====================================================================================
def animate(i):
    if stanleyController.drivingMode != DrivingMode.MISSION_FINISHED:
        global currentState, states, inputs
        newInputs, headingError, crossTrackError, closestPointID, closestPoint, closestPointDistance = stanleyController.computeNextInput()
        # adding error on the inputs
        newInputs += np.array([np.random.normal(0, np.deg2rad(10)),np.random.normal(0,0.5)])
        inputs = np.column_stack((inputs, newInputs))
        w = (inputs[:, -1]-inputs[:, -2])/samplingTime if inputs.shape[1] > 1 else np.zeros(2)
        currentState = carModel.computeNextState(x=currentState, u=newInputs, w=w)
        stanleyController.setState(currentState[:4])
        states = np.column_stack((states, currentState))

        # referenceTraj.set_data(pathPoints[0, :], pathPoints[1, :])
        carTraj.set_data(states[0, :], states[1, :])
        timeText.set_text(timeString % (i*samplingTime))

    # FOR SKIDPAD  ===================================================================================== 
    if closestPointID >= shortSkidpadEnd+3:
        # we have passed the finish line
        stanleyController.drivingMode = DrivingMode.STOPPING
    if stanleyController.drivingMode == DrivingMode.STOPPING and currentState[3] < 0.1:
        # the car has stopped
        stanleyController.drivingMode = DrivingMode.MISSION_FINISHED
        global ani
        ani.pause()

    return carTraj, timeText


ani = animation.FuncAnimation(fig, animate, frames=simulationLength, interval=samplingTime*1000, blit=True)
plt.show()
# ani.save('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/StanleyPID/StanleyPID-outputs/skidpadAnimation.mp4', dpi=200, fps=30,bitrate=5000)
# ani.save('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/StanleyPID/StanleyPID-outputs/shortSkidpadAnimation.mp4', dpi=200, fps=30,bitrate=5000)
# ani.save('/Users/tudoroancea/Developer/racing-team/EPFL-RT-Driverless/Control/StanleyPID/StanleyPID-outputs/shortSkidpadAnimation-steering-10.mp4', dpi=200, fps=30,bitrate=5000)

