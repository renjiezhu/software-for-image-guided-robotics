import numpy as np
import matplotlib.pyplot as plt

class trajectoryGenerator:
    
    #list of start points and end points and corresponding max velocities,
    #period is in how fast the output command is sent to the robot, unit should be same as velocity
    def createTrajectoryMaxVelocity(self, startPoints, endPoints, maxVelocities, period):
        
        startPoints = np.array(startPoints)
        endPoints = np.array(endPoints)
        maxVelocities = np.array(maxVelocities)
        
        dist = np.abs(endPoints - startPoints)
        
        #cubic spline has maximum velocity at the center (1.5 distance/sample)
        numPoints_list = (np.ceil(1.5*(dist/(period*maxVelocities)))).astype(int)
                        
        trajectory = np.ones((len(startPoints), np.max(numPoints_list)))
        time = np.linspace(0, np.max(numPoints_list)*period, np.max(numPoints_list))
                
        for i in range(0, len(startPoints)):
            trajectory[i, :numPoints_list[i]] = self.createTrajectoryNumPoints(startPoints[i], endPoints[i], numPoints_list[i])
            trajectory[i, numPoints_list[i]:] = endPoints[i]
            
        return trajectory, time
    
    def createTrajectoryNumPoints(self, startPoints, endPoints, num_points):
        
        startPoints = np.array(startPoints)
        endPoints = np.array(endPoints)
        
        #Create smooth function with cubic spline
        # y = -2x^3 + 3x^2
        x = np.linspace(0, 1, num_points)
        y = (-2*x**3 + 3*x**2)
        
        dist = endPoints - startPoints
        if dist.size > 1:
            numDim = len(dist)
        else:
            numDim = 1
            
        trajectory = np.ones((numDim, num_points))
        
        if numDim > 1:
            for i in range(0, numDim):
                trajectory[i, :] = dist[i]*y + startPoints[i]
        else:
            trajectory[0, :] = dist*y + startPoints
            
        return trajectory
        
    def plotTrajectory(self, trajectory, time = None):
        for i in range(0, trajectory.shape[0]):
            if time is not None:
                plt.plot(time, trajectory[i,:])
            else:
                plt.plot(trajectory[i,:])
        plt.show()