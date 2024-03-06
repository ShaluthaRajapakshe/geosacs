import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

def plotReproduction(initPoints, newTrajectories):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.scatter(initPoints[:, 0], initPoints[:, 1], initPoints[:, 2], color='red', marker='o', label='Initial Points')
    
    for traj in newTrajectories:
        traj = np.array(traj)
        ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2], color='red', marker='o', linewidth=2)
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], marker='.', linestyle='-', linewidth=1, color='m')
    
    ax.legend()
    plt.show()

# Usage:
# plotReproduction(initPoints, newTrajectories)
