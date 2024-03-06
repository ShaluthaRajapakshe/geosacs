import numpy as np
from scipy.interpolate import splprep, splev

def extract_boundaries_bspline(directrix, X1s, X2s, X3s, eT, eN, eB):
    num_points = directrix.shape[0]
    num_traj = X1s.shape[1]
    neighborhood = num_points // 3
    
    Curve = []
    
    tt = np.arange(0, num_points)
    Idx1 = np.maximum(tt - neighborhood, 0)
    Idx2 = np.minimum(tt + neighborhood, num_points)
    
    for ii in range(num_points - 1):
        idx1, idx2 = Idx1[ii], Idx2[ii]
        point1 = directrix[ii, :]
        distances = np.zeros(num_traj)
        max_point = np.zeros((num_traj, 3))
        
        for jj in range(num_traj):
            min_func = eT[ii, 0] * (X1s[idx1:idx2, jj] - point1[0]) + \
                       eT[ii, 1] * (X2s[idx1:idx2, jj] - point1[1]) + \
                       eT[ii, 2] * (X3s[idx1:idx2, jj] - point1[2])
            
            idx3 = np.argmin(np.abs(min_func))
            point2 = np.array([X1s[idx1 - 1 + idx3, jj], X2s[idx1 - 1 + idx3, jj], X3s[idx1 - 1 + idx3, jj]])
            max_point[jj, :] = point2
            distances[jj] = np.linalg.norm(point1 - point2)
        
        new_points = np.zeros((num_traj, 2))
        for k in range(num_traj):
            new_points[k, :] = [np.dot(eN[ii, :], max_point[k, :] - point1),
                                np.dot(eB[ii, :], max_point[k, :] - point1)]
        
        tck, _ = splprep(new_points.T, s=0)
        u = np.linspace(0, 1, num=100)
        curve = splev(u, tck)
        Curve.append(curve)
    
    return Curve