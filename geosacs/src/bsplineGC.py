import numpy as np

def bspline_GC(directrix, eN, eB, eT, Curve):
    xn = directrix[:, 0]
    yn = directrix[:, 1]
    zn = directrix[:, 2]
    
    num_points = len(xn) - 1
    num_arcs = Curve[0]['curve'].shape[1]
    
    GC = np.zeros((3, num_arcs, num_points))
    
    for ii in range(0, num_points, 1):
        x = Curve[ii]['curve'][0, :]
        y = Curve[ii]['curve'][1, :]
        x = x.reshape(-1, 1)
        y = y.reshape(-1, 1)
        
        C = np.tile(directrix[ii, :], (num_arcs, 1)) + (
            np.array([eN[ii, :], eB[ii, :], [0, 0, 0]]).T @ np.array([x.flatten(), y.flatten(), np.zeros(num_arcs)])).T
        GC[:, :, ii] = C.T
    
    return GC