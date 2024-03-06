import numpy as np
from getTNB import get_TNB

from extractBoundariesCircle import extract_boundaries_circle
from extractBoundariesBSpline import extract_boundaries_bspline

from circularGC import circular_GC
from bsplineGC import bspline_GC

def construct_GC(demos, num_demos, num_points, idx, cross_section_type = 'circle'):
    # Get the X, Y, Z matrices (size = number of points, number of trajectories)
    X1s = np.zeros((num_points, num_demos))
    X2s = np.zeros((num_points, num_demos))
    X3s = np.zeros((num_points, num_demos))
    for i in range(num_demos):
        # X1s[:, ii] = demos[ii][:, 0]
        # X2s[:, ii] = demos[ii][:, 1]
        # X3s[:, ii] = demos[ii][:, 2]
        X1s[:, i] = demos[:,i,0]
        X2s[:, i] = demos[:,i,1]
        X3s[:, i] = demos[:,i,2]


    s = np.linspace(0, 1, num_points)

    # Calculate the mean trajectory (Point wise mean for each demo)
    directrix = np.column_stack((np.mean(X1s, axis=1),
                                    np.mean(X2s, axis=1),
                                    np.mean(X3s, axis=1)))
    
    # print('directrix Shape', directrix.shape) ##correct up to here

    # Calculate the TNB Frames
    eT, eN, eB = get_TNB(directrix)


    # print("Sucessfully completed calculating TNB Frames")

    # cross_section_type = 'circle'

    if cross_section_type == 'circle':
        Rc, threshold = extract_boundaries_circle(directrix, X1s, X2s, X3s)
    elif cross_section_type == 'spline':
        Curve = extract_boundaries_bspline(directrix, X1s, X2s, X3s, eT, eN, eB)
    else:
        raise ValueError('No boundary was extracted!')

    idx1 = idx[0]
    idx2 = idx[1]

    # print("et shape before", eT.shape, "idx1 and idx 2", idx1, idx2, 'directrix shape', directrix.shape)
    # print("diretrix reshaping", directrix.shape[0], directrix[idx1:1500-idx2].shape)
    # Get only subset of values within specified boundary
    directrix_len = directrix.shape[0]

    ss = s[idx1:directrix_len-idx2]
    eT = eT[idx1:directrix_len-idx2, :]
    eN = eN[idx1:directrix_len-idx2, :]
    eB = eB[idx1:directrix_len-idx2, :]
    directrix = directrix[idx1:directrix_len-idx2, :]
    # print('after diretrix', directrix.shape)

    # make the model as a dictionary
    model = {}
    model['s'] = ss
    model['eN'] = eN
    model['eB'] = eB
    model['eT'] = eT
    model['directrix'] = directrix

    # Construct the GC depending on the cross-section type
    if cross_section_type == 'circle':
        RRc = Rc[idx1:directrix_len-idx2]

        # print('Rc shape ', Rc.shape, 'RRc shape', RRc.shape)

        GC = circular_GC(directrix, eN, eB, RRc)
        model['Rc'] = RRc
        model['GC'] = GC
    elif cross_section_type == 'spline':
        Curvec = Curve[0, idx1:-idx2]
        GC = bspline_GC(directrix, eN, eB, eT, Curvec)
        model['Curve'] = Curvec
        model['GC'] = GC
    else:
        raise ValueError('Error! Cross-section type unknown!')

    return model