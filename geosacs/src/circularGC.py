import numpy as np

def circular_GC(directrix, eN, eB, Rc):
    tc = np.arange(0, 1.01, 0.01)  # arc-length on the circle
    num_points = directrix.shape[0]  # number of points on directrix
    num_arcs = len(tc)  # number of points on each circle
    stepc=1
    # stepc = int(0.25 * num_points)
    # stepc = int(0.05 * num_points)  # 1% of the data is used
    L = len(range(0, num_points, stepc))  # number of cross-sections to calculate

    GC = np.zeros((L, num_arcs, 3))  # Generalized Cylinder

    # print("Num of cross sections", L, "GC shape", GC.shape, "EN shape", eN.shape, "EB shape", eB.shape, "Rc shape", Rc.shape)
    k = 0

    ######## TO DO: CHECK THIS ALGOTITHM ON FINDING THE COORDINATES ########
    for ii in range(0, num_points, stepc):
        # print("ii", ii, directrix[ii,:])
        C = np.tile(directrix[ii, :], (num_arcs, 1)) + Rc[ii] * (
                    np.array([eN[ii, :], eB[ii, :], [0, 0, 0]]).T @ np.array([np.cos(2 * np.pi * tc),
                                                                                np.sin(2 * np.pi * tc),
                                                                                np.zeros(num_arcs)])).T
        # print("C shape", C.shape)   
        # print('C and its values', directrix[ii, :], C.shape, C)

        GC[k, :, :] = C
        k += 1

    return GC