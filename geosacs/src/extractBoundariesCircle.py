import numpy as np
from scipy.interpolate import splev, splrep

def extract_boundaries_circle(directrix, X1s, X2s, X3s):
    num_points = directrix.shape[0]
    num_traj = X1s.shape[1]

    neighborhood = num_points // 3

    # print("In boundary func: directrix", directrix.T.shape)
    directrix_transposed = directrix.T
    # G = np.gradient(directrix.T)
    G = np.gradient(directrix_transposed, axis=0)
    Gnorm = G / np.linalg.norm(G)

    # print("Gnorm shape", Gnorm.shape)

    Rc = np.zeros(num_points)

    tt = np.arange(1, num_points + 1)
    Idx1 = np.maximum(tt - neighborhood, 1)
    Idx2 = np.minimum(tt + neighborhood, len(tt))

    for i in range(num_points):
        dx, dy, dz = Gnorm[:, i]
        idx1, idx2 = Idx1[i], Idx2[i]

        point1 = directrix[i, :]
        distances = np.zeros(num_traj)

        for j in range(num_traj):
            # print("---------")
            # print("idx1", idx1)
            # print("idx2", idx2)
            # print("point1", point1)
            # print("x1s", X1s[idx1:idx2, j].shape)
            # print("dx:", dx)
            # min_func = dx * (X1s[idx1:idx2, j] - point1[0]) + \
            #            dy * (X2s[idx1:idx2, j] - point1[1]) + \
            #            dz * (X3s[idx1:idx2, j] - point1[2])
            # print("min_func shape:",min_func.shape)

            # idx3 = np.argmin(np.abs(min_func))
            # print("idx3", idx3)
            # point2 = np.array([X1s[idx1 - 1 + idx3, j], X2s[idx1 - 1 + idx3, j], X3s[idx1 - 1 + idx3, j]])
            # distances[j] = np.linalg.norm(point1 - point2)
            ##########
            # min_func = []
            # for i in range(idx1, idx2):
            #     dx1 = X1s[i,j] - point1[0]
            #     dx2 = X2s[i,j] - point1[1]
            #     dx3 = X3s[i,j] - point1[2]
            #     delta = np.array([[dx1, dx2, dx3]])
            #     min_func.append(np.linalg.norm(delta, axis=1))

            # idx3 = np.argmin(np.abs(min_func))
            # print("idx3", idx3)
            # point2 = np.array([X1s[idx1 - 1 + idx3, j], X2s[idx1 - 1 + idx3, j], X3s[idx1 - 1 + idx3, j]])
            # distances[j] = np.linalg.norm(point1 - point2)
            ##########
            point2 = np.array([X1s[i,j], X2s[i,j], X3s[i,j]])
            distances[j] = np.linalg.norm(point1 - point2)

        max_val = np.max(distances)
        Rc[i] = max_val

    ttt = np.arange(0, num_points, 10)

    tck = splrep(ttt, Rc[::10])
    Rc_smoothed = splev(np.arange(num_points), tck)

    threshold_percentage = 0.05
    threshold = threshold_percentage * np.mean([np.max(Rc), np.min(Rc)])
    Rc_smoothed += threshold

    return Rc_smoothed, threshold