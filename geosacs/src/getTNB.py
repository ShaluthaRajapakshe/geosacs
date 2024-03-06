import numpy as np
from scipy.interpolate import CubicSpline, interp1d

def get_TNB(directrix, init=None):
    x = directrix[:, 0]
    y = directrix[:, 1]
    x = x.ravel()
    y = y.ravel()
    sz = len(x)
    if directrix.shape[1] == 3:
        dim = 3
        z = directrix[:, 2]
        z = z.ravel()
    else:
        dim = 2


    # print('x,y, z shapes', x.shape, y.shape, z.shape)

    v = np.arange(1, sz + 1)
    X = CubicSpline(v, x, bc_type='natural')
    Y = CubicSpline(v, y, bc_type='natural')
    Z = CubicSpline(v, z, bc_type='natural') if dim == 3 else None


    mx = X(v, 1)
    my = Y(v, 1)
    mz = Z(v, 1) if dim == 3 else None

    # print('mx,my, mz shapes', mx.shape, my.shape, mz.shape)

    ind = np.where(np.sqrt(mx**2 + my**2 + mz**2) > 0)

    # print('ind[0]', ind[0])
    data = np.column_stack((mx[ind], my[ind], mz[ind])) if dim == 3 else np.column_stack((mx[ind], my[ind]))

    T = data / np.sqrt(np.sum(data**2, axis=1))[:, None]

    # print('Tangent', T.shape, T[:,0].shape)

    s = len(ind[0])
    N = np.zeros((s + 1, 3))
    B = np.zeros((s, 3))

    if init is None:
        N[0] = [T[0, 2]*T[0, 0], T[0, 2]*T[0, 1], -(T[0, 0]**2 + T[0, 1]**2)]
        if np.all(N[0] == 0) or np.all(np.isnan(N[0])):
            N[0] = [1, 0, 0]
    else:
        N[0] = init

    N[0] /= np.linalg.norm(N[0])

    for m in range(s):
        B[m] = np.cross(N[m], T[m])
        N[m + 1] = np.cross(T[m], B[m])

    N = np.delete(N, 0, axis=0)

    # # T = np.interp(np.arange(1, sz + 1), ind[0], T, axis=0)
    # T = np.interp(np.arange(1, sz + 1), ind[0], T)
    # # B = np.interp(np.arange(1, sz + 1), ind[0], B, axis=0)
    # B = np.interp(np.arange(1, sz + 1), ind[0], B)
    # # N = np.interp(np.arange(1, sz + 1), ind[0], N, axis=0)
    # N = np.interp(np.arange(1, sz + 1), ind[0], N)

    ### Interpolation code ####


    ## Interpolate T
    query_points = np.arange(1, sz + 1)
    interpolated_values_T = np.zeros((query_points.size, T.shape[1]))

    # print("T new size", interpolated_values_T.shape)

    for col_idx in range(T.shape[1]):
        f = interp1d(ind[0], T[:, col_idx], kind='linear', fill_value='extrapolate')
        interpolated_values_T[:, col_idx] = f(query_points)

    T = interpolated_values_T

    #########

    ## Interpolate B 
    interpolated_values_B = np.zeros((query_points.size, B.shape[1]))

    # print("B new size", interpolated_values_B.shape)  

    for col_idx in range(B.shape[1]):
        f = interp1d(ind[0], B[:, col_idx], kind='linear', fill_value='extrapolate')
        interpolated_values_B[:, col_idx] = f(query_points)

    B = interpolated_values_B

    #########

    ## Interpolate N
    interpolated_values_N = np.zeros((query_points.size, N.shape[1]))

    # print("N new size", interpolated_values_N.shape)

    for col_idx in range(N.shape[1]):
        f = interp1d(ind[0], N[:, col_idx], kind='linear', fill_value='extrapolate')
        interpolated_values_N[:, col_idx] = f(query_points)
    
    N = interpolated_values_N

    #########


    T /= np.sqrt(np.sum(T**2, axis=1))[:, None]
    B /= np.sqrt(np.sum(B**2, axis=1))[:, None]
    N /= np.sqrt(np.sum(N**2, axis=1))[:, None]

    if dim == 2:
        T = np.delete(T, 2, axis=1)
        N = np.delete(N, 2, axis=1)
        B = np.delete(B, 2, axis=1)

    return T, N, B