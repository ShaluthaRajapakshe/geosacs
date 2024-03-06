import numpy as np
from scipy.interpolate import CubicSpline, interp1d

def get_T(directrix, init=None):
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

    v = np.arange(1, sz + 1)
    X = CubicSpline(v, x, bc_type='natural')
    Y = CubicSpline(v, y, bc_type='natural')
    Z = CubicSpline(v, z, bc_type='natural') if dim == 3 else None


    mx = X(v, 1)
    my = Y(v, 1)
    mz = Z(v, 1) if dim == 3 else None

    ind = np.where(np.sqrt(mx**2 + my**2 + mz**2) > 0)

    data = np.column_stack((mx[ind], my[ind], mz[ind])) if dim == 3 else np.column_stack((mx[ind], my[ind]))

    T = data / np.sqrt(np.sum(data**2, axis=1))[:, None]


    ## Interpolate T
    query_points = np.arange(1, sz + 1)
    interpolated_values_T = np.zeros((query_points.size, T.shape[1]))

    print("T new size", interpolated_values_T.shape)

    for col_idx in range(T.shape[1]):
        f = interp1d(ind[0], T[:, col_idx], kind='linear', fill_value='extrapolate')
        interpolated_values_T[:, col_idx] = f(query_points)

    T = interpolated_values_T
    T /= np.sqrt(np.sum(T**2, axis=1))[:, None]

    return T
