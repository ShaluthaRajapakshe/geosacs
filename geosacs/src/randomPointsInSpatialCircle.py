import numpy as np

def randomPointsInSpatialCircle(n, r, C, eN, eB):
    rr = -r + 2 * r * np.random.rand(n, 1)  # Get random points on diameter
    tr = np.random.rand(n, 1) * 2 * np.pi   # Get random angles
    P0 = rr * np.cos(tr) * eN + rr * np.sin(tr) * eB  # Calculate points inside the circle
    Ratio = np.sqrt(np.sum(P0**2, axis=1)) / r  # Calculate the ratio
    Pout = P0 + np.tile(C, (n, 1))  # Translate to the directrix
    return Pout, Ratio