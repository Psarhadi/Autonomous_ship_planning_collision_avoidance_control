import numpy as np
from scipy.special import expit  # For the sigmoid function
from zmf import zmf


def reactive_avoidance(x_ob, y_ob, x, y, psi, t):
    """
    Perform reactive avoidance.

    Parameters:
    x_ob, y_ob (array-like): Positions of obstacles
    x, y (float): Current position of the vessel
    psi (float): Current heading of the vessel
    t (float): Current time (not used in this function)

    Returns:
    tuple: (psi_oa, w_b, w_r, distance_ob, bearing_ob)
        psi_oa (float): Avoidance heading
        w_b (numpy.array): Bearing weights
        w_r (numpy.array): Range weights
        distance_ob (numpy.array): Distances to obstacles
        bearing_ob (numpy.array): Bearings to obstacles
    """
    a = 500
    b = 1000

    x_distro = np.arange(-90, 90.1, 0.1)
    c = 0
    sig = 80 * np.pi / 180

    distance_ob = np.sqrt((np.array(x_ob) - x)**2 + (np.array(y_ob) - y)**2)
    los_ob = np.arctan2((np.array(y_ob) - y), (np.array(x_ob) - x))

    bearing_ob = psi - los_ob
    w_r = zmf(distance_ob, a, b)
    w_b = -np.exp(-(bearing_ob**2) / (2 * sig**2))

    psi_oa = np.sum(w_r * w_b) * 2

    return psi_oa, w_b, w_r, distance_ob, bearing_ob
