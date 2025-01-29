import numpy as np
from zmf import zmf


def risk_calculations(dcpa, tcpa, distance_ob, v_rel):
    """
    Calculate risk based on DCPA, TCPA, and Distance.

    Parameters:
    dcpa (array-like): Distance at Closest Point of Approach
    tcpa (array-like): Time to Closest Point of Approach
    distance_ob (array-like): Distance to obstacles
    v_rel (array-like): Relative velocity (not used in this function)

    Returns:
    numpy.array: Calculated risk for each obstacle
    """
    dcpa = np.asarray(dcpa)
    tcpa = np.asarray(tcpa)
    distance_ob = np.asarray(distance_ob)
    
    a_dcpa = 300.0
    b_dcpa = 1000.0

    a_tcpa = 60.0
    b_tcpa = 240.0

    a_dist = 300.0
    b_dist = 1000.0

    risk_dcpa = zmf(np.abs(dcpa), a_dcpa, b_dcpa)
    risk_tcpa = zmf(np.abs(tcpa), a_tcpa, b_tcpa)
    risk_dist = zmf(distance_ob, a_dist, b_dist)

    risk = (risk_dcpa + risk_tcpa + risk_dist) / 3

    return risk
