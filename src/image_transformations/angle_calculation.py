import numpy as np

def calculate_angles(pt: np.ndarray):
    """Calculates the angles based on (x,y,z) coordinates

    Args:
        pt (np.ndarray): target cartesian points in the form of (x,y,z)

    Returns:
        float, float: y-axis angle, x-axis angle
    """
    
    pt = list(pt)
    assert len(pt) == 3

    x,y,z = pt
    xz = np.linalg.norm([x,z])
    
    ay = np.arctan2(x, z)
    ax = np.arctan2(y, xz)

    return np.rad2deg(ay), np.rad2deg(ax)