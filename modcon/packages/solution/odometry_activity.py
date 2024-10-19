from typing import Tuple

import numpy as np


def delta_phi(ticks: int, prev_ticks: int, resolution: int) -> Tuple[float, float]:
    """
    Args:
        ticks: Current tick count from the encoders.
        prev_ticks: Previous tick count from the encoders.
        resolution: Number of ticks per full wheel rotation returned by the encoder.
    Return:
        dphi: Rotation of the wheel in radians.
        ticks: current number of ticks.
    """
    ticks = ticks - prev_ticks # delta ticks 
    dphi = ticks * resolution # total rotation of wheel 
    return dphi, ticks


def pose_estimation(
    R: float,
    baseline: float,
    x_prev: float,
    y_prev: float,
    theta_prev: float,
    delta_phi_left: float,
    delta_phi_right: float,
) -> Tuple[float, float, float]:

    """
    Calculate the current Duckiebot pose using the dead-reckoning model.

    Args:
        R:                  radius of wheel (both wheels are assumed to have the same size) - this is fixed in simulation,
                            and will be imported from your saved calibration for the real robot
        baseline:           distance from wheel to wheel; 2L of the theory
        x_prev:             previous x estimate - assume given
        y_prev:             previous y estimate - assume given
        theta_prev:         previous orientation estimate - assume given
        delta_phi_left:     left wheel rotation (rad)
        delta_phi_right:    right wheel rotation (rad)

    Return:
        x_curr:                  estimated x coordinate
        y_curr:                  estimated y coordinate
        theta_curr:              estimated heading
    """

    # Distance travelled by each wheel
    d_right = (delta_phi_right/(2 * np.deg2rad(np.pi))) * R
    d_left = (delta_phi_left/(2 * np.deg2rad(np.pi))) * R

    # Traveled distance of robot 
    delta_A = (d_right + d_left) / 2 

    # Angle rotated of robot
    delta_Theta = (d_right - d_left) / (2 * baseline) # [radians]
    
    # New pose of robot
    x_curr = x_prev + delta_A * np.cos(theta_prev)
    y_curr = y_prev + delta_A * np.sin(theta_prev)
    theta_curr = (theta_prev + delta_Theta) #% np.deg2rad(2 * np.pi)  
    # ---
    return x_curr, y_curr, theta_curr
