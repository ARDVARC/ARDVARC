"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    File for guidance functions

Notes:
    - 
"""


def calc_orbit_setpoint(None) -> None:
    """ Calculates the orbit set point

    This function takes the RGV & UAS states, as well as time to calculate a 
    setpoint that can be given to the Pixhawk.

    Args:
        RGV: struct containing RGV state information
        UAS: struct containing UAS state information
        t: time elapsed since start of mission time

    Returns: 
        orbit_setpoint: Struct (TBR) containing orbit setpoint data

    Raises:
        None: Raises None at the moment (TBR)
    """


    pass
