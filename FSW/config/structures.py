"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    Structs used by fsw

Notes:
    - 
"""

import numpy as np
import numpy.typing as npt
from enum import Enum
from dataclasses import dataclass
import time
import sys


# All possible states of Ardvarc automaton
class STATES(Enum):
    PIC = 1 # Pilot in command
    FIND = 2
    TRACK = 3
    LOCALIZE = 4
    JOINT = 5
    PANIC = 6

# Struct for UAS state
@dataclass
class UAS_t:

    # Physical State
    pos_org2uas_in_enu: np.ndarray
    vel_uas_in_enu: np.ndarray

    # Temporal State
    mission_time_start: float
    mission_time: float

    # Whether we've completed the mission
        # Start with neither having already been loacalized
    RGV_1_localized: bool = False
    RGV_2_localized: bool = False
    JOINT_localized: bool = False

    # Always default to PIC on initialization
    STATE: STATES = STATES.PIC
    

if __name__ == "__main__":

    UAS = UAS_t(np.array([1.,2.,3.]), np.array([.1,.2,.3]), 0.0, 0.0)
    print(UAS)
    print(sys.getsizeof(UAS))


