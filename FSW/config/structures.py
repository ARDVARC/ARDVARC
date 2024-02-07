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
class MissionStates(Enum):
    FIND_RGV_1 = 0
    TRACK_RGV_1 = 1
    LOCALIZE_RGV_1 = 2
    FIND_RGV_2 = 3
    TRACK_RGV_2 = 4
    LOCALIZE_RGV_2 = 5
    JOINT_LOCALIZE = 6
    GO_HOME = 7