"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    Definition of states of main fsw state machine as enum

Notes:
    - Delete state that are unused by state machines.
"""

from enum import Enum

class STATES(Enum):
    TAKEOFF = 1
    TRACK = 2
