"""
ARDVARC Flight Software

Author: Lyon Foster
email: lyon.foster@colorado.edu

Description:
    main state machine for all fsw

Notes:
    - Some things we NEVER want to do:
     + quit the program
     + do noting unintentionally
     + error out and quit the program
     + get stuck in a loop
"""

# local imports
from config.states import STATES

# third part imports
import numpy as np

# stl imports
import time



"""Assignment of first state"""
main_state = STATES.TAKEOFF

"""Assignment of mission start time"""
mission_start_time = time.time()

while(True):

    # TODO(LF): review state machine structure and design
    # ! Notional state machine example, not for flight use

    # Stuff that always happens
    current_uplink_msg = read_uplink_data()

    if main_state == STATES.TAKEOFF:

        is_PIC = current_uplink_msg.is_PIC

        if (height >= mission_heights and !PIC):
            main_state = STATES.TRACK

    if main_state == STATES.TRACK:
        pass
            
