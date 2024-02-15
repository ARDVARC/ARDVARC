import unittest
from FSW.config.structures import MissionStates
from FSW.config.constants import LOCALIZE_DURATION
import rospy

class TestStateMachineCriteriaGenerator(unittest.TestCase):
    def test_finish_localizing_after_time(self):
        import FSW.functional.generate_state_machine_criteria as gsmc
        
        # Change module global variables so that they should result in
        # a message that says we are done localizing
        now = rospy.Time.from_sec(1000)
        gsmc._current_mission_state = MissionStates.LOCALIZE_RGV_1
        gsmc._current_mission_state_start_time = now - LOCALIZE_DURATION
                
        # Create the message
        msg = gsmc._build_state_machine_criteria_message(now)
        
        # Check that the message says that we are done localizing
        self.assertTrue(msg.rgv_1_localized)
    
    def test_dont_finish_localizing_early(self):
        import FSW.functional.generate_state_machine_criteria as gsmc
        
        # Change module global variables so that they should result in
        # a message that says we are not done localizing
        now = rospy.Time.from_sec(1000)
        gsmc._current_mission_state = MissionStates.LOCALIZE_RGV_1
        gsmc._current_mission_state_start_time = now - LOCALIZE_DURATION * 0.9
                
        # Create the message
        msg = gsmc._build_state_machine_criteria_message(now)
        
        # Check that the message does not say that we are done localizing
        self.assertFalse(msg.rgv_1_localized)
        
        