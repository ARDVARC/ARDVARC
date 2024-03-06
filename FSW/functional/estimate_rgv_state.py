import rospy
import collections
import math
import numpy as np
from rosardvarc.msg import EstimatedRgvState, UasToRgvDirectionVectorUasFrame
from ..config.topic_names import ESTIMATED_RGV_STATES, UAS_POSES, UAS_TO_RGV_DIRECTION_VECTORS
from geometry_msgs.msg import PoseStamped
# import navpy

# TODO import previous rgv and uas states for ismoving


_uas_state_buffer = collections.deque([], 50)
_direction_vector_buffer = collections.deque([], 50)


def _uas_state_callback(msg: PoseStamped):
    _uas_state_buffer.appendleft(msg)
    rospy.loginfo("RGV state estimator saved UAS state")


def _direction_vector_callback(msg: UasToRgvDirectionVectorUasFrame):
    _direction_vector_buffer.appendleft(msg)
    rospy.loginfo("RGV state estimator saved direction vector")


_estimated_rgv_state_pub = rospy.Publisher(ESTIMATED_RGV_STATES, EstimatedRgvState, queue_size=1)
_uas_state_sub = rospy.Subscriber(UAS_POSES, PoseStamped, _uas_state_callback)
_direction_vector_sub = rospy.Subscriber(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, _direction_vector_callback)

def find_sensor_pos(sensor_angle, uas_to_rgv_az, uas_to_rgv_el, uas_pos, uas_yaw):
    # Find rectangular coords
    # Vector length from uas to rgv
    uas_height = uas_pos[2]
    uas_pos_x = uas_pos[0]
    uas_pos_y = uas_pos[1]
    rho = uas_height/math.cos(uas_to_rgv_el)
    # Angle math for relative locations in inertial coords
    uas_to_rgv_x = rho*math.sin(uas_to_rgv_az - uas_yaw)*math.cos(uas_to_rgv_el - sensor_angle)
    uas_to_rgv_y = -rho*math.sin(uas_to_rgv_az - uas_yaw)*math.sin(uas_to_rgv_el - sensor_angle)
    # Convert relative to absolute
    rgv_state_x = uas_pos_x + uas_to_rgv_x
    rgv_state_y = uas_pos_y + uas_to_rgv_y

    rgv_state = [rgv_state_x, rgv_state_y, 0]
    return rgv_state

def estimate_rgv_state():
    """
    Called by main.py in the main loop.
    Estimates and publishes the RGV state.
    """

    # Estimate the RGV state

    # Sensor angles relative to down z vector
    # Hardcode for now 
    bluetooth_angle = 0 
    camera_angle = 85*np.pi/180
    # Is timestamp old?
    old_metric = 0.1
    # for now say that the UAS is never moving - import UAS state at some point
    uas_moving = False

    # import variables
    uas_to_rgv1_az = UasToRgvDirectionVectorUasFrame.bluetooth.rgv1.angle1
    uas_to_rgv1_el = UasToRgvDirectionVectorUasFrame.bluetooth.rgv1.angle2
    uas_to_rgv2_az = UasToRgvDirectionVectorUasFrame.bluetooth.rgv2.angle1
    uas_to_rgv2_el = UasToRgvDirectionVectorUasFrame.bluetooth.rgv2.angle2

    uas_to_rgv1_az_cam = UasToRgvDirectionVectorUasFrame.camera.rgv1.angle1
    uas_to_rgv1_el_cam = UasToRgvDirectionVectorUasFrame.camera.rgv1.angle2
    uas_to_rgv2_az_cam = UasToRgvDirectionVectorUasFrame.camera.rgv2.angle1
    uas_to_rgv2_el_cam = UasToRgvDirectionVectorUasFrame.camera.rgv2.angle2

    # TEMP -- PULL IN UAS POSITION AND ATT
    uas_pos = [0,0,10]
    uas_attitude = [0,0,0]
    uas_yaw = uas_attitude[2]

    # Define if bluetooth and camera signals are new or not
    if UasToRgvDirectionVectorUasFrame.bluetooth.rgv1_timestamp <= old_metric:
        bluetooth_new_rgv1 = True
    else:
        bluetooth_new_rgv1 = False

    if UasToRgvDirectionVectorUasFrame.bluetooth.rgv2_timestamp <= old_metric:
        bluetooth_new_rgv2 = True
    else:
        bluetooth_new_rgv2 = False

    if UasToRgvDirectionVectorUasFrame.camera.rgv1_timestamp <= old_metric:
        camera_new_rgv1 = True
    else:
        camera_new_rgv1 = False

    if UasToRgvDirectionVectorUasFrame.camera.rgv2_timestamp <= old_metric:
        camera_new_rgv2 = True
    else:
        camera_new_rgv2 = False
    
    # find positions
    if bluetooth_new_rgv1:
        bluetooth_pos_rgv1 = find_sensor_pos(bluetooth_angle, uas_to_rgv1_az, uas_to_rgv1_el, uas_pos, uas_yaw)
    if bluetooth_new_rgv2:
        bluetooth_pos_rgv2 = find_sensor_pos(bluetooth_angle, uas_to_rgv2_az, uas_to_rgv2_el, uas_pos, uas_yaw)
    if camera_new_rgv1:
        camera_pos_rgv1 = find_sensor_pos(camera_angle, uas_to_rgv1_az_cam, uas_to_rgv1_el_cam, uas_pos, uas_yaw)
    if camera_new_rgv2:
        camera_pos_rgv2 = find_sensor_pos(camera_angle, uas_to_rgv2_az_cam, uas_to_rgv2_el_cam, uas_pos, uas_yaw)
        camera_pos_rgv1 = find_sensor_pos(camera_angle, uas_to_rgv1_az, uas_to_rgv1_el, uas_pos, uas_yaw)
    if camera_new_rgv2:
        camera_pos_rgv2 = find_sensor_pos(bluetooth_angle, uas_to_rgv2_az, uas_to_rgv2_el, uas_pos, uas_yaw)
    
    # find final position
    rgv1_position_x =  bluetooth_pos_rgv1[0] + camera_pos_rgv1[0]   
    rgv1_position_y =  bluetooth_pos_rgv1[1] + camera_pos_rgv1[1]             
    rgv1_position = [rgv1_position_x, rgv1_position_y, 0]
    rgv2_position_x =  bluetooth_pos_rgv2[0] + camera_pos_rgv2[0]   
    rgv2_position_y =  bluetooth_pos_rgv2[1] + camera_pos_rgv2[1]             
    rgv2_position = [rgv2_position_x, rgv2_position_y, 0]

    # find if moving or not
    if np.linalg.norm(rgv1_position - prev_rgv1_position) >= 0:
        rgv1_moving = True
    else:
        rgv1_moving = False
    if np.linalg.norm(rgv2_position - prev_rgv2_position) >= 0:
        rgv2_moving = True
    else:
        rgv2_moving = False

    # Determine confidence value brute force for now
    if not uas_moving and bluetooth_new_rgv1 and camera_new_rgv1:
        rgv1_confidence = 1
    if bluetooth_new_rgv1 and camera_new_rgv1:
        rgv1_confidence = 0.75
    if bluetooth_new_rgv1:
        rgv1_confidence = 0.25
    else:
        rgv1_confidence = 0  


    if not uas_moving and bluetooth_new_rgv2 and camera_new_rgv2:
        rgv2_confidence = 1
    if bluetooth_new_rgv2 and camera_new_rgv2:
        rgv2_confidence = 0.75
    if bluetooth_new_rgv2:
        rgv2_confidence = 0.25
    else:
        rgv2_confidence = 0   
    
    # assemble output
    EstimatedRgvState.rgv1_position = rgv1_position
    EstimatedRgvState.rgv2_position = rgv2_position
    EstimatedRgvState.rgv1_moving = rgv1_moving
    EstimatedRgvState.rgv2_moving = rgv2_moving
    EstimatedRgvState.rgv1_confidence = rgv1_confidence
    EstimatedRgvState.rgv2_confidence = rgv2_confidence

    # Publish estimate
    rospy.loginfo("RGV state estimator published an RGV state estimate")
    _estimated_rgv_state_pub.publish(EstimatedRgvState)
