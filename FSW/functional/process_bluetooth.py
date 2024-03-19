import rospy
from rosardvarc.msg import UasToRgvDirectionVectorUasFrame, BluetoothAzimuthElevation
from ..config.topic_names import UAS_TO_RGV_DIRECTION_VECTORS, RAW_BLUETOOTH
from ..config.constants import BLUETOOTH_DCM

import math
import numpy as np 


def _bluetooth_callback(msg: BluetoothAzimuthElevation):
    # Calculates pointing vector in Bluetooth body frame
    x = math.cos(BluetoothAzimuthElevation[2]*math.pi/180)*math.cos(BluetoothAzimuthElevation[1]*math.pi/180)
    y = math.cos(BluetoothAzimuthElevation[2]*math.pi/180)*math.cos(BluetoothAzimuthElevation[1]*math.pi/180)
    z = math.sin(BluetoothAzimuthElevation[2]*math.pi/180)

    #TODO: convert calculated points to pointing vector (NEED Current attitude of BlueTooth Array in reference to UAS frame)

    #TODO: Create storage array that stores values (Might not need if Bluetooth frequency is 20)

    BluetoothToRgvDirectionVector = [[x,y,z]]

    # Rotation Matrix Call goes here Euler Angles = [180,90,0]

    UasToRgvDirectionVectorUasFrame = BluetoothToRgvDirectionVector
    
    rospy.logdebug("Bluetooth processor received raw bluetooth data")
    rospy.logdebug("Bluetooth processor published a direction vector")
    
    _direction_vector_pub.publish(UasToRgvDirectionVectorUasFrame)
    
    # publisher below seems much, unless someone had a syntax they wanted
    '''_direction_vector_pub.publish(
        UasToRgvDirectionVectorUasFrame(
            # TODO: Make this something reasonable

        )
    )'''


def setup():
    """
    Setup publishers and subscribers for process_bluetooth.py
    """
    
    global _direction_vector_pub, _bluetooth_sub
    
    _direction_vector_pub = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
    _bluetooth_sub = rospy.Subscriber(RAW_BLUETOOTH, BluetoothAzimuthElevation, _bluetooth_callback)


_direction_vector_pub: rospy.Publisher
_bluetooth_sub: rospy.Subscriber