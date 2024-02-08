import rospy
from rosardvarc.msg import UasToRgvDirectionVectorUasFrame, BluetoothAzimuthElevation
from ..config.topic_names import UAS_TO_RGV_DIRECTION_VECTORS, RAW_BLUETOOTH


def _bluetooth_callback(msg: BluetoothAzimuthElevation):
    # math math math
    
    _direction_vector_pub.publish(
        UasToRgvDirectionVectorUasFrame(
            # TODO: Make this something reasonable
        )
    )


_direction_vector_pub = rospy.Publisher(UAS_TO_RGV_DIRECTION_VECTORS, UasToRgvDirectionVectorUasFrame, queue_size=1)
_bluetooth_sub = rospy.Subscriber(RAW_BLUETOOTH, BluetoothAzimuthElevation, _bluetooth_callback)