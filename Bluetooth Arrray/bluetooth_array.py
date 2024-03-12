#libraries for the serial port read
import serial
import numpy as np
#libraries i thought ros needs
import rospy
from std_msgs.msg import Byte
from std_msgs.msg import Int8

#SERIAL PORT READ LINE FUNCTION
def read_lines(s, sep=b"\r\n\r\n"): #/r/n/r/n: carriage return, readline, carriage return, readline (what the array outputs to denote new line)
    buffer = b"" #initialise buffer to store line from serial port
    while True:
        buffer += s.read() #read only when UTF-8 character is available on the port
        while sep in buffer: #triggers when /r/n/r/n is detected
            line, _, buffer = buffer.partition(sep) #store buffer in variable line, variable line changes every time new line is complete
            yield line

#THIS IS ME TRYING TO DEFINE THE ROS FUNCTION
def bluetooth_array():
   7     pub = rospy.Publisher('bluetooth', Byte, queue_size=10)
   8     rospy.init_node('array', anonymous=True)
   9     rate = rospy.Rate(20) # 10hz
  10     while not rospy.is_shutdown():
  11         hello_str = "hello world %s" % rospy.get_time()
  12         rospy.loginfo(hello_str)
  13         pub.publish(hello_str)
  14         rate.sleep()

#Call up serial port
s = serial.Serial("/dev/ttyUSB0", 115200) #CHANGE TO COMXX PORT FOR WINDOWS, baud rate stays the same

for line in read_lines(s): #run every time a new line is detected
    decoded = line.decode('utf-8') #decode bytes into characters
    splitted = decoded.split(',') #split string by comma character
    if line[6:9] == b'D77': #Read ID of beacon 1
        b_id = np.ubyte(1) #Save 1 to b_id for beacon 1
    else: #NEED TO WRITE ELSE STATEMENT FOR SECOND BEACON
        b_id = np.ubyte(2) #Save 2 to b_id for beacon 2
    ang1 = np.byte(splitted[2]) #Parse angle 1
    ang2 = np.byte(splitted[3]) #Parse angle 2
    #tstamp = np.
    #print(b_id, ang1, ang2)
