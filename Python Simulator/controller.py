import asyncio
import socket
import struct
import sys
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R 

import simulator
import mavgen_common as mg

async def main():
    reader, writer = await asyncio.open_connection("localhost", 1234)
    mv = mg.MAVLink(writer, 1, mg.MAV_COMP_ID_ONBOARD_COMPUTER)
    # Send requests 
    mv.command_long_send(1, mg.MAV_COMP_ID_AUTOPILOT1, mg.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mg.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 1e6, 0, 0, 0, 0, 0)
    mv.command_long_send(1, mg.MAV_COMP_ID_AUTOPILOT1, mg.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mg.MAVLINK_MSG_ID_ATTITUDE, 1e6, 0, 0, 0, 0, 0)
    xs = []
    ys = []
    timer = asyncio.create_task(asyncio.sleep(30))
    while not timer.done():
        buffer = await reader.read(sys.maxsize)
        messages = mv.parse_buffer(buffer)
        if messages == None:
            continue
        for msg in messages:
            match msg:
                case mg.MAVLink_local_position_ned_message():
                    xs.append(msg.x)
                    ys.append(msg.y)
    plt.scatter(xs, ys, s=1, c="green")
    plt.axis('equal')
    plt.grid(which='both')
    plt.show()

if __name__ == "__main__":
    asyncio.run(main())