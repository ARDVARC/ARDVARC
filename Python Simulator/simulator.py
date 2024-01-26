from enum import IntEnum
import random
import socket
import struct
import sys
import time
from typing import NoReturn
import numpy as np
from scipy.spatial.transform import Rotation as R 
import asyncio

import rgv as RGV
import uasPhysics
import mavgen_common as mg

class SimulatorMessageKind(IntEnum):
    simulateUntil = 0
    measureRgv1 = 1
    measureRgv2 = 2
    measureUas = 3
    setComplexWaypoint = 4
    reset = 5

rgv1 = RGV.RGV(1, np.array([5,5]), np.pi/2, 900)
rgv2 = RGV.RGV(2, np.array([-5,-5]), -np.pi/2, 900)
global_lock = asyncio.Lock()
start_time = time.time()
def timeSinceStart() -> float:
    return time.time() - start_time
tPrev = 0
vec_uasStatePrev = np.array([-30,0,-10,0,0,0,0,0,0,0,0,0])
dcm_local_ned2uas = np.zeros((3,3))
complexWaypoint = uasPhysics.ComplexWaypoint(np.array([0,0,-10]),np.array([0,0,0]))

async def main():
    asyncio.create_task(physicsLoop())
    server = await asyncio.start_server(handle_client, "localhost", 1234)
    async with server:
        await server.serve_forever()

async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    mv = mg.MAVLink(writer, 1, mg.MAV_COMP_ID_AUTOPILOT1)
    temp = asyncio.create_task(heartbeatLoop(mv))
    message_loops = dict[int,asyncio.Task[NoReturn]]()
    while True:
        buffer = await reader.read(sys.maxsize)
        messages = mv.parse_buffer(buffer)
        if messages == None:
            continue
        for msg in messages:
            match msg:
                case mg.MAVLink_command_long_message():
                    match msg.command:
                        case mg.MAV_CMD_SET_MESSAGE_INTERVAL:
                            messageId = msg.param1
                            interval = msg.param2
                            match messageId:
                                case mg.MAVLINK_MSG_ID_LOCAL_POSITION_NED | mg.MAVLINK_MSG_ID_ATTITUDE:
                                    if messageId in message_loops:
                                        message_loops[messageId].cancel()
                                    match messageId:
                                        case mg.MAVLINK_MSG_ID_LOCAL_POSITION_NED:
                                            message_loops[messageId] = asyncio.create_task(localPositionLoop(mv, interval))
                                        case mg.MAVLINK_MSG_ID_ATTITUDE:
                                            message_loops[messageId] = asyncio.create_task(attitudeLoop(mv, interval))
                                case other:
                                    print(f"Simulator cannot set message interval for unrecognized message ID {other}")
                        case other:
                            print(f"Simulator does not recognize command ID {other}")
                case other:
                    print(f"Simulator does not recognize message {type(other)}")

def measureRgv(rgv: RGV.RGV, tPrev: float, vec_uasState: np.ndarray, dcm_local_ned2uas: np.ndarray) -> np.ndarray:
    _, vec_rgvPosition_local_ne = RGV.getRgvStateAtTime(rgv, tPrev)
    vec_pointingVecToRgv_local_ned = np.array([vec_rgvPosition_local_ne[0],vec_rgvPosition_local_ne[1],0]) - vec_uasState[0:3]
    vec_pointingVecToRgv_local_ned /= np.linalg.norm(vec_pointingVecToRgv_local_ned)
    
    # Get a specific vector that is orthogonal to the true
    # poining vector.
    vec_specificOrthogonal_local_ned = np.array([-vec_pointingVecToRgv_local_ned[1],vec_pointingVecToRgv_local_ned[0],0])
    vec_specificOrthogonal_local_ned /= np.linalg.norm(vec_specificOrthogonal_local_ned)
    # Generate a random rotation about the true pointing
    # vector.
    dcm_specificOrthogonal2randomOrthogonal = R.from_rotvec(vec_pointingVecToRgv_local_ned * (random.random()+1)*2*np.pi).as_matrix()
    # Rotate the specific orthogonal vector by the random
    # rotation to get a random orthogonal vector.
    vec_randomOrthogonal_local_ned = dcm_specificOrthogonal2randomOrthogonal@vec_specificOrthogonal_local_ned
    # Specify the axis and angle of rotation for the
    # estimated pointing vector. The axis is the random orthogonal vector
    # and the angle is random based on the pointing angle standard
    # deviation parameter. Then generate the rotation matrix corresponding
    # to the specified axis and angle.
    dcm_truePointingVec2sensorPointingVec = R.from_rotvec(vec_randomOrthogonal_local_ned * (random.normalvariate(0,np.deg2rad(0))+2*np.pi)).as_matrix()
    # Rotate the true pointing vector by the pointing
    # error rotation to get the sensor pointing vector.
    vec_sensorPointingVec_local_ned = dcm_truePointingVec2sensorPointingVec@vec_pointingVecToRgv_local_ned
    # Finally convert to the UAS frame
    vec_sensorPointingVec_uas = dcm_local_ned2uas@vec_sensorPointingVec_local_ned
    return vec_sensorPointingVec_uas

async def physicsLoop():
    global global_lock, tPrev, vec_uasStatePrev, dcm_local_ned2uas, complexWaypoint
    DT = 1/(2**8)
    while True:
        sleeper = asyncio.create_task(asyncio.sleep(tPrev - timeSinceStart() + DT))
        sol = uasPhysics.simulateUasPhysics((tPrev, tPrev + DT), vec_uasStatePrev, complexWaypoint)
        temp_dcm_local_ned2uas = R.from_euler("xyz", vec_uasStatePrev[3:6]).as_matrix().transpose()
        if sleeper.done():
            print("WARNING: Simulation is going slower than real time")
        await sleeper
        async with global_lock:
            tPrev += DT
            vec_uasStatePrev = sol.y[:,-1]
            dcm_local_ned2uas = temp_dcm_local_ned2uas

async def heartbeatLoop(mv: mg.MAVLink):
    heartbeat_last_sent = timeSinceStart()
    DT = 1
    while True:
        await asyncio.create_task(asyncio.sleep(heartbeat_last_sent - timeSinceStart() + DT))
        mv.heartbeat_send(mg.MAV_TYPE_HEXAROTOR, mg.MAV_AUTOPILOT_GENERIC, mg.MAV_MODE_FLAG_HIL_ENABLED, 0, mg.MAV_STATE_ACTIVE)
        heartbeat_last_sent += DT

async def localPositionLoop(mv: mg.MAVLink, interval: float):
    global global_lock, tPrev, vec_uasStatePrev, dcm_local_ned2uas
    local_position_last_sent = timeSinceStart()
    interval /= 1e6
    while True:
        await asyncio.create_task(asyncio.sleep(local_position_last_sent - timeSinceStart() + interval))
        async with global_lock:
            t = tPrev
            vec_uasStatePrev_copy = vec_uasStatePrev.copy()
            dcm_local_ned2uas_copy = dcm_local_ned2uas.copy()
        dcm_uas2local_ned = dcm_local_ned2uas_copy.transpose()
        vec_vel_local_ned = dcm_uas2local_ned.transpose()@vec_uasStatePrev_copy[6:9]
        mv.local_position_ned_send(int(t*1e3), vec_uasStatePrev_copy[0], vec_uasStatePrev_copy[1], vec_uasStatePrev_copy[2], vec_vel_local_ned[0], vec_vel_local_ned[1], vec_vel_local_ned[2])
        local_position_last_sent += interval

async def attitudeLoop(mv: mg.MAVLink, interval: float):
    global global_lock, tPrev, vec_uasStatePrev, dcm_local_ned2uas
    attitude_last_sent = timeSinceStart()
    interval /= 1e6
    while True:
        await asyncio.create_task(asyncio.sleep(attitude_last_sent - timeSinceStart() + interval))
        async with global_lock:
            t = tPrev
            vec_uasStatePrev_copy = vec_uasStatePrev.copy()
            dcm_local_ned2uas_copy = dcm_local_ned2uas.copy()
        dcm_uas2local_ned = dcm_local_ned2uas_copy.transpose()
        vec_ang_vel_local_ned = dcm_uas2local_ned.transpose()@vec_uasStatePrev_copy[9:12]
        mv.attitude_send(int(t*1e3), vec_uasStatePrev_copy[3], vec_uasStatePrev_copy[4], vec_uasStatePrev_copy[5], vec_ang_vel_local_ned[0], vec_ang_vel_local_ned[1], vec_ang_vel_local_ned[2])
        attitude_last_sent += interval

if __name__ == "__main__":
    asyncio.run(main())