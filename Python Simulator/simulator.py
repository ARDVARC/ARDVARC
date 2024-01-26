from enum import IntEnum
import random
import socket
import struct
import numpy as np
from scipy.spatial.transform import Rotation as R 
import asyncio
import mavgen_common as mg

import rgv as RGV
import uasPhysics

class SimulatorMessageKind(IntEnum):
    simulateUntil = 0
    measureRgv1 = 1
    measureRgv2 = 2
    measureUas = 3
    setComplexWaypoint = 4
    reset = 5

async def main():
    # Run simulation as commanded
    server = await asyncio.start_server(handle_client, "localhost", 1234)
    async with server:
        await server.serve_forever()

async def handle_client(reader: asyncio.StreamReader, writer: asyncio.StreamWriter):
    # Initialize simulation
    rgv1 = RGV.RGV(1, np.array([5,5]), np.pi/2, 900)
    rgv2 = RGV.RGV(2, np.array([-5,-5]), -np.pi/2, 900)
    tPrev = 0
    vec_uasStatePrev = np.array([-30,0,-10,0,0,0,0,0,0,0,0,0])
    dcm_local_ned2uas = np.zeros((3,3))
    complexWaypoint = uasPhysics.ComplexWaypoint(np.array([0,0,10]),np.array([0,0,0]))
    
    while True:
        data = await reader.read(1)
        messageKind = SimulatorMessageKind(data[0])
        # print(messageKind)
        match messageKind:
            case SimulatorMessageKind.simulateUntil:
                t: float = struct.unpack("!d", await reader.read(8))[0]
                if t <= tPrev:
                    raise Exception(f"Asked to simulate until invalid time {t} (tPrev: {tPrev}).")
                sol = uasPhysics.simulateUasPhysics((tPrev, t), vec_uasStatePrev, complexWaypoint)
                tPrev = t
                vec_uasStatePrev = sol.y[:,-1]
                dcm_local_ned2uas = R.from_euler("xyz", vec_uasStatePrev[3:6]).as_matrix().transpose()
                
                writer.write(bytearray([6]))
            case SimulatorMessageKind.measureRgv1:
                vec_sensorPointingVec_uas = measureRgv(rgv1, tPrev, vec_uasStatePrev, dcm_local_ned2uas)
                writer.write(struct.pack("!ddd", vec_sensorPointingVec_uas[0], vec_sensorPointingVec_uas[1], vec_sensorPointingVec_uas[2]))
            case SimulatorMessageKind.measureRgv2:
                vec_sensorPointingVec_uas = measureRgv(rgv2, tPrev, vec_uasStatePrev, dcm_local_ned2uas)
                writer.write(struct.pack("!ddd", vec_sensorPointingVec_uas[0], vec_sensorPointingVec_uas[1], vec_sensorPointingVec_uas[2]))
            case SimulatorMessageKind.measureUas:
                # mg.MAVLink.local_position_ned_encode(self, tPrev, )
                writer.write(struct.pack("!dddddddddddd", vec_uasStatePrev[0], vec_uasStatePrev[1], vec_uasStatePrev[2], vec_uasStatePrev[3], vec_uasStatePrev[4], vec_uasStatePrev[5], vec_uasStatePrev[6], vec_uasStatePrev[7], vec_uasStatePrev[8], vec_uasStatePrev[9], vec_uasStatePrev[10], vec_uasStatePrev[11]))
            case SimulatorMessageKind.setComplexWaypoint:
                vals = struct.unpack("!dddddd", await reader.read(48))
                complexWaypoint = uasPhysics.ComplexWaypoint(np.array([vals[0],vals[1],vals[2]]), np.array([vals[3],vals[4],vals[5]]))
                writer.write(bytearray([6]))
            case SimulatorMessageKind.reset:
                tPrev = 0
                vec_uasStatePrev = np.array([-30,0,-10,0,0,0,0,0,0,0,0,0])
                complexWaypoint = uasPhysics.ComplexWaypoint(np.array([0,0,10]),np.array([0,0,0]))
                writer.write(bytearray([6]))

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

if __name__ == "__main__":
    asyncio.run(main())