from enum import IntEnum
import random
import socket
import struct
import numpy as np
from scipy.spatial.transform import Rotation as R 

import rgv as RGV
import uasPhysics

class SimulatorMessageKind(IntEnum):
    simulateUntil = 0
    measureRgv1 = 1
    measureRgv2 = 2
    measureUas = 3
    setComplexWaypoint = 4
    reset = 5

def main():
    # Initialize simulation
    rgv1 = RGV.RGV(1, np.array([5,5]), np.pi/2, 900)
    rgv2 = RGV.RGV(2, np.array([-5,-5]), -np.pi/2, 900)
    tPrev = 0
    vec_uasStatePrev = np.array([-30,0,0,0,0,0,0,0,0,0,0,0])
    complexWaypoint = uasPhysics.ComplexWaypoint(np.array([0,0,10]),np.array([0,0,0]))
    
    # Run simulation as commanded
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.bind(("localhost", 1234))
        s.listen()
        connection, _ = s.accept()
        with connection:
            while True:
                messageKind = SimulatorMessageKind(connection.recv(1)[0])
                # print(messageKind)
                match messageKind:
                    case SimulatorMessageKind.simulateUntil:
                        t: float = struct.unpack("!d", connection.recv(8))[0]
                        if t <= tPrev:
                            raise Exception(f"Asked to simulate until invalid time {t} (tPrev: {tPrev}).")
                        sol = uasPhysics.simulateUasPhysics((tPrev, t), vec_uasStatePrev, complexWaypoint)
                        vec_uasStatePrev = sol.y[:,-1]
                        tPrev = t
                        connection.send(bytearray([6]))
                    case SimulatorMessageKind.measureRgv1:
                        vec_sensorPointingVec_uas = measureRgv(rgv1, tPrev, vec_uasStatePrev)
                        connection.send(struct.pack("!ddd", vec_sensorPointingVec_uas[0], vec_sensorPointingVec_uas[1], vec_sensorPointingVec_uas[2]))
                    case SimulatorMessageKind.measureRgv2:
                        vec_sensorPointingVec_uas = measureRgv(rgv2, tPrev, vec_uasStatePrev)
                        connection.send(struct.pack("!ddd", vec_sensorPointingVec_uas[0], vec_sensorPointingVec_uas[1], vec_sensorPointingVec_uas[2]))
                    case SimulatorMessageKind.measureUas:
                        connection.send(struct.pack("!dddddddddddd", vec_uasStatePrev[0], vec_uasStatePrev[1], vec_uasStatePrev[2], vec_uasStatePrev[3], vec_uasStatePrev[4], vec_uasStatePrev[5], vec_uasStatePrev[6], vec_uasStatePrev[7], vec_uasStatePrev[8], vec_uasStatePrev[9], vec_uasStatePrev[10], vec_uasStatePrev[11]))
                    case SimulatorMessageKind.setComplexWaypoint:
                        vals = struct.unpack("!dddddd", connection.recv(48))
                        complexWaypoint = uasPhysics.ComplexWaypoint(np.array([vals[0],vals[1],vals[2]]), np.array([vals[3],vals[4],vals[5]]))
                        connection.send(bytearray([6]))
                    case SimulatorMessageKind.reset:
                        tPrev = 0
                        vec_uasStatePrev = np.array([-30,0,0,0,0,0,0,0,0,0,0,0])
                        complexWaypoint = uasPhysics.ComplexWaypoint(np.array([0,0,10]),np.array([0,0,0]))
                        connection.send(bytearray([6]))

def measureRgv(rgv: RGV.RGV, tPrev: float, vec_uasState: np.ndarray) -> np.ndarray:
    vec_uasPos_local = vec_uasState[0:3]
    phi = vec_uasState[3]
    theta = vec_uasState[4]
    psi = vec_uasState[5]
    
    sinphi = np.sin(phi)
    cosphi = np.cos(phi)
    sintheta = np.sin(theta)
    costheta = np.cos(theta)
    sinpsi = np.sin(psi)
    cospsi = np.cos(psi)
    
    _, vec_rgvPosition_localGround = RGV.getRgvStateAtTime(rgv, tPrev)
    vec_pointingVecToRgv_local = np.array([vec_rgvPosition_localGround[0],vec_rgvPosition_localGround[1],0]) - vec_uasPos_local
    vec_pointingVecToRgv_local /= np.linalg.norm(vec_pointingVecToRgv_local)
    
    dcm_local2uas = np.array([
        [costheta*cospsi                     ,costheta*sinpsi                     ,-sintheta      ],
        [sinphi*sintheta*cospsi-cosphi*sinpsi,sinphi*sintheta*sinpsi+cosphi*cospsi,sinphi*costheta],
        [cosphi*sintheta*cospsi+sinphi*sinpsi,cosphi*sintheta*sinpsi-sinphi*cospsi,cosphi*costheta]
    ])
    
    # Get a specific vector that is orthogonal to the true
    # poining vector.
    vec_specificOrthogonal_local = np.array([-vec_pointingVecToRgv_local[1],vec_pointingVecToRgv_local[0],0])
    vec_specificOrthogonal_local /= np.linalg.norm(vec_specificOrthogonal_local)
    # Generate a random rotation about the true pointing
    # vector.
    dcm_specificOrthogonal2randomOrthogonal = R.from_rotvec(vec_pointingVecToRgv_local * (random.random()+1)*2*np.pi).as_matrix()
    # Rotate the specific orthogonal vector by the random
    # rotation to get a random orthogonal vector. This is easy because the
    # specific orthogonal vector is already stored in a matrix.
    vec_randomOrthogonal_local = dcm_specificOrthogonal2randomOrthogonal@vec_specificOrthogonal_local
    # Specify the axis and angle of rotation for the
    # estimated pointing vector. The axis is the random orthogonal vector
    # and the angle is random based on the pointing angle standard
    # deviation parameter. Then generate the rotation matrix corresponding
    # to the specified axis and angle.
    dcm_truePointingVec2sensorPointingVec = R.from_rotvec(vec_randomOrthogonal_local * (random.normalvariate(0,np.deg2rad(5))+2*np.pi)).as_matrix()
    # Rotate the true pointing vector by the pointing
    # error rotation to get the sensor pointing vector.
    vec_sensorPointingVec_local = dcm_truePointingVec2sensorPointingVec@vec_pointingVecToRgv_local
    # Finally convert to the UAS frame
    vec_sensorPointingVec_uas = dcm_local2uas@vec_sensorPointingVec_local
    return vec_sensorPointingVec_uas

if __name__ == "__main__":
    main()