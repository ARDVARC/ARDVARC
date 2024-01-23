import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

import simulator

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect(("localhost", 1234))
        # Move forward 5 seconds
        s.sendall(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
        s.sendall(struct.pack("!d", 5))
        ack = s.recv(1)
        
        N = 160
        trix_vec_uasPositions_local = np.empty((3,N))
        trix_vec_rgv1Predictions_local = np.empty((3,N))
        i = 0
        while i < N:
            i += 1
            # Move to next time
            s.sendall(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
            s.sendall(struct.pack("!d", 5+i/16))
            ack = s.recv(1)
            # Measure RGV 1
            s.sendall(bytearray([simulator.SimulatorMessageKind.measureRgv1]))
            pointingBytes = s.recv(24)
            pointingVals = struct.unpack("!ddd", pointingBytes)
            vec_pointing_uas = np.array(pointingVals)
            # Measure UAS
            s.sendall(bytearray([simulator.SimulatorMessageKind.measureUas]))
            uasStateBytes = s.recv(96)
            uasStateVals = struct.unpack("!dddddddddddd", uasStateBytes)
            _, _, z, phi, theta, psi, _, _, _, _, _, _ = uasStateVals
            vec_uasState = np.array(uasStateVals)
            # Project to ground to get RGV prediction
            sinphi = np.sin(phi)
            cosphi = np.cos(phi)
            sintheta = np.sin(theta)
            costheta = np.cos(theta)
            sinpsi = np.sin(psi)
            cospsi = np.cos(psi)
            dcm_uas2local = np.array([
                [costheta*cospsi,sinphi*sintheta*cospsi-cosphi*sinpsi,cosphi*sintheta*cospsi+sinphi*sinpsi],
                [costheta*sinpsi,sinphi*sintheta*sinpsi+cosphi*cospsi,cosphi*sintheta*sinpsi-sinphi*cospsi],
                [-sintheta      ,sinphi*costheta                     ,cosphi*costheta                     ]
            ])
            vec_pointingVec_enu = dcm_uas2local@vec_pointing_uas
            # print(vec_pointingVec_enu)
            coeff = z/vec_pointingVec_enu[2]
            
            trix_vec_rgv1Predictions_local[:,i-1] = vec_uasState[0:3] - vec_pointingVec_enu * coeff
            trix_vec_uasPositions_local[:,i-1] = vec_uasState[0:3]
    plt.scatter(trix_vec_rgv1Predictions_local[0,:],trix_vec_rgv1Predictions_local[1,:])
    plt.axis('equal')
    plt.grid(which='both')
    plt.show()

if __name__ == "__main__":
    main()