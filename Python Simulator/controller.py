import socket
import struct
import numpy as np
import matplotlib.pyplot as plt

import simulator

def main():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect(("localhost", 1234))
        # Send reset command
        s.sendall(bytearray([simulator.SimulatorMessageKind.reset]))
        s.recv(1) # ACK
        # # Move forward 5 seconds
        # s.sendall(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
        # s.sendall(struct.pack("!d", 5))
        # s.recv(1) # ACK
        
        N = 500
        trix_vec_uasPositions_local = np.empty((3,N))
        trix_vec_rgv1Predictions_local = np.empty((3,N))
        for i in range(1,N+1):
            # Move to next time
            s.sendall(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
            s.sendall(struct.pack("!d", i/5))
            s.recv(1) # ACK
            
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
            coeff = z/vec_pointingVec_enu[2]
            vec_rgv1Prediction_local = vec_uasState[0:3] - vec_pointingVec_enu * coeff
            
            # Set waypoint to above RGV measurement
            s.sendall(bytearray([simulator.SimulatorMessageKind.setComplexWaypoint]))
            s.sendall(struct.pack("!dddddd", vec_rgv1Prediction_local[0], vec_rgv1Prediction_local[1], 10, vec_rgv1Prediction_local[0], vec_rgv1Prediction_local[1], vec_rgv1Prediction_local[2]))
            s.recv(1) # ACK
            
            # Store info to plot later
            trix_vec_rgv1Predictions_local[:,i-1] = vec_rgv1Prediction_local
            trix_vec_uasPositions_local[:,i-1] = vec_uasState[0:3]
    plt.scatter(trix_vec_rgv1Predictions_local[0,:],trix_vec_rgv1Predictions_local[1,:], s=1, c="red")
    plt.scatter(trix_vec_uasPositions_local[0,:],trix_vec_uasPositions_local[1,:], s=1, c="green")
    plt.axis('equal')
    plt.grid(which='both')
    plt.show()

if __name__ == "__main__":
    main()