import asyncio
import socket
import struct
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R 

import simulator

async def main():
    reader, writer = await asyncio.open_connection("localhost", 1234)
    # Send reset command
    writer.write(bytearray([simulator.SimulatorMessageKind.reset]))
    await reader.read(1) # ACK
    # # Move forward 5 seconds
    # s.sendall(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
    # s.sendall(struct.pack("!d", 5))
    # s.recv(1) # ACK
    
    N = 500
    trix_vec_uasPositions_local_ned = np.empty((3,N))
    trix_vec_rgv1Predictions_local_ned = np.empty((3,N))
    for i in range(1,N+1):
        # Move to next time
        writer.write(bytearray([simulator.SimulatorMessageKind.simulateUntil]))
        writer.write(struct.pack("!d", i/5))
        await reader.read(1) # ACK
        
        # Measure RGV 1
        writer.write(bytearray([simulator.SimulatorMessageKind.measureRgv1]))
        pointingBytes = await reader.read(24)
        pointingVals = struct.unpack("!ddd", pointingBytes)
        vec_pointing_uas = np.array(pointingVals)
        
        # Measure UAS
        writer.write(bytearray([simulator.SimulatorMessageKind.measureUas]))
        uasStateBytes = await reader.read(96)
        uasStateVals = struct.unpack("!dddddddddddd", uasStateBytes)
        vec_uasState = np.array(uasStateVals)
        print(vec_uasState[0:3])
        
        # Project to ground to get RGV prediction
        dcm_uas2local_ned = R.from_euler("xyz", vec_uasState[3:6]).as_matrix()
        vec_pointingVec_local_ned = dcm_uas2local_ned@vec_pointing_uas
        coeff = uasStateVals[2]/vec_pointingVec_local_ned[2]
        vec_rgv1Prediction_local_ned = vec_uasState[0:3] - vec_pointingVec_local_ned * coeff
        
        # Set waypoint to above RGV measurement
        writer.write(bytearray([simulator.SimulatorMessageKind.setComplexWaypoint]))
        writer.write(struct.pack("!dddddd", vec_rgv1Prediction_local_ned[0], vec_rgv1Prediction_local_ned[1], -10, vec_rgv1Prediction_local_ned[0], vec_rgv1Prediction_local_ned[1], vec_rgv1Prediction_local_ned[2]))
        await reader.read(1) # ACK
        
        # Store info to plot later
        trix_vec_rgv1Predictions_local_ned[:,i-1] = vec_rgv1Prediction_local_ned
        trix_vec_uasPositions_local_ned[:,i-1] = vec_uasState[0:3]
    plt.scatter(trix_vec_rgv1Predictions_local_ned[1,:],trix_vec_rgv1Predictions_local_ned[0,:], s=1, c="red")
    plt.scatter(trix_vec_uasPositions_local_ned[1,:],trix_vec_uasPositions_local_ned[0,:], s=1, c="green")
    plt.axis('equal')
    plt.grid(which='both')
    plt.show()

if __name__ == "__main__":
    asyncio.run(main())