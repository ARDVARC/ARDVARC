using UnityEngine;
using System.IO;
using System;
using AALG3.RGVMovementPolicy;

namespace AALG3
{
    public static class DebugHelper
    {
        public static readonly StreamWriter DroneWeightsWriter = new ($"./Assets/DataDump/DroneWeights/{DateTime.Now:yyyy-dd-M--HH-mm-ss}.csv");
        static Simulation Simulation;
        
        public static void Setup(Simulation simulation)
        {
            Simulation = simulation;
            DroneWeightsWriter.WriteLine(
                "Time, " +
                "Weighted Vertical Position Error, " +
                "Weighted Vertical Velocity Error, " +
                "Z, " +
                "Weighted Roll Rate Error, " +
                "Weighted Roll Angle Error, " +
                "Weighted Horizontal Position Error, " +
                "Weighted Horizontal Velocity Error, " +
                "L, " +
                "Weighted Pitch Rate Error, " +
                "Weighted Pitch Angle Error, " +
                "Weighted Forward Position Error, " +
                "Weighted Forward Velocity Error, " +
                "M, " +
                "Weighted Yaw Rate Error, " +
                "Weighted Yaw Angle Error, " +
                "N," +
                "Pitch, " + 
                "Roll, " + 
                "Yaw, " +
                "U, " + 
                "V, " + 
                "W, " +
                "Pitch Rate, " + 
                "Yaw Rate, " + 
                "Roll Rate, " +
                "Wind Speed, " +
                "RGV1 Moving?, " +
                "Gravity Error, " +
                "Weighted Forward Position Error For Z"
            );
        }
        
        public static Vector3 CurrentWind() => Simulation.Wind.CurrentWind();
        
        public static bool RGV1IsMoving() => Simulation.RGVs[0].movementPolicy is not StandardRGVMovementPolicy standardRGVMovementPolicy || standardRGVMovementPolicy.IsMoving();
    }
}