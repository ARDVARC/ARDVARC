using UnityEngine;
using System.IO;
using System;
using ARDVARC_Unity_Project.RGVMovementPolicy;
using RosMessageTypes.Geometry;
using RosMessageTypes.RosArdvarcUnitySim;

namespace ARDVARC_Unity_Project
{
    public static class DebugHelper
    {
        public static readonly StreamWriter DroneWeightsWriter = new ($"{Application.dataPath}/DataDump/DroneWeights/{DateTime.Now:yyyy-dd-M--HH-mm-ss}_datadump.csv");
        static Simulation Simulation = null!;
        
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
                "Gravity Error"
            );
        }
        
        public static void Teardown()
        {
            DroneWeightsWriter.Dispose();
        }
        
        public static Vector3 CurrentWind() => Simulation.Wind.CurrentWind();
        
        public static bool RGV1IsMoving() => Simulation.RGVs[0].movementPolicy is not StandardRGVMovementPolicy standardRGVMovementPolicy || standardRGVMovementPolicy.IsMoving();
        
        public static void PublishROSUpdate(UAS_StateMsg msg) => Simulation.SceneManager.ros.Publish("uas_state", msg);
    }
}