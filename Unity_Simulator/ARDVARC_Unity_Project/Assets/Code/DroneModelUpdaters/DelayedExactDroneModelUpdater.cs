using ARDVARC_Unity_Project.Structs;
using UnityEngine;

namespace ARDVARC_Unity_Project.DroneModelUpdaters
{
    public class DelayedExactDroneModelUpdater: IDroneModelUpdater
    {
        public float UPDATE_DELAY = 0;
    
        public DroneModel Model { get; }
        public float TimeSinceLastUpdate { get; set; } = 0;
        public DroneModelState OldDroneModelState;

        public DelayedExactDroneModelUpdater(DroneModel model)
        {
            Model = model;
            OldDroneModelState = model.ModelState;
        }
    
        public void TryUpdateModel(Drone drone, Simulation simulation)
        {
            TimeSinceLastUpdate += Time.deltaTime;
            if (TimeSinceLastUpdate > UPDATE_DELAY)
            {
                // Model.ModelState = OldDroneModelState;
                TimeSinceLastUpdate = 0;
                
                OldDroneModelState.GlobalPositionEstimate = drone.GameObject.transform.position;
                OldDroneModelState.GlobalEulerEstimate = drone.GameObject.transform.rotation.eulerAngles;
                var rb = drone.GameObject.GetComponent<Rigidbody>();
                OldDroneModelState.LocalVelocityEstimate = drone.GameObject.transform.InverseTransformDirection(rb.velocity);
                OldDroneModelState.LocalAngularVelocityEstimate = drone.GameObject.transform.InverseTransformDirection(rb.angularVelocity);
                
                // TEMP
                Model.ModelState = OldDroneModelState;
                Model.RGVPositionEstimate = simulation.RGVs[0].gameObject.transform.position;
            }
        }
    }
}