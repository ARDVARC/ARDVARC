using UnityEngine;

namespace AALG3.Structs
{
    public struct DroneModelState
    {
        public Vector3 GlobalPositionEstimate;
        public Vector3 GlobalEulerEstimate;
        public Vector3 LocalVelocityEstimate;
        public Vector3 LocalAngularVelocityEstimate;
        
        public DroneModelState(Vector3 globalPositionEstimate, Vector3 globalEulerEstimate, Vector3 localVelocityEstimate, Vector3 localAngularVelocityEstimate)
        {
            GlobalPositionEstimate = globalPositionEstimate;
            GlobalEulerEstimate = globalEulerEstimate;
            LocalVelocityEstimate = localVelocityEstimate;
            LocalAngularVelocityEstimate = localAngularVelocityEstimate;
        }
    }
}