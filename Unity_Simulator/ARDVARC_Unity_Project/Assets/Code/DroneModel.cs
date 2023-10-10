using UnityEngine;
using AALG3.DroneMovementPolicy;
using AALG3.Structs;
using System;

namespace AALG3
{
    public class DroneModel
    {
        public IDroneMovementPolicy MovementPolicy { get; }
        public DroneModelState ModelState;
        // TODO: WIP
        public Vector3 RGVPositionEstimate;
        
        public DroneModel(IDroneMovementPolicy movementPolicy, DroneModelState droneModelState, Vector3 RGVInitialModeledPosition)
        {
            MovementPolicy = movementPolicy;
            ModelState = droneModelState;
            RGVPositionEstimate = RGVInitialModeledPosition;
        }

        public QuadcopterRotorForces CalculateDesiredRotorSpeeds() => MovementPolicy.CalculateDesiredRotorSpeeds(this);
        
        public void OnDrawGizmos()
        {
            MovementPolicy.OnDrawGizmos();
        }
        
        public static QuadcopterRotorForces CalculateDesiredRotorSpeedsFromLMNZ(LMNZ lmnz)
        {
            var SQRT2 = Mathf.Sqrt(2);
            var speeds = new QuadcopterRotorForces(
                lmnz.Z + lmnz.N/(4*Drone.K_M) + SQRT2*(-lmnz.L+lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - lmnz.N/(4*Drone.K_M) + SQRT2*(-lmnz.L-lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z + lmnz.N/(4*Drone.K_M) + SQRT2*(lmnz.L-lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - lmnz.N/(4*Drone.K_M) + SQRT2*(lmnz.L+lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM)
            );
            
            if (speeds.F1 <= Drone.MAX_ROTOR_SPEED && speeds.F2 <= Drone.MAX_ROTOR_SPEED && speeds.F3 <= Drone.MAX_ROTOR_SPEED && speeds.F4 <= Drone.MAX_ROTOR_SPEED)
            {
                return speeds;
            }
            
            speeds = new QuadcopterRotorForces(
                lmnz.Z + SQRT2*(-lmnz.L+lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - SQRT2*(-lmnz.L-lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z + SQRT2*(lmnz.L-lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - SQRT2*(lmnz.L+lmnz.M)/(4*Drone.ROTOR_MOMENT_ARM)
            );
            
            if (speeds.F1 <= Drone.MAX_ROTOR_SPEED && speeds.F2 <= Drone.MAX_ROTOR_SPEED && speeds.F3 <= Drone.MAX_ROTOR_SPEED && speeds.F4 <= Drone.MAX_ROTOR_SPEED)
            {
                return speeds;
            }
            
            return new QuadcopterRotorForces(
                lmnz.Z + SQRT2*lmnz.M/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - SQRT2*lmnz.M/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z + SQRT2*lmnz.M/(4*Drone.ROTOR_MOMENT_ARM),
                lmnz.Z - SQRT2*lmnz.M/(4*Drone.ROTOR_MOMENT_ARM)
            );
        }
    }
}
