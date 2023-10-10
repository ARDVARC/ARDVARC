using AALG3.Structs;

namespace AALG3.DroneMovementPolicy
{
    public class YeetDroneMovementPolicy: IDroneMovementPolicy
    {
        public QuadcopterRotorForces CalculateDesiredRotorSpeeds(DroneModel droneModel)
        {
            return new QuadcopterRotorForces(10, 10, 10, 10);
        }

        public void OnDrawGizmos() {}
    }
}
