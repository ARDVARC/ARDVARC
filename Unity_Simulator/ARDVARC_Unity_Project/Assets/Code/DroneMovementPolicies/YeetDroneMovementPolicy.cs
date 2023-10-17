using ARDVARC_Unity_Project.Structs;

namespace ARDVARC_Unity_Project.DroneMovementPolicy
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
