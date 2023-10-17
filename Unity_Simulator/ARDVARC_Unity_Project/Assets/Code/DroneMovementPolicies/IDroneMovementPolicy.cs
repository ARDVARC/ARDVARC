using ARDVARC_Unity_Project.Structs;

namespace ARDVARC_Unity_Project.DroneMovementPolicy
{
    public interface IDroneMovementPolicy
    {
        public QuadcopterRotorForces CalculateDesiredRotorSpeeds(DroneModel droneModel);
        public void OnDrawGizmos();
    }
}
