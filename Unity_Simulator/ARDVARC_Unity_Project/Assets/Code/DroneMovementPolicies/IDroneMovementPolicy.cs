using AALG3.Structs;

namespace AALG3.DroneMovementPolicy
{
    public interface IDroneMovementPolicy
    {
        public QuadcopterRotorForces CalculateDesiredRotorSpeeds(DroneModel droneModel);
        public void OnDrawGizmos();
    }
}
