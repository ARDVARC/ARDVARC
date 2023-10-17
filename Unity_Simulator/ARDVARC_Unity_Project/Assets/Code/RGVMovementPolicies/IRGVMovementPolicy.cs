namespace ARDVARC_Unity_Project.RGVMovementPolicy
{
    public interface IRGVMovementPolicy
    {
        public void Move(RGV RGV);
        public void OnDrawGizmos();
    }
}