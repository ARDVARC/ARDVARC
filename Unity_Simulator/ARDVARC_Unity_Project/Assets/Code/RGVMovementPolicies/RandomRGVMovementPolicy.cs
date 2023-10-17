using UnityEngine;

namespace ARDVARC_Unity_Project.RGVMovementPolicy
{
    public class RandomRGVMovementPolicy: IRGVMovementPolicy
    {
        public void Move(RGV RGV)
        {
            RGV.gameObject.transform.Translate(Time.deltaTime, 0, 0);
        }
        
        public void OnDrawGizmos() {}
    }
}
