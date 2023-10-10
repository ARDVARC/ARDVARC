using UnityEngine;

namespace AALG3.RGVMovementPolicy
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
