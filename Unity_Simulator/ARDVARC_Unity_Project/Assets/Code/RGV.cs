using UnityEngine;
using AALG3.RGVMovementPolicy;
using System.Collections.Generic;
using System.Linq;
using AALG3.DroneMovementPolicy;

namespace AALG3
{
    public class RGV
    {
        public const float SPEED = 1;
        static int RGVcounter = 0;
        public readonly GameObject gameObject;
        public readonly IRGVMovementPolicy movementPolicy;
        public readonly int RGVId;

        public RGV(GameObject gameObject, IRGVMovementPolicy movementPolicy)
        {
            this.gameObject = gameObject;
            this.movementPolicy = movementPolicy;
            RGVId = ++RGVcounter;
        }

        public void Update() => movementPolicy.Move(this);
        
        public void OnDrawGizmos()
        {
            movementPolicy.OnDrawGizmos();
            Gizmos.color = Color.black;
            var points = PointsInACircleAbout(gameObject.transform.position + Vector3.up * 0.1f, HoverDroneMovementPolicy.TARGET_DISTANCE_FROM_DRONE, 32).ToArray();
            Gizmos.DrawLineStrip(points, true);
            
            IEnumerable<Vector3> PointsInACircleAbout(Vector3 center, float radius, int numberOfPoints)
            {
                foreach (var n in Enumerable.Range(1, numberOfPoints))
                {
                    var angleInDegrees = (n-1.0f)/numberOfPoints*360;
                    yield return center + Quaternion.AngleAxis(angleInDegrees, Vector3.up) * Vector3.forward * radius;
                }
            }
        }
    }
}

