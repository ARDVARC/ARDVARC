using UnityEngine;
using Random = System.Random;

namespace AALG3.RGVMovementPolicy
{
    public class StandardRGVMovementPolicy : IRGVMovementPolicy
    {
        abstract class PathSegment
        {
            public readonly float startTime;
            public readonly float pathTime;
            
            protected PathSegment(float pathTime, float spilloverTime)
            {
                startTime = Time.timeSinceLevelLoad - spilloverTime;
                this.pathTime = pathTime;
            }
            
            /// <summary>
            /// Determines the current position and direction along the path. If the path is complete, also gives the amount of spillover time to use on the next path.
            /// </summary>
            /// <param name="pos">The current position of the RGV along the path, or the end of the path if the path is now complete.</param>
            /// <param name="dir">The current direction of the RGV along the path, or the direction at the end of the path if the path is now complete.</param>
            /// <param name="spilloverTime">If false, the amount of time to continue into the next path segment(s).</param>
            /// <returns>Whether or not the path is now complete. If it is, use <paramref name="spilloverTIme"/> to continue along the next path segment.</returns>
            public abstract bool GetCurrentPositionAlongPath(out Vector3 pos, out Vector3 dir, out float spilloverTime);
            public abstract void OnDrawGizmos();
        }
        
        class StraightPathSegment : PathSegment
        {
            const float STRAIGHT_MIN_TIME = 5;
            const float STRAIGHT_TIME_RANGE = 10;
            readonly Vector3 startPosition;
            readonly Vector3 pathDirection;

            StraightPathSegment(Vector3 startPosition, Vector3 pathDirection, float pathTime, float spilloverTime) : base(pathTime, spilloverTime)
            {
                this.startPosition = startPosition;
                this.pathDirection = pathDirection;
            }

            public override bool GetCurrentPositionAlongPath(out Vector3 pos, out Vector3 dir, out float spilloverTime)
            {
                dir = pathDirection;
                var timeWaited = Time.timeSinceLevelLoad - startTime;
                if (timeWaited > pathTime)
                {
                    pos = startPosition + pathTime * RGV.SPEED * pathDirection;
                    spilloverTime = timeWaited - pathTime;
                    return true;
                }
                else
                {
                    pos = startPosition + timeWaited * RGV.SPEED * pathDirection;
                    spilloverTime = float.NaN;
                    return false;
                }
            }
            
            public static StraightPathSegment GetRandom(Random random, Vector3 startPosition, Vector3 startDirection, float spilloverTime)
            {
                return new StraightPathSegment(startPosition, startDirection, (float)random.NextDouble() % STRAIGHT_TIME_RANGE + STRAIGHT_MIN_TIME, spilloverTime);
            }
            
            public override void OnDrawGizmos()
            {
                
            }
        }
        
        class WaitingPathSegment : PathSegment
        {
            const float WAITING_MIN_TIME = 5;
            const float WAITING_TIME_RANGE = 10;
            readonly Vector3 startPosition;
            readonly Vector3 startDirection;
            
            WaitingPathSegment(Vector3 startPosition, Vector3 startDirection, float waitTime, float spilloverTime) : base(waitTime, spilloverTime)
            {
                this.startPosition = startPosition;
                this.startDirection = startDirection;
            }

            public override bool GetCurrentPositionAlongPath(out Vector3 pos, out Vector3 dir, out float spilloverTime)
            {
                var timeWaited = Time.timeSinceLevelLoad - startTime;
                pos = startPosition;
                dir = startDirection;
                if (timeWaited > pathTime)
                {
                    spilloverTime = timeWaited - pathTime;
                    return true;
                }
                else
                {
                    spilloverTime = float.NaN;
                    return false;
                }
            }
            
            public static WaitingPathSegment GetRandom(Random random, Vector3 startPosition, Vector3 startDirection, float spilloverTime)
            {
                return new WaitingPathSegment(startPosition, startDirection, (float)random.NextDouble() % WAITING_TIME_RANGE + WAITING_MIN_TIME, spilloverTime);
            }
            
            public override void OnDrawGizmos()
            {
                
            }
        }
        
        class ArcingPathSegment : PathSegment
        {
            const float ARCING_MIN_TIME = 5;
            const float ARCING_TIME_RANGE = 10;
            const float ARCING_MIN_RADIUS = 5;
            const float ARCING_RADIUS_RANGE = 5;
            readonly Vector3 arcCenter;
            readonly Vector3 arcArmToStartPosition;
            readonly float radialVelocityInDeg;
            readonly int turningSign;
            
            public ArcingPathSegment(Vector3 startPosition, Vector3 startDir, float arcRadius, int turningSign, float pathTime, float spilloverTime) : base(pathTime, spilloverTime)
            {
                arcArmToStartPosition = arcRadius * new Vector3(turningSign * -startDir.z, startDir.y, turningSign * startDir.x);
                arcCenter = startPosition - arcArmToStartPosition;
                radialVelocityInDeg = turningSign * RGV.SPEED / arcRadius * Mathf.Rad2Deg;
                this.turningSign = turningSign;
            }
            
            public override bool GetCurrentPositionAlongPath(out Vector3 pos, out Vector3 dir, out float spilloverTime)
            {
                var timeWaited = Time.timeSinceLevelLoad - startTime;
                var complete = false;
                if (timeWaited > pathTime)
                {
                    complete = true;
                    spilloverTime = timeWaited - pathTime;
                    timeWaited = pathTime;
                }
                else
                {
                    spilloverTime = float.NaN;
                }
                var angle = radialVelocityInDeg * timeWaited;
                pos = arcCenter + Quaternion.AngleAxis(angle, Vector3.up) * arcArmToStartPosition;
                dir = Quaternion.AngleAxis(angle + turningSign * 90, Vector3.up) * arcArmToStartPosition.normalized;
                return complete;
            }
            
            public static ArcingPathSegment GetRandom(Random random, Vector3 startPosition, Vector3 startDirection, float spilloverTime)
            {
                return new ArcingPathSegment(startPosition, startDirection, (float)random.NextDouble() % ARCING_RADIUS_RANGE + ARCING_MIN_RADIUS, random.Next(2) * 2 - 1, (float)random.NextDouble() % ARCING_TIME_RANGE + ARCING_MIN_TIME, spilloverTime);
            }
            
            public override void OnDrawGizmos()
            {
                Gizmos.color = Color.black;
                Gizmos.DrawRay(arcCenter, Vector3.up);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(arcCenter, arcArmToStartPosition);
            }
        }
        
        class EdgeAvoidingPathSegment : PathSegment
        {
            public const float EDGE_AVOID_ARC_RADIUS = Simulation.PLAY_AREA_MARGIN/2;
            public const float EDGE_AVOID_ARC_TIME = Mathf.PI*EDGE_AVOID_ARC_RADIUS/RGV.SPEED;
            public const float EDGE_AVOID_STRAIGHT_TIME = 5;
            readonly Vector3 arcCenter;
            readonly Vector3 arcArmToStartPosition;
            readonly float radialVelocityInDeg;
            readonly int turningSign;
            
            public EdgeAvoidingPathSegment(Vector3 startPosition, Vector3 startDir, int turningSign) : base(EDGE_AVOID_ARC_TIME + EDGE_AVOID_STRAIGHT_TIME, 0)
            {
                arcArmToStartPosition = EDGE_AVOID_ARC_RADIUS * new Vector3(turningSign * -startDir.z, startDir.y, turningSign * startDir.x);
                arcCenter = startPosition - arcArmToStartPosition;
                radialVelocityInDeg = turningSign * RGV.SPEED / EDGE_AVOID_ARC_RADIUS * Mathf.Rad2Deg;
                this.turningSign = turningSign;
            }
            
            public override bool GetCurrentPositionAlongPath(out Vector3 pos, out Vector3 dir, out float spilloverTime)
            {
                var timeWaited = Time.timeSinceLevelLoad - startTime;
                float arcTime, straightTime;
                var complete = false;
                
                if (timeWaited < EDGE_AVOID_ARC_TIME)
                {
                    arcTime = timeWaited;
                    straightTime = 0;
                    spilloverTime = float.NaN;
                }
                else if (timeWaited < EDGE_AVOID_ARC_TIME + EDGE_AVOID_STRAIGHT_TIME)
                {
                    arcTime = EDGE_AVOID_ARC_TIME;
                    straightTime = timeWaited - EDGE_AVOID_ARC_TIME;
                    spilloverTime = float.NaN;
                }
                else
                {
                    arcTime = EDGE_AVOID_ARC_TIME;
                    straightTime = EDGE_AVOID_STRAIGHT_TIME;
                    spilloverTime = timeWaited - pathTime;
                    complete = true;
                }
                
                // Do the turn
                var angle = radialVelocityInDeg * arcTime;
                pos = arcCenter + Quaternion.AngleAxis(angle, Vector3.up) * arcArmToStartPosition;
                dir = Quaternion.AngleAxis(angle + turningSign * 90, Vector3.up) * arcArmToStartPosition.normalized;
                
                // Maybe do the straight
                if (straightTime > 0)
                {
                    pos += straightTime * RGV.SPEED * dir;
                }
                
                return complete;
            }
            
            public static EdgeAvoidingPathSegment GetNew(Vector3 pos, Vector3 dir)
            {
                return new EdgeAvoidingPathSegment(pos, dir, Vector3.SignedAngle(dir, pos, Vector3.up) > 0 ? -1 : 1);
            }
            
            public override void OnDrawGizmos()
            {
                Gizmos.color = Color.black;
                Gizmos.DrawRay(arcCenter, Vector3.up);
                Gizmos.color = Color.red;
                Gizmos.DrawRay(arcCenter, arcArmToStartPosition);
            }
        }
        
        PathSegment currentPathSegment;
        Random randomizer;
        bool avoidingEdge = false;

        public StandardRGVMovementPolicy(Random randomizer, Vector3 startPosition, Vector3 startDirection)
        {
            this.randomizer = randomizer;
            currentPathSegment = GetRandomPathSegment(startPosition, startDirection);
        }
        
        PathSegment GetRandomPathSegment(Vector3 startPosition, Vector3 startDirection, float spilloverTime = 0)
        {
            return randomizer.Next(3) switch
            {
                0 => StraightPathSegment.GetRandom(randomizer, startPosition, startDirection, spilloverTime),
                1 => WaitingPathSegment.GetRandom(randomizer, startPosition, startDirection, spilloverTime),
                _ => ArcingPathSegment.GetRandom(randomizer, startPosition, startDirection, spilloverTime),
            };
        }

        public void Move(RGV RGV)
        {
            Vector3 pos, dir;
            while (currentPathSegment.GetCurrentPositionAlongPath(out pos, out dir, out float spilloverTime))
            {
                RGV.gameObject.transform.position = pos;
                RGV.gameObject.transform.LookAt(pos + dir);
                currentPathSegment = GetRandomPathSegment(pos, dir, spilloverTime);
                avoidingEdge = false;
            }
            RGV.gameObject.transform.position = pos;
            RGV.gameObject.transform.LookAt(pos + dir);
            
            // Steer away from edges
            if (!avoidingEdge && (Mathf.Abs(pos.x) > (Simulation.PLAY_AREA_SIZE/2 - Simulation.PLAY_AREA_MARGIN) || Mathf.Abs(pos.z) > Simulation.PLAY_AREA_SIZE/2 - Simulation.PLAY_AREA_MARGIN))
            {
                currentPathSegment = EdgeAvoidingPathSegment.GetNew(pos, dir);
                avoidingEdge = true;
            }
        }
        
        public void OnDrawGizmos()
        {
            currentPathSegment.OnDrawGizmos();
        }
        
        public bool IsMoving() => currentPathSegment is not WaitingPathSegment;
    }
}
