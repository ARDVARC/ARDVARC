using AALG3.Structs;
using UnityEngine;
using Unity.Robotics;
using RosMessageTypes;
using RosMessageTypes.ARDVARCExp;

namespace AALG3.DroneMovementPolicy
{
    public class HoverDroneMovementPolicy: IDroneMovementPolicy
    {
        public const float ZK_y = 200, ZK_v = 200, ZK_grav = 9.81f/4, ZK_z = 80;
        public const float LK_roll_rate = 40, LK_roll = 60, LK_x = 20, LK_u = 60, LK_target_u = 0;
        public const float MK_pitch_rate = 250, MK_pitch = 15, MK_z = 100, MK_w = 60, MK_pitch_deadband = 0;
        public const float NK_yaw_rate = 60, NK_yaw = 1.2f;
        public const float TARGET_DISTANCE_FROM_DRONE = 4;
        readonly Vector3 TARGET_HEIGHT_VECTOR = new Vector3(0, 10, 0);
        
        Vector3 globalVectorToTargetCenter = Vector3.zero;
        Vector3 localVectorToTarget = Vector3.zero;
        Vector3 droneModelPosition = Vector3.zero;
    
        public QuadcopterRotorForces CalculateDesiredRotorSpeeds(DroneModel droneModel)
        {
            var targetLocation = droneModel.RGVPositionEstimate + TARGET_HEIGHT_VECTOR;
            var orientation = Quaternion.Euler(droneModel.ModelState.GlobalEulerEstimate) * Vector3.forward;
            var pitch = Vector3.SignedAngle(orientation, Vector3.up, Vector3.Cross(Vector3.Normalize(orientation), Vector3.up)) - 90;
            var yaw = Vector3.SignedAngle(orientation, Vector3.forward, Vector3.Cross(Vector3.Normalize(orientation), Vector3.forward));
            var roll = droneModel.ModelState.GlobalEulerEstimate.z;
            droneModelPosition = droneModel.ModelState.GlobalPositionEstimate;
            globalVectorToTargetCenter = -(droneModelPosition - targetLocation);
            var globalVectorToTarget = globalVectorToTargetCenter - globalVectorToTargetCenter.normalized * TARGET_DISTANCE_FROM_DRONE;
            localVectorToTarget = Quaternion.Inverse(Quaternion.Euler(droneModel.ModelState.GlobalEulerEstimate)) * globalVectorToTarget;
            var flatLocalVectorToTarget = Quaternion.Inverse(Quaternion.Euler(droneModel.ModelState.GlobalEulerEstimate)) * Vector3.ProjectOnPlane(globalVectorToTarget, Vector3.up);

            BoundPlusMinus180(ref pitch);
            BoundPlusMinus180(ref yaw);
            BoundPlusMinus180(ref roll);

            var weightedVerticalPositionError = localVectorToTarget.y * ZK_y;
            var weightedVericalVelocityError = -(droneModel.ModelState.LocalVelocityEstimate.y - 0) * ZK_v;
            var gravityError = ZK_grav / Drone.ROTOR_SPEED_TO_FORCE;
            var weightedForwardPositionErrorForZ = (flatLocalVectorToTarget.z < 0 ? 2 : 1) * -flatLocalVectorToTarget.z * Mathf.Min(ZK_z, Mathf.Max(-ZK_z, pitch != 0 ? 1/pitch : 0));
            var Z = weightedVerticalPositionError + weightedVericalVelocityError + gravityError + weightedForwardPositionErrorForZ;

            var weightedRollRateError = (droneModel.ModelState.LocalAngularVelocityEstimate.z - 0) * LK_roll_rate;
            var weightedRollAngleError = (roll - 0) * LK_roll;
            var weightedHorizontalPositionError = localVectorToTarget.x * LK_x;
            var weightedHorizontalVelocityError = (LK_target_u - droneModel.ModelState.LocalVelocityEstimate.x) * LK_u;
            var L = weightedRollRateError + weightedRollAngleError + weightedHorizontalPositionError + weightedHorizontalVelocityError;

            var weightedPitchRateError = (droneModel.ModelState.LocalAngularVelocityEstimate.x - 0) * MK_pitch_rate;
            var pitchAngleError = PitchAngleError();
            var weightedPitchAngleError = pitchAngleError * Mathf.Abs(pitchAngleError) * MK_pitch;
            var weightedForwardPositionError = (flatLocalVectorToTarget.z < 0 ? 2 : 1) * -flatLocalVectorToTarget.z * MK_z;
            var weightedForwardVelocityError = -(0 - droneModel.ModelState.LocalVelocityEstimate.z) * MK_w;
            var M = weightedPitchRateError + weightedPitchAngleError + weightedForwardPositionError + weightedForwardVelocityError;

            var weightedYawRateError = -(droneModel.ModelState.LocalAngularVelocityEstimate.y - 0) * NK_yaw_rate;
            var weightedYawAngleError = -YawAngleError() * NK_yaw;
            var N = weightedYawRateError + weightedYawAngleError;

            LogWeightedValues();

            return DroneModel.CalculateDesiredRotorSpeedsFromLMNZ(new LMNZ(L, M, N, Z));

            float YawAngleError()
            {
                var targetDirection = -Vector3.Normalize(droneModel.ModelState.GlobalPositionEstimate - targetLocation);
                var targetDirectionFlat = new Vector3(targetDirection.x, 0, targetDirection.z);
                var droneForward = Quaternion.Euler(droneModel.ModelState.GlobalEulerEstimate) * Vector3.forward;
                var droneForwardFlat = new Vector3(droneForward.x, 0, droneForward.z);
                var angleToGetToCenter = Vector3.SignedAngle(targetDirectionFlat, droneForwardFlat, Vector3.up);
                BoundPlusMinus180(ref angleToGetToCenter);
                return angleToGetToCenter;
            }

            float PitchAngleError()
            {
                return Mathf.Max(Mathf.Abs(pitch) - MK_pitch_deadband, 0) * Mathf.Sign(pitch);
            }

            void BoundPlusMinus180(ref float angle)
            {
                if (angle < -180)
                {
                    angle += 360;
                }
                else if (angle > 180)
                {
                    angle -= 360;
                }
            }

            void LogWeightedValues()
            {
                if (!Application.isEditor)
                {
                    return;
                }
                
                DebugHelper.PublishROSUpdate(droneModel.ModelState.GlobalPositionEstimate.ToTestMessage());
                
                DebugHelper.DroneWeightsWriter.WriteLine(
                    $"{Time.time}, " +
                    $"{weightedVerticalPositionError}, " +
                    $"{weightedVericalVelocityError}, " +
                    $"{Z}, " +
                    $"{weightedRollRateError}, " +
                    $"{weightedRollAngleError}, " +
                    $"{weightedHorizontalPositionError}, " +
                    $"{weightedHorizontalVelocityError}, " +
                    $"{L}, " +
                    $"{weightedPitchRateError}, " +
                    $"{weightedPitchAngleError}, " +
                    $"{weightedForwardPositionError}, " +
                    $"{weightedForwardVelocityError}, " +
                    $"{M}, " +
                    $"{weightedYawRateError}, " +
                    $"{weightedYawAngleError}, " +
                    $"{N}, " +
                    $"{pitch}, " +
                    $"{yaw}, " +
                    $"{roll}, " +
                    $"{droneModel.ModelState.LocalVelocityEstimate.z}, " +
                    $"{droneModel.ModelState.LocalVelocityEstimate.x}, " +
                    $"{droneModel.ModelState.LocalVelocityEstimate.y}, " +
                    $"{droneModel.ModelState.LocalAngularVelocityEstimate.x}, " +
                    $"{droneModel.ModelState.LocalAngularVelocityEstimate.y}, " +
                    $"{droneModel.ModelState.LocalAngularVelocityEstimate.z}, " +
                    $"{DebugHelper.CurrentWind().magnitude}, " +
                    $"{(DebugHelper.RGV1IsMoving() ? 1 : 0)}, " +
                    $"{gravityError}, " +
                    $"{weightedForwardPositionErrorForZ}"
                );
            }
        }
        
        public void OnDrawGizmos()
        {
            Gizmos.color = Color.black;
            Gizmos.DrawRay(droneModelPosition, globalVectorToTargetCenter);
            Gizmos.color = Color.red;
            Gizmos.DrawRay(droneModelPosition, localVectorToTarget);
        }
    }
}