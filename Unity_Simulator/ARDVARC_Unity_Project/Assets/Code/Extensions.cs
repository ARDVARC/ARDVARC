using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using RosMessageTypes.Geometry;
using UnityEngine;

namespace AALG3
{
    public static class Extensions
    {
        public static PointMsg ToPointMsg(this Vector3 v) => new (v.x, v.y, v.z);
        public static QuaternionMsg ToQuaternionMsg(this Quaternion q) => new(q.w, q.x, q.y, q.z);
        public static Vector3Msg ToVector3Msg(this Vector3 v) => new (v.x, v.y, v.z);
        public static TwistMsg ToTwistMsg(Vector3 linear, Vector3 angular) => new (linear.ToVector3Msg(), angular.ToVector3Msg());
    }
}