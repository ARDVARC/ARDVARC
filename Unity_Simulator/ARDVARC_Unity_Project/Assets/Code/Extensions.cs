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
        public static Vector3Msg ToTestMessage(this Vector3 v) => new (v.x, v.y, v.z);
    }
}