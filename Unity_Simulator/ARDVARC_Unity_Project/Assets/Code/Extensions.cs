using System;
using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using RosMessageTypes.ARDVARCExp;
using RosMessageTypes.Geometry;
using UnityEngine;

namespace AALG3
{
    public static class Extensions
    {
        public static TestMsg ToTestMessage(this Vector3 v) => new (new (v.x, v.y, v.z));
    }
}