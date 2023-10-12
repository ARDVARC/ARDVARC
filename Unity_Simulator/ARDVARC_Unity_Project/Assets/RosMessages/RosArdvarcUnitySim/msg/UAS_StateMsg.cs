//Do not edit! This file was generated by Unity-ROS MessageGeneration.
using System;
using System.Linq;
using System.Collections.Generic;
using System.Text;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;

namespace RosMessageTypes.RosArdvarcUnitySim
{
    [Serializable]
    public class UAS_StateMsg : Message
    {
        public const string k_RosMessageName = "ros_ardvarc_unity_sim/UAS_State";
        public override string RosMessageName => k_RosMessageName;

        public Geometry.PointMsg pos;
        public Geometry.QuaternionMsg orientation;
        public Geometry.TwistMsg twist;

        public UAS_StateMsg()
        {
            this.pos = new Geometry.PointMsg();
            this.orientation = new Geometry.QuaternionMsg();
            this.twist = new Geometry.TwistMsg();
        }

        public UAS_StateMsg(Geometry.PointMsg pos, Geometry.QuaternionMsg orientation, Geometry.TwistMsg twist)
        {
            this.pos = pos;
            this.orientation = orientation;
            this.twist = twist;
        }

        public static UAS_StateMsg Deserialize(MessageDeserializer deserializer) => new UAS_StateMsg(deserializer);

        private UAS_StateMsg(MessageDeserializer deserializer)
        {
            this.pos = Geometry.PointMsg.Deserialize(deserializer);
            this.orientation = Geometry.QuaternionMsg.Deserialize(deserializer);
            this.twist = Geometry.TwistMsg.Deserialize(deserializer);
        }

        public override void SerializeTo(MessageSerializer serializer)
        {
            serializer.Write(this.pos);
            serializer.Write(this.orientation);
            serializer.Write(this.twist);
        }

        public override string ToString()
        {
            return "UAS_StateMsg: " +
            "\npos: " + pos.ToString() +
            "\norientation: " + orientation.ToString() +
            "\ntwist: " + twist.ToString();
        }

#if UNITY_EDITOR
        [UnityEditor.InitializeOnLoadMethod]
#else
        [UnityEngine.RuntimeInitializeOnLoadMethod]
#endif
        public static void Register()
        {
            MessageRegistry.Register(k_RosMessageName, Deserialize);
        }
    }
}
