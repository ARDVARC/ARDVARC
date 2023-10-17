using UnityEngine;
using ARDVARC_Unity_Project.RGVMovementPolicy;
using ARDVARC_Unity_Project.DroneMovementPolicy;
using ARDVARC_Unity_Project.DroneModelUpdaters;
using ARDVARC_Unity_Project.Structs;
using Random = System.Random;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.RosArdvarcUnitySim;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using System;

namespace ARDVARC_Unity_Project
{
    public class SceneManager : MonoBehaviour
    {
        public const int SHRUB_COUNT = 200;
        
        // Inspector
        public GameObject RGVPrefab;
        public GameObject DronePrefab;
        public GameObject ShrubPrefab;
    
        // Non-Inspector
        public Simulation Simulation { get; set; }
        [HideInInspector]
        public ROSConnection ros;
    
        // Start is called before the first frame update
        void Start()
        {
            // Time.fixedDeltaTime = 0.002f;
            // Time.captureDeltaTime = 60;
            Simulation = new Simulation(this, new Random(Simulation.DEFAULT_SIMULATION_SEED));
            ros = ROSConnection.GetOrCreateInstance();
            ros.RegisterPublisher<UAS_StateMsg>("uas_state");
            ros.RegisterPublisher<RGV_StateMsg>("rgv1_state");
            ros.RegisterPublisher<RGV_StateMsg>("rgv2_state");
            ros.RegisterPublisher<ImageMsg>("primary_sensor");
            if (Application.isEditor)
            {
                Debug.Break();
            }
            foreach (var i in Enumerable.Range(1, SHRUB_COUNT))
            {
                Instantiate(
                    ShrubPrefab,
                    new Vector3(
                        (((float)Simulation.Random.NextDouble())-0.5f)*Simulation.PLAY_AREA_SIZE,
                        0,
                        (((float)Simulation.Random.NextDouble())-0.5f)*Simulation.PLAY_AREA_SIZE
                    ),
                    Quaternion.AngleAxis(((float)Simulation.Random.NextDouble())*360, Vector3.up)
                );
            }
        }
    
        void FixedUpdate()
        {
            Simulation.FixedUpdate();
            
            var drone = Simulation.Drones[0];
            var droneModelState = drone.Model.ModelState;
            var primarySensor = drone.PrimarySensor;
            var rgv1 = Simulation.RGVs[0];
            var rgv1Transform = rgv1.gameObject.transform;
            var rgv2 = Simulation.RGVs[1];
            var rgv2Transform = rgv2.gameObject.transform;
            
            ros.Publish(
                "uas_state",
                new UAS_StateMsg(
                    droneModelState.GlobalPositionEstimate.ToPointMsg(),
                    Quaternion.Euler(droneModelState.GlobalEulerEstimate).ToQuaternionMsg(),
                    Extensions.ToTwistMsg(droneModelState.LocalVelocityEstimate, droneModelState.LocalAngularVelocityEstimate)
                )
            );
            ros.Publish(
                "rgv1_state",
                new RGV_StateMsg(
                    rgv1Transform.position.ToPointMsg(),
                    rgv1Transform.rotation.ToQuaternionMsg(),
                    ((StandardRGVMovementPolicy)rgv1.movementPolicy).IsMoving()
                )
            );
            ros.Publish(
                "rgv2_state",
                new RGV_StateMsg(
                    rgv2Transform.position.ToPointMsg(),
                    rgv2Transform.rotation.ToQuaternionMsg(),
                    ((StandardRGVMovementPolicy)rgv2.movementPolicy).IsMoving()
                )
            );
            
            var tempTexture = new RenderTexture(256, 256, 0, RenderTextureFormat.BGRA32);
            primarySensor.targetTexture = tempTexture;
            primarySensor.Render();
            RenderTexture.active = tempTexture;
            var image = new Texture2D(256, 256, TextureFormat.BGRA32, false);
            image.SetPixel(0,0,Color.white);
            image.ReadPixels(new Rect(0, 0, 256, 256), 0, 0);
            Graphics.CopyTexture(tempTexture, image);
            image.Apply();
            ros.Publish(
                "primary_sensor",
                new ImageMsg(
                    new HeaderMsg(),
                    (uint)primarySensor.targetTexture.height,
                    (uint)primarySensor.targetTexture.width,
                    "bgra8",
                    0,
                    1024,
                    image.GetRawTextureData()
                )
            );
            primarySensor.targetTexture = null;
        }
        
        void OnDrawGizmos()
        {
            Simulation?.OnDrawGizmos();
            Gizmos.color = Color.black;
            Gizmos.DrawLine(Vector3.zero, new Vector3(0, 40, 0));
        }
        
        void OnApplicationQuit()
        {
            
        }
    
        public RGV MakeRGV(Vector3 startPosition, Vector3 startDirection, Random random)
        {
            var rgv = new RGV(Instantiate(RGVPrefab), new StandardRGVMovementPolicy(random, startPosition, startDirection));
            rgv.gameObject.transform.position = startPosition;
            rgv.gameObject.transform.LookAt(startPosition + startDirection);
            return rgv;
        }
    
        public Drone MakeDrone(Vector3 startPosition, Random random)
        {
            var droneModel = new DroneModel(
                new HoverDroneMovementPolicy(),
                new DroneModelState(
                    startPosition,
                    Vector3.zero,
                    Vector3.zero,
                    Vector3.zero
                ),
                Vector3.zero);
            var droneModelUpdater = new DelayedExactDroneModelUpdater(droneModel);
            var drone = new Drone(Instantiate(DronePrefab), droneModel, droneModelUpdater, Simulation);
            drone.GameObject.transform.position = startPosition;
            // TODO - Temporary initial misalignment
            drone.GameObject.transform.Rotate(new Vector3(0.1f,0.1f,0.1f));
            return drone;
        }
    }
}
