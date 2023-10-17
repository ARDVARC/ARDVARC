using UnityEngine;
using ARDVARC_Unity_Project.RGVMovementPolicy;
using ARDVARC_Unity_Project.DroneMovementPolicy;
using ARDVARC_Unity_Project.DroneModelUpdaters;
using ARDVARC_Unity_Project.Structs;
using Random = System.Random;
using System.Linq;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.RosArdvarcUnitySim;

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
        public ROSConnection ros;
    
        // Start is called before the first frame update
        void Start()
        {
            // Time.fixedDeltaTime = 0.002f;
            // Time.captureDeltaTime = 60;
            Simulation = new Simulation(this, new Random(Simulation.DEFAULT_SIMULATION_SEED));
            if (Application.isEditor)
            {
                ros = ROSConnection.GetOrCreateInstance();
                ros.RegisterPublisher<UAS_StateMsg>("uas_state");
                ros.Publish("uas_state", new UAS_StateMsg());
                Debug.Break();
                DebugHelper.Setup(Simulation);
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
            Simulation.Update();
        }
        
        void OnDrawGizmos()
        {
            Simulation?.OnDrawGizmos();
            Gizmos.color = Color.black;
            Gizmos.DrawLine(Vector3.zero, new Vector3(0, 40, 0));
        }
        
        void OnApplicationQuit()
        {
            if (Application.isEditor)
            {
                DebugHelper.Teardown();
            }
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
