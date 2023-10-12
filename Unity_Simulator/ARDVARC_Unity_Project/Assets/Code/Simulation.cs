using System.Collections.Generic;
using AALG3.RGVMovementPolicy;
using UnityEngine;
using Random = System.Random;

namespace AALG3
{
    public class Simulation
    {
        public const int DEFAULT_SIMULATION_SEED = 1234;
        internal const float PLAY_AREA_SIZE = 45.72f;
        internal const float PLAY_AREA_MARGIN = 4;

        public SceneManager SceneManager { get; }
        public List<RGV> RGVs { get; }
        public List<Drone> Drones { get; }
        public Wind Wind { get; }
        public Random Random { get; }

        public Simulation(SceneManager sceneManager, Random random)
        {
            SceneManager = sceneManager;
            RGVs = new List<RGV>
            {
                SceneManager.MakeRGV(new Vector3(10, 0, 10), Vector3.left, new Random(~random.Next())),
                SceneManager.MakeRGV(new Vector3(-10, 0, -10), Vector3.right, new Random(~random.Next()))
            };
            Drones = new List<Drone>
            {
                SceneManager.MakeDrone(new Vector3(0, 0, 0), new Random(~random.Next()))
            };
            Wind = new Wind(random.Next());
            Random = random;
        }
    
        public void Update()
        {
            foreach (var rgv in RGVs)
            {
                rgv.Update();
            }
            
            foreach (var drone in Drones)
            {
                drone.Update(this);
                Wind.Push(drone);
            }
        }
        
        public void OnDrawGizmos()
        {
            foreach (var drone in Drones)
            {
                drone.OnDrawGizmos();
                Gizmos.color = Color.magenta;
                Gizmos.DrawRay(drone.GameObject.transform.position, Wind.CurrentWind());
            }
            
            foreach (var RGV in RGVs)
            {
                RGV.OnDrawGizmos();
            }
        }
    }
}
