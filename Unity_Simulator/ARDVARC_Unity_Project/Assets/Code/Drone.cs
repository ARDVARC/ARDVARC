using UnityEngine;

namespace ARDVARC_Unity_Project
{
    public class Drone
    {
        public const float ROTOR_SPEED_TO_FORCE = 0.05f;
        public const float MAX_ROTOR_SPEED = 1000;
        public const float ROTOR_MOMENT_ARM = 0.4f;
        public const float K_M = 10;
        public Vector3 F1Global, F2Global, F3Global, F4Global;
    
        public GameObject GameObject { get; }
        public DroneModel Model { get; }
        public IDroneModelUpdater ModelUpdater { get; }
        public Camera PrimarySensor { get; }
        public Drone(GameObject gameObject, DroneModel model, IDroneModelUpdater modelUpdater, Simulation simulation)
        {
            GameObject = gameObject;
            Model = model;
            ModelUpdater = modelUpdater;
            PrimarySensor = gameObject.transform.GetChild(0).GetComponent<Camera>();
        }

        public void FixedUpdate(Simulation simulation) {
            // Update model
            ModelUpdater.TryUpdateModel(this, simulation);
        
            // Determine desired rotor speeds
            var quadcopterRotorSpeeds = Model.CalculateDesiredRotorSpeeds();
            var multiplier = 1f;
            if (Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F1) < multiplier)
            {
                multiplier = Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F1);
            }
            if (Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F2) < multiplier)
            {
                multiplier = Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F2);
            }
            if (Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F3) < multiplier)
            {
                multiplier = Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F3);
            }
            if (Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F4) < multiplier)
            {
                multiplier = Mathf.Abs(MAX_ROTOR_SPEED / quadcopterRotorSpeeds.F4);
            }
            if (multiplier != 1f)
            {
                quadcopterRotorSpeeds.F1 *= multiplier;
                quadcopterRotorSpeeds.F2 *= multiplier;
                quadcopterRotorSpeeds.F3 *= multiplier;
                quadcopterRotorSpeeds.F4 *= multiplier;
            }
        
            // Do physics
            var gameObjectRigidBody = GameObject.GetComponent<Rigidbody>();
            var gameObjectUp = GameObject.transform.up;
            var gameObjectRight = GameObject.transform.right;
            var gameObjectForward = GameObject.transform.forward;
        
            F1Global = quadcopterRotorSpeeds.F1 * ROTOR_SPEED_TO_FORCE * gameObjectUp;
            F2Global = quadcopterRotorSpeeds.F2 * ROTOR_SPEED_TO_FORCE * gameObjectUp;
            F3Global = quadcopterRotorSpeeds.F3 * ROTOR_SPEED_TO_FORCE * gameObjectUp;
            F4Global = quadcopterRotorSpeeds.F4 * ROTOR_SPEED_TO_FORCE * gameObjectUp;
        
            var droneArm1Global = GameObject.transform.position + Vector3.Normalize(gameObjectRight + gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm2Global = GameObject.transform.position + Vector3.Normalize(gameObjectRight - gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm3Global = GameObject.transform.position + Vector3.Normalize(-gameObjectRight - gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm4Global = GameObject.transform.position + Vector3.Normalize(-gameObjectRight + gameObjectForward) * ROTOR_MOMENT_ARM;
        
            gameObjectRigidBody.AddForceAtPosition(F1Global, droneArm1Global);
            gameObjectRigidBody.AddForceAtPosition(F2Global, droneArm2Global);
            gameObjectRigidBody.AddForceAtPosition(F3Global, droneArm3Global);
            gameObjectRigidBody.AddForceAtPosition(F4Global, droneArm4Global);
            gameObjectRigidBody.AddRelativeTorque(K_M * new Vector3(0, quadcopterRotorSpeeds.F1 - quadcopterRotorSpeeds.F2 + quadcopterRotorSpeeds.F3 - quadcopterRotorSpeeds.F4, 0));
        }
        
        public void OnDrawGizmos()
        {
            Model.OnDrawGizmos();
            
            var gameObjectRight = GameObject.transform.right;
            var gameObjectForward = GameObject.transform.forward;
        
            var droneArm1Global = GameObject.transform.position + Vector3.Normalize(gameObjectRight + gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm2Global = GameObject.transform.position + Vector3.Normalize(gameObjectRight - gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm3Global = GameObject.transform.position + Vector3.Normalize(-gameObjectRight - gameObjectForward) * ROTOR_MOMENT_ARM;
            var droneArm4Global = GameObject.transform.position + Vector3.Normalize(-gameObjectRight + gameObjectForward) * ROTOR_MOMENT_ARM;
            
            Gizmos.color = Color.red;
            Gizmos.DrawRay(droneArm1Global, F1Global);
            Gizmos.color = Color.yellow;
            Gizmos.DrawRay(droneArm2Global, F2Global);
            Gizmos.color = Color.green;
            Gizmos.DrawRay(droneArm3Global, F3Global);
            Gizmos.color = Color.blue;
            Gizmos.DrawRay(droneArm4Global, F4Global);
            Gizmos.color = Color.black;
            Gizmos.DrawRay(GameObject.transform.position, GameObject.transform.up * 10);
            Gizmos.DrawLine(GameObject.transform.position, new Vector3(GameObject.transform.position.x, 0, GameObject.transform.position.z));
        }
    }
}