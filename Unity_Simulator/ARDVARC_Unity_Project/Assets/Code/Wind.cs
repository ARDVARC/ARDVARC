using UnityEngine;

namespace ARDVARC_Unity_Project
{
    public class Wind
    {
        public const float HORIZONTAL_WIND_SPEED_MULTIPLIER = 0; // 8
        public const float VERTICAL_WIND_SPEED_MULTIPLIER = 0; // 4
        public const float MESSY_WIND_SPEED_MULTIPLIER = 0.1f;
        public const float CALM_WIND_NOISE_SAMPLE_RATE = 0.01f;
        public const float MESSY_WIND_NOISE_SAMPLE_RATE = 1;
        
        public long SimulationSeed { get; }

        public Wind(long simulationSeed)
        {
            SimulationSeed = simulationSeed;
        }
        
        public void Push(Drone drone)
        {
            drone.GameObject.GetComponent<Rigidbody>().AddForce(Time.deltaTime * CurrentWind());
        }
        
        public Vector3 CurrentWind()
        {
            var windSpeedX = (Mathf.PerlinNoise1D(SimulationSeed + Time.timeSinceLevelLoad * CALM_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * HORIZONTAL_WIND_SPEED_MULTIPLIER;
            windSpeedX += (Mathf.PerlinNoise1D(SimulationSeed + 500 + Time.timeSinceLevelLoad * MESSY_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * MESSY_WIND_SPEED_MULTIPLIER * HORIZONTAL_WIND_SPEED_MULTIPLIER;
            var windSpeedY = (Mathf.PerlinNoise1D(SimulationSeed + 1000 + Time.timeSinceLevelLoad * CALM_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * VERTICAL_WIND_SPEED_MULTIPLIER;
            windSpeedY += (Mathf.PerlinNoise1D(SimulationSeed + 1500 + Time.timeSinceLevelLoad * MESSY_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * MESSY_WIND_SPEED_MULTIPLIER * VERTICAL_WIND_SPEED_MULTIPLIER;
            var windSpeedZ = (Mathf.PerlinNoise1D(SimulationSeed + 2000 + Time.timeSinceLevelLoad * CALM_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * HORIZONTAL_WIND_SPEED_MULTIPLIER;
            windSpeedZ += (Mathf.PerlinNoise1D(SimulationSeed + 2500 + Time.timeSinceLevelLoad * MESSY_WIND_NOISE_SAMPLE_RATE) - 0.5f)*2 * MESSY_WIND_SPEED_MULTIPLIER * HORIZONTAL_WIND_SPEED_MULTIPLIER;
            return new Vector3(windSpeedX, windSpeedY, windSpeedZ);
        }
    }
}
