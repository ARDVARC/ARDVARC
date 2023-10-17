using ARDVARC_Unity_Project;

public interface IDroneModelUpdater
{
    public DroneModel Model { get; }
    public void TryUpdateModel(Drone drone, Simulation simulation);
}