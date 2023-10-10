using AALG3;

public interface IDroneModelUpdater
{
    public DroneModel Model { get; }
    public void TryUpdateModel(Drone drone, Simulation simulation);
}