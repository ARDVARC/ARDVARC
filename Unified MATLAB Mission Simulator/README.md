## Getting Started
### Compiling
To compile the simulator, double click on `main.prj` and step through the MATLAB Coder app. This should create a `main_mex.mex<os>` file, where `<os>` depends on your operating system.
### Running
To run the simulator, type the following command into the MATLAB command window:
```
main_mex(SimParams())
```
You can also specify different simulation parameters to make minor tweaks to the simulation. The `startup.m` script should have added a couple of example SimParams objects to the workspace.

Example:
```
main_mex(sp_1_2)
```

Read through `Simulation/SimParams.m` to see what options can be easily changed.