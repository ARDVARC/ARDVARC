# How to Run
Run this example with:
```
./run_all.sh
```
# How to Kill
You should be able to stop it with `Ctrl-C` though we've had mixed success. If that doesn't kill everything,
you may need to run
```
ps aux
```
to list all active processes, then
```
kill PID
```
for each of the spawned processes where `PID` is the process ID that you got from `ps aux`.
# Expected Behavior
Running this should start a bunch of nodes that roughly align with the flowchart except most of them will be under `everything_else` because we've crammed it all into one process.

To visualize this, you can run
```
rqt_graph
```
in a new terminal window. I also recommend switching from `Nodes only` to `Nodes/Topic (All)` in the rqt_graph GUI.