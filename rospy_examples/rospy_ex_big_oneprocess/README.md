# How to Run
Run this example with:
```
./run.sh
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
Running this example should cause a continous cycle of messages to start after about 3 seconds. Specfically,
- node_1 sends a message to topic_1
- node_2 recieves this, and "responds" by sending a message to topic_2
- node_3 recieves this, and "responds" by sending a message to topic_3
- node_4 recieves this, and "responds" by sending a message to topic_4
- node_5 recieves this, and "responds" by sending a message to topic_5
- node_1 recieves this, and "responds" by sending a message to topic_1
- ... and so on

To (kind-of) visualize this, you can run
```
rqt_graph
```
in a new terminal window. I also recommend switching from `Nodes only` to `Nodes/Topic (All)` in the rqt_graph GUI.

Note that because this process only has one node, the rqt_graph visualization isn't that helpful.