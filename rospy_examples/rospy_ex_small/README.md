# How to Run
Run this example with:
```
python3 rospy_ex.py
```
You should be able to stop it with `Ctrl-C` though we've had mixed success. If that doesn't kill everything,
you may need to run
```
ps aux
```
to list all active processes, then
```
kill PID
```
for the Python process where `PID` is the process ID that you got from `ps aux`.