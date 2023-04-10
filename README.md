# ERC2021-Maintenance-Task

These codes were written in python using MoveIt framework and Robot Operating System (ROS1 Noetic). Clone the repository for launching the simulation, and see the UR3 arm perform various tasks.

Launch the simulation:
```
roslaunch simulation simulator.launch
```

Search Patern:
```
roslaunch simulation objective1.launch
```

Push a sequence of buttons:
```
roslaunch simulation objective2.launch sequence:="string of 4 buttons"
```

Pick up Sensor:
```
roslaunch simulation objective3.launch
```

Place Sensor:
```
roslaunch simulation objective4.launch
```

Pick up inspection panel cover:
```
roslaunch simulation objective5.launch
```

Scan ArUco Marker inside inspection panel:
```
roslaunch simulation objective6.launch
```

Push Button Corresponding to Hidden Tag:
```
roslaunch simulation objective7.launch
```

Close Inspection Panel:
```
roslaunch simulation objective8.launch
```

Home Position:
```
roslaunch simulation objective9.launch
```

## Watch the simulation on youtube: 
### Qualification B Submission | European Rover Challenge 2021: https://youtu.be/pOXV26B_EI0?t=84

