# Villa Manipulation Package

This package contains (or will contain) modular actionlib servers for common manipulation activities (picking up, placing, etc).

## Implemented Functionalities

* grasp\_server - simple implementation for picking up an object off of a table. (in progress)


## Give pose & Receive pose
```
roslaunch villa_manipulation hri.launch
```


## Demo for opening door

Robot should be localized first w.r.t map - laucnh full-version rviz and use pose_estimation 

### Call service server
 
```
roslaunch villa_manipulation open_door_task.launch
```
This contains three servers including
 - handle detector service server
 - opening door service server
 - navigation waypoint server


## Call client for service

```
rosrun villa_manipulation nav_open_door_client.py
```
