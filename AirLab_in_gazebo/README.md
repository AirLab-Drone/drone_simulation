### Simulate Realsense D435i VSLAM in RTAB-Map  ###


```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws_sim/src/AirLab_in_gazebo/models
```


## launch file
```
launch
    │  
    ├──launch/launch_sim.launch.py      (啟動模擬環境: Gazebo)
    │  
    ├──rtabmap.launch.py                (啟動RTAB-Map)
```