### Simulate Realsense D435i VSLAM in RTAB-Map  ###

啟動虛擬環境
```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/ros2_ws/src/AirLab_in_gazebo/models
ros2 launch AirLab_in_gazebo launch_sim.launch.py 
```
啟動rtabmap
```
ros2 launch AirLab_in_gazebo rtabmap.launch.py    
```


## Pakage structure
```
AirLab_in_gazebo
│
├── configs                                     
│   ├── gazebo_params.yaml                  
│   └── sim.rviz                            
│
├── description                             (機器人描述檔)     
│   ├── camera.xacro                        
│   ├── _d435i.gazebo copy.xacro            
│   ├── _d435i.gazebo.xacro                 
│   ├── _d435i.urdf.xacro                   
│   ├── depth_camera.xacro                  
│   ├── gazebo_control.xacro                
│   ├── imu.xacro                           
│   ├── inertial_macros.xacro               
│   ├── robot_core.xacro                    
│   └── robot.urdf.xacro                    
│
├── launch
│   ├── gazebo_garden.launch.py             (Gazebo garden version test)
│   ├── launch_sim.launch.py                (啟動模擬環境: Gazebo)
│   ├── nav2.launch.py                      (navigation2 test)
│   └── rtabmap.launch.py                   (啟動RTAB-Map and rviz2)
│
├── map                                     (導航地圖)
│   
├── models                                  (gazebo models)
│
├── worlds                                  (gazebo worlds)
│   ├── empty_world.model                   
│   ├── empty_world.world                   
│   └── my_house.world                      
│
├── CMakeLists.txt
├── README.md
└── package.xml

```