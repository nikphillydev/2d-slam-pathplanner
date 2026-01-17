# CMPUT312 Final Project
This projects presents a unified navigation system that executes real-time 2D SLAM and dynamic path planning concurrently. Validated on a Clearpath Jackal ground robot, our project facilitates autonomous exploration with object avoidance as it builds a map of its environment. The subsystems developed in this project are:
 - 2D Local SLAM using LiDAR: a non-linear optimization SLAM backend using Google’s Ceres Solver to fuse data from wheel odometry, an IMU, and a 2D LiDAR to generate a 2D map and localize the robot in the map
 - Path Planner: executes autonomous navigation in the generated map via A* path planning and Pure Pursuit controller to enable frontier exploration with obstacle avoidance

[**Final project report**](https://github.com/nikphillydev/2d-slam-pathplanner/tree/main/final_report).

### Project Organization
```
├── catkin_ws/                                               # catkin workspace (ROS1)
│   ├── scripts/
│   │   └── remote-robot.sh                                  # setup distributed ROS architecture
│   └── src/
│       ├── planning/
│       │   ├── CMakeLists.txt
│       │   ├── include/
│       │   │   └── planning/
│       │   │       ├── AStar.hpp
│       │   │       ├── planning_node.hpp
│       │   │       └── pure_pursuit.hpp
│       │   ├── package.xml
│       │   └── src/
│       │       ├── AStar.cpp                                # A* path planner
│       │       ├── main.cpp                                 # main entrance
│       │       ├── planning_node.cpp                        # Path planning node
│       │       └── pure_pursuit.cpp                         # Pure Pursuit controller
│       └── slam/
│           ├── CMakeLists.txt
│           ├── include/
│           │   └── slam/
│           │       ├── local_slam_node.hpp                        
│           │       ├── residual.hpp                         # Ceres Solver cost function
│           │       └── utils.hpp                            # transformation, log-odds, and clamping helpers
│           ├── msg/
│           │   └── DoubleOccupancyGrid.msg
│           ├── package.xml
│           └── src/
│               ├── local_slam_node.cpp                      # 2D SLAM node
│               └── main.cpp                                 # main entrance
├── docker-compose.gpu.yml
├── docker-compose.yml
├── Dockerfile                                               # docker
├── final_report/
│   └── Local_2D_SLAM_with_Dynamic_Path_Planning.pdf         # project final report
└── README.md
```
### Project Setup
Clone the repo. This project uses Docker and VSCode Dev Containers to containerize the development process.

- On a machine WITHOUT a NVIDIA GPU:

```docker-compose up -d```

- On a machine WITH a NVIDIA GPU (must have nvidia-container-toolkit installed):

```docker-compose -f docker-compose.yml -f docker-compose.gpu.yml up -d```

Once the container is running (you may verify with command ```docker ps```), use VSCode **Dev Containers: Attach to running container...** to open the project in the newly created Docker container:

To get a shell in the running container:

```docker exec -it project312 /bin/bash```

To stop the container:

```docker-compose down```

### Running the Project
Open a shell on your computer (the host machine). Run the following command to allow GUI apps in the Docker container to connect your computer's screen:

```xhost +```

Now open a shell in the running Docker container (command shown above).

To launch the Jackal robot in the Gazebo simulation:

```roslaunch jackal_gazebo jackal_world.launch config:=front_laser```

To launch the Rviz visualization:

```roslaunch jackal_viz view_robot.launch```

To run our custom SLAM package:

```rosrun slam slam_node```

To run our custom Path Planner package:

```rosrun planning planning_node```

You may now visualize the generated map in RViz. Add the 2D Nav Goal plugin to RViz and request the robot to navigate to that location!
