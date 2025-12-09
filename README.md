# CMPUT312 Final Project
A custom implementation of both 
 - 2D Local SLAM using LiDAR: generates a dynamic map and localizes robot in the map
 - Path Planner: navigates the robot in the generated map using A* algorithm and Pure Pursuit controller

Developed on Clearpath Jackal robot platform.

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

To launch the Jackal robot in the Gazebo simulation

```roslaunch jackal_gazebo jackal_world.launch config:=front_laser```

To launch the Rviz visualization:

```roslaunch jackal_viz view_robot.launch```

To run our custom SLAM package:

```rosrun slam slam_node```

To run our custom Path Planner package:

```rosrun planning planning_node```

You may now visualize the generated map in RViz. Add the 2D Nav Goal plugin to RViz and request the robot to navigate to that location!