# CMPUT312 Final Project
A custom implementation of both 
 - 2D Local SLAM using LiDAR: generates a dynamic map and localizes robot in the map
 - Path Planner: navigates the robot in the generated map using A* algorithm

Developed on Clearpath Jackal robot platform.

### Running the Project
This project uses Docker and VSCode Dev Containers to containerize the development process.

- On a machine WITHOUT a NVIDIA GPU:

```docker-compose up -d```

- On a machine WITH a NVIDIA GPU:

```docker-compose -f docker-compose.yml -f docker-compose.gpu.yml up -d```

Once the container is running (you may verify with command ```docker ps```), use VSCode **Dev Containers: Attach to running container...** to open the project
in the newly created Docker container.

To get a shell in the running container:

```docker exec -it project312 /bin/bash```

To stop the container:

```docker-compose down```

To run the gazebo simulation:

```roslaunch jackal_gazebo jackal_world.launch```

To run the Rviz visualization:

```roslaunch jackal_viz view_robot.launch```

### Ceres Solver
This project uses Ceres Solver for optimization in the SLAM implementation.

[Ceres Solver](http://ceres-solver.org/installation.html)
#### Install Ceres Solver dependencies
```
sudo apt-get update
# google-glog + gflags
sudo apt-get install libgoogle-glog-dev libgflags-dev
# Use ATLAS for BLAS & LAPACK
sudo apt-get install libatlas-base-dev
# Eigen3
sudo apt-get install libeigen3-dev
# SuiteSparse (optional)
sudo apt-get install libsuitesparse-dev
```

#### Build and Install Ceres Solver
```
sudo apt-get install libceres-dev
```
 #### Test Ceres Solver installation
 '''
 dpkg -L libceres-dev | head
 '''

I have written the apt into the Dockerfile to install Ceres Solver in the Docker container.

### Rebuild the Docker Container

First stop and remove the existing container:

```docker-compose -f docker-compose.yml -f docker-compose.gpu.yml down```

Then rebuild the container with:

```docker-compose -f docker-compose.yml -f docker-compose.gpu.yml up -d --build```

[Bicubic interpolation](https://en.wikipedia.org/wiki/Bicubic_interpolation)
