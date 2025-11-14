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
