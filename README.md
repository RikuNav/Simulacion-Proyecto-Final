# Final Simulation Project: Autonomous Navigation with PuzzleBot

This project demonstrates the creation and simulation of a ROS 2 package focused on autonomous navigation using the PuzzleBot robot in a custom Gazebo Garden environment. The package was created under the name `Mordecai y los Rigbys`.

## ROS 2 Package Creation

A ROS 2 package was created with the following directory structure:


The package was successfully compiled using `colcon build`. Below are screenshots showing:
- The terminal where the package was created.
- The internal structure of the package directory.
- The successful output of the `colcon build` command.

![Pkg Create](/images/pkg_create.png)
![Colcon build](/images/colcon.png)


## Launch File for Simulation

A main launch file was created to start the simulation in Gazebo Garden with the custom track and PuzzleBot. The file is located in the `launch/` directory.

### Command to launch Gazebo with the scenario:

```
ros2 launch mlr_nav2_puzzlebot launch.py
```
![Pista](/images/pista_sim.png)


## RVIZ for Profile Configurations

To aid visualization, two RVIZ profiles were included:

### RVIZ Configuration for Mapping:
```
ros2 launch mlr_nav2_puzzlebot launch.py mode:=map use_sim_time:=True
```
![Map Rviz](/images/map_rviz.jpeg)

### RVIZ Configuration for Navigation:
```
ros2 launch mlr_nav2_puzzlebot launch.py mode:=nav use_sim_time:=True
```
![Nav Rviz](/images/nav_rviz.jpeg)

## System Requirements

    ROS 2 Humble

    Gazebo Garden

## Quick Launch
```
# Launch simulation
ros2 launch mlr_nav2_puzzlebot launch.py

# Launch RVIZ for mapping
ros2 launch mlr_nav2_puzzlebot launch.py mode:=map use_sim_time:=True

# Launch RVIZ for navigation
ros2 launch mlr_nav2_puzzlebot launch.py mode:=nav use_sim_time:=True

```

## Video of our working package

https://www.youtube.com/watch?v=GmPqFrR-eNQ