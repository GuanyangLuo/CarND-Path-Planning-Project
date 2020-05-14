# Code Model for Generating Paths

Setup for this project can be found [here](Udacity_Project_README.md)

---
**Path Planning**

The goals / steps of this project are the following:
* Design a path planner that is able to create smooth, safe paths for the car to follow along a 3 lane highway with traffic
* A successful path planner will be able to keep inside its lane, avoid hitting other cars, and pass slower moving traffic all by using localization, sensor fusion, and map data
* Reflect on your work in a write-up that details how the project was completed / how to generate paths
---

## `main.cpp`

The path planning is done in the following steps:

1. Keep some of the waypoints from the previous path so as to handle latency, and setup the starting point for adding new path
2. Generate predictions from `sensor_fusion` data, assuming other cars are traveling at constant velocity and staying in their lanes
3. Set up a finite state machine (`ego`) using the `Vehicle` class, according to the car's position, velocity, etc at the new path starting point
4. Use the FSM to find a trajectory that gives minimum cost from a set of cost functions in `cost.h` and `cost.cpp`
5. The trajectory is defined by two `Vehicle`: one describing the ego at the start of the trajectory and the other describing the ego at the end of the trajectory (`ego_end`) 
6. Use cubic spline to fit a smooth path connecting the positions of the two `Vehicle` defining the trajectory, with additional "control points" to shape the path to be a smooth continuation of the previous path
7. Calculate the Frenet `s` value for each time step in the transition between the start and end points of the trajectory
8. Obtain the actual waypoints for the path by finding a point in the spline corresponding to the `s` value

## `Vehicle` Class:

Modified from the `Vehicle` class in the "Behavior Planning" lesson, this class provides the following:

1. A finite state machine that tell us what states ego can be in (keep lane, prepare lane change right, lane change left, etc)
2. Methods to compute a trajectory for each state using physics as well as predictions about other cars around ego
3. A method to select the next state for ego, based on the cost of the trajectory for each possible successor state
4. A method to "generate predictions" for the other cars around ego, be calculating each car's physics (position, velocity, etc) at a number of time steps into the future

## `cost.h` and `cost.cpp`

Modified from the `cost.h` and `cost.cpp` in the "Behavior Planning" lesson, these files contain the following:

1. A cost function for how close the trajectory leads to a goal position and lane. This is not super important for this project, but does help with getting ego to the more maneuverable center lane when there are no other cars
2. A cost function for how fast ego can travel along the trajectory
3. A method to add up the costs from each cost function to a weighted cost 
