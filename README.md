# Path Planning Project

I completed this project as part of [Udacity](https://www.udacity.com)'s [Self-driving Car Engineer Nanodegree](https://www.udacity.com/course/self-driving-car-engineer-nanodegree--nd013). 

The goal of this project is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. 

We are provided with the car's localization and sensor fusion data. There is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible. 

Note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. 

The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

The map of the highway is in [highway_map.csv](data/highway_map.csv).
Each waypoint in the list contains `[x,y,s,dx,dy]` values. `x` and `y` are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the `dx` and `dy` values define the unit normal vector pointing outward of the highway loop.

This is a sample extract from [highway map data](data/highway_map.csv) showing what the data looks like:

```text
...
784.6001 1135.571 0 -0.02359831 -0.9997216
815.2679 1134.93 30.6744785308838 -0.01099479 -0.9999396
844.6398 1134.911 60.0463714599609 -0.002048373 -0.9999979
875.0436 1134.808 90.4504146575928 -0.001847863 -0.9999983
905.283 1134.799 120.689735412598 0.004131136 -0.9999915
934.9677 1135.055 150.375551223755 0.05904382 -0.9982554
...
```

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

# Details
The car uses a perfect controller and will visit every `(x,y)` point it recieves in the list every .02 seconds. The units for the `(x,y)` points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The `(x,y)` point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. `previous_path_x`, and `previous_path_y` can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.


# Project Specification

## Compilation
**The code compiles correctly.**

Code must compile without errors with cmake and make.

## Valid Trajectories
**The car is able to drive at least 4.32 miles without incident.**

The top right screen of the simulator shows the current/best miles driven without incident. Incidents include exceeding acceleration/jerk/speed, collision, and driving outside of the lanes. Each incident case is also listed below in more detail.

**The car drives according to the speed limit.**

The car doesn't drive faster than the speed limit. Also the car isn't driving much slower than speed limit unless obstructed by traffic.

**Max Acceleration and Jerk are not Exceeded.**

The car does not exceed a total acceleration of 10 m/s^2 and a jerk of 10 m/s^3.

**Car does not have collisions.**

The car must not come into contact with any of the other cars on the road.

**The car stays in its lane, except for the time between changing lanes.**

The car doesn't spend more than a 3 second length out side the lane lanes during changing lanes, and every other time the car stays inside one of the 3 lanes on the right hand side of the road.

**The car is able to change lanes**

The car is able to smoothly change lanes when it makes sense to do so, such as when behind a slower moving car and an adjacent lane is clear of other traffic.

## Reflection

**There is a reflection on how to generate paths.**

The code model for generating paths is described in detail. This can be part of the README or a separate doc labeled "Model Documentation".

# Design Notes

## Highway Map

### About the Highway
* The [map file](data/highway_map.csv) contains a list of waypoints that go all the way around the track. 
* The track contains a total of 181 waypoints, with the last waypoint mapping back around to the first. The waypoints are in the middle of the double-yellow dividing line in the center of the highway.
* The track is 6945.554 meters around (about 4.32 miles). If the car averages near 50 MPH, then it should take a little more than 5 minutes for it to go all the way around the highway.
* The highway has 6 lanes total - 3 heading in each direction. Each lane is 4 m wide and the car should only ever be in one of the 3 lanes on the right-hand side. The car should always be inside a lane unless doing a lane change.

### Waypoints and Highway Measurements
Each waypoint has an `(x,y)` global map position, and a Frenet s value and Frenet d unit normal vector (split up into the `x` component, and the `y` component).

The `s` value is the distance along the direction of the road. The first waypoint has an s value of `0` because it is the starting point.

The `d` vector has a magnitude of `1` and points perpendicular to the road in the direction of the right-hand side of the road. The `d` vector can be used to calculate lane positions. For example, if you want to be in the left lane at some waypoint just add the waypoint's `(x,y)` coordinates with the `d` vector multiplied by 2. Since the lane is 4 m wide, the middle of the left lane (the lane closest to the double-yellow dividing line) is 2 m from the waypoint.

If you would like to be in the middle lane, add the waypoint's coordinates to the `d` vector multiplied by 6 = (2+4), since the center of the middle lane is 4 m from the center of the left lane, which is itself 2 m from the double-yellow dividing line and the waypoints.
### Converting Frenet Coordinates

There is a helper function, `getXY()`, in [helpers.h](src/helpers.h) which takes in Frenet `(s,d)` coordinates and transforms them to `(x,y)` coordinates.

### Interpolating Points

To estimate the location of points between the known waypoints, we need to "interpolate" the position of those points.

Once we have a polynomial function, we can use it to interpolate the location of a new point.

There are also other methods we could use. For example, Bezier curve fitting with control points, or spline fitting, which guarantees that the generated function passes through every point.




# Building and running the project

## Code Style

This project employs [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Code Structure
The content of [src](src) directory is as follows:

```
src
 |-- Eigen-3.3/
 |-- helpers.h
 |-- json.hpp
 |-- main.cpp
```


## Important Dependencies

* cmake >= 3.19
    * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
    * Linux: make is installed by default on most Linux distros
    * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
    * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
    * Linux: gcc / g++ is installed by default on most Linux distros
    * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
    * Windows: recommend using [MinGW](http://www.mingw.org/)
* uWebSockets
    * Set up and install [uWebSocketIO](https://github.com/uWebSockets/uWebSockets)
    * Note: the branch `e94b6e1` is the version of `uWebSocketIO` that works with the Udacity simulator
* C++ cubic spline interpolation
    * The latest version is [from GitHub](https://github.com/ttk592/spline/)  

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
    * On Windows, you may need to run: `cmake .. -G "Unix Makefiles" && make`
4. Run it: `./path_planning`


# Some Example Output

This is a **bad** example where the car just drives in circles. There are lots of collisions, and the car exceeds acceleration, speed, and jerk limits. 

![Driving in Circles](videos/driving_in_circles.gif)

# References

* Tino Kluge. [Cubic Spline interpolation in C++](https://kluge.in-chemnitz.de/opensource/spline/). kluge.in-chemnitz.de/.
