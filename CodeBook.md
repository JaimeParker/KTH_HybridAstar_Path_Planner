# Hybrid A star Algorithm Analysis

## 1. Data Flow

in `main.cpp`

```c++
    // init ros node
    ros::init(argc, argv, "a_star");

    // main function
    HybridAStar::Planner hy;
    hy.plan();  // main process

    ros::spin();
    return 0;
```

there are two main parts,
* `HybridAStar::Planner`, use constructor function to initialize an instance of planner
* `HybridAStar::Planner::plan()`, include each sub-part of the main process

### 1.1 Constructor function

firstly, publish a topic named `/move_base_simple/start`, from [ROS WIKI](http://wiki.ros.org/move_base_simple).

message type is `geometry_msgs::PoseStamped`, including 3-D coordinates, [Points](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Point.html)
and pose orientation, [Quaternion](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/Quaternion.html).
And will use a function to define value of this message.

then define several subscribers, including
* `/map`, call function `Planner::setMap`
* `/move_base_simple/goal`, call function `Planner::setGoal`
* `/initialpose`, call function `Planner::setStart`

publish a topic for start point, subscribe topics for start point, goal and map.

then the program goes into `plan()`

### 1.2 `HybridAStar::Planner::plan()`

## 2. Main Functions

### 2.1 ROS and OMPL interface

### 2.2 Map data structure and usage

### 2.3 Dynamic voronoi diagram construction

### 2.4 Hybrid A star algorithm

### 2.5 Reed-Sheep or Dubins

### 2.6 Non-linear optimization

### 2.7 Non-parametric interpolation

## 3. Tricks