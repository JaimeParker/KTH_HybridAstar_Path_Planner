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

#### 1.1.1 set map

```c++
    grid = map;  // grid: nav_msgs::OccupancyGrid::Ptr
    //update the configuration space with the current map
    configurationSpace.updateGrid(map);
```

`grid` is an instance of `nav_msgs::OccupancyGrid::Ptr`, a pointer in navigation message package.

`configurationSpace` is an instance of class `CollisionDetection`; updating grid is rather simple.

```c++
  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid::Ptr map) {grid = map;}
```

but it seems no difference with line above, why?

```c++
    int height = map->info.height;
    int width = map->info.width;
    bool** binMap;
    binMap = new bool*[width];
```

`binMap`, a pointer to a bool pointer, `binMap = new bool*[width]`, 
using `new` to alloc memory for this, type is `bool*`, a bool type pointer,
with number of `width`.

这一步创建了一个指向指针的指针，并且使用`new`进行内存分配，分配了`width`个类型为`bool*`的变量。

```c++
    for (int x = 0; x < width; x++){
        binMap[x] = new bool[height];
    }
```

now `binMap` is a 2 dimension pointer array with range of `width` * `height`.

```c++
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            binMap[x][y] = map->data[y * width + x] ? true : false;
        }
    }
```

using a `?:` operator, can be simplified as `xxx != 0`;
* if `map->data[y * width + x] = true`, `binMap[x][y] = true`
* if `map->data[y * width + x] = false`, `binMap[x][y] = false`

```c++
    voronoiDiagram.initializeMap(width, height, binMap);
    voronoiDiagram.update();
    voronoiDiagram.visualize();
```

some voronoi diagram stuff, ignore here. `binMap` is used in voronoi diagram.

#### 1.1.2 set start

#### 1.1.3 set goal

then the program goes into `plan()`

### 1.2 `HybridAStar::Planner::plan()`

firstly, define list pointers and initialize lists

```c++
        int width = grid->info.width;
        int height = grid->info.height;
        int depth = Constants::headings;
        int length = width * height * depth;
        // define list pointers and initialize lists
        Node3D* nodes3D = new Node3D[length]();
        Node2D* nodes2D = new Node2D[width * height]();
```

then define the goal point

```c++
        x = start.pose.pose.position.x / Constants::cellSize;
        y = start.pose.pose.position.y / Constants::cellSize;
        t = tf::getYaw(start.pose.pose.orientation);
```

the planning will start

```c++
        // CLEAR THE VISUALIZATION
        visualization.clear();
        // CLEAR THE PATH
        path.clear();
        smoothedPath.clear();
        // FIND THE PATH
        Node3D* nSolution = Algorithm::hybridAStar(
                nStart, nGoal, nodes3D, nodes2D, width, height,
                configurationSpace, dubinsLookup, visualization);
        // TRACE THE PATH
        smoother.tracePath(nSolution);
        // CREATE THE UPDATED PATH
        path.updatePath(smoother.getPath());
        // SMOOTH THE PATH
        smoother.smoothPath(voronoiDiagram);
        // CREATE THE UPDATED PATH
        smoothedPath.updatePath(smoother.getPath());
```

* main part, get an initial path by `Algorithm::hybridAStar`
* send path to smoother, `smoother.tracePath(nSolution)`
* send path to `path`
* smooth path, `smoother.smoothPath(voronoiDiagram);`
* update smoothed path

and the rest will be ROS topic and service stuff.

```c++
        // PUBLISH THE RESULTS OF THE SEARCH
        path.publishPath();
        path.publishPathNodes();
        path.publishPathVehicles();
        smoothedPath.publishPath();
        smoothedPath.publishPathNodes();
        smoothedPath.publishPathVehicles();
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2D, width, height);

        // delete lists
        delete [] nodes3D;
        delete [] nodes2D;
```

but I'm still confused about the data transferring between ROS-rviz and this program.

**Providing a ROS graph here**,
![rqt_graph](images/rosgraph.png)

summarise some questions:

* what is the different between `initialpose` and `move_base_simple/start`?
* in which way the map image is delivered?
* what's the function of `tf` in this ROS program?

Map is sent to ROS master when running the `manual.launch` script, through the `map_server` node.
In detail, by loading a `yaml` file to choose map file, namely a jpg file.

According to the simulation process, user can add start and goal in rviz. 
Meanwhile, data will be transferred to this program, through ROS master and node, but which node?


## 2. Main Functions

### 2.1 ROS and OMPL interface

### 2.2 Map data structure and usage

### 2.3 Dynamic voronoi diagram construction

### 2.4 Hybrid A star algorithm

### 2.5 Reed-Sheep or Dubins

### 2.6 Non-linear optimization

### 2.7 Non-parametric interpolation

## 3. Tricks