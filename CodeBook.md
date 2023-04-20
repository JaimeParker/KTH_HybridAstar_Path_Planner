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

#### 1.1.2 set goal

```c++
    // retrieving goal position
    float x = end->pose.position.x / Constants::cellSize;
    float y = end->pose.position.y / Constants::cellSize;
    float t = tf::getYaw(end->pose.orientation);

    std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
        validGoal = true;
        goal = *end;

        if (Constants::manual){
            plan();
        }
    } 
```

why call `plan()` here?

#### 1.1.3 set start

```c++
void Planner::setStart(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& initial) {
    float x = initial->pose.pose.position.x / Constants::cellSize;
    float y = initial->pose.pose.position.y / Constants::cellSize;
    float t = tf::getYaw(initial->pose.pose.orientation);
}
```

set coordinates of start.

```c++
    geometry_msgs::PoseStamped startN;
    startN.pose.position = initial->pose.pose.position;
    startN.pose.orientation = initial->pose.pose.orientation;
    startN.header.frame_id = "map";
    startN.header.stamp = ros::Time::now();

    if (grid->info.height >= y && y >= 0 && grid->info.width >= x && x >= 0) {
        validStart = true;
        start = *initial;
    
        if (Constants::manual) { plan();}
    
        // publish start for RViz
        pubStart.publish(startN);
    }
```

if start is valid, call `plan()` here, and publish start information to topic `"/move_base_simple/start"`.

but I'm confused why `plan()` need to be called here, even it will be called in the next step in `main`?

before the program running into `ros::spin()`, message of start and goal is sent by publisher, 

then the program goes into `plan()`

### 1.2 `Planner::plan()`

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

Summarise some questions:

* what is the different between `initialpose` and `move_base_simple/start`?
* in which way the map image is delivered?
* what's the function of `tf` in this ROS program?

Map is sent to ROS master when running the `manual.launch` script, through the `map_server` node.
In detail, by loading a `yaml` file to choose map file, namely a jpg file.

According to the simulation process, user can add start and goal in rviz. 
Meanwhile, data will be transferred to this program, through ROS master and node, but which node?

### 1.3 Overall Analysis

我为什么又写了这么一节？因为耽搁了几天回来再看的时候，我忘了我写到哪了，就记得整体的数据流还没分析清楚。
上面的1.1， 1.2两个小节也提出了一些问题，在此做一个整汇总。

So in `main.cpp`, firstly create an instance of `Planner` class.
As we know about c++ feature, this will lead to the call of constructor function by this class.

According to analysis above, we know that in the constructor function, several publisher and subscriber are defined.
* publisher: "/move_base_simple/start"
* subscriber: "/map", callback function is `setMap`
* subscriber: "/move_base_simple/goal", callback function is `setGoal`
* subscriber: "/initialpose", callback function is `setStart`

As stated by ROS WIKI, only when the program reaches `ros::spin()` can the
subscribers start to receive message and callback their functions.
So process above only name some variables.

Then the program goes to `plan()`.
However, without valid start and goal point, this will not be proceeded.

Now it's time for `ros::spin()`, if user provide start and goal information to `RVIZ`,
the message will be received by those subscribers, and callback functions will be called.

In callback functions `setStart` and `setGoal`, `plan()` shall be called again and again
once when the start or goal is set or changed.

Until now, data flow is basically clear.


## 2. Main Functions

### 2.1 ROS and OMPL interface

### 2.2 Map data structure and usage

### 2.3 Dynamic voronoi diagram construction

### 2.4 Hybrid A star algorithm

This part is in class `algorithm.cpp` and `algorithm.h`.

Got 4 functions(one class member) and a struct.
* float `aStar`
* void `updateH`
* Node3D* `dubinsShot`
* Node3D* `Algorithm::hybridAStar`
* struct `CompareNodes`

#### 2.4.1 Node for 3D and 2D

The Node class by author is defined to describe each node of hybrid A star node, 
just like A star node with information of `x,y,g,f`.

The difference between 2D and 3D is whether to take $\theta$ into consideration.

In `Node2D.h and cpp`, there are several functions:

* `isOnGrid`, if this node is inside the map
* `createSuccessor(const int i)`, set the next node according to i, which has 8 options.
* `bool Node2D::operator == (const Node2D& rhs) const`, return if the same position
* `setIdx`, set id in a one dimension array

In `Node3D.h` and `Node3D.cpp`, there are more functions:

* `isOnGrid`, x and y is inside the map, and heading angle is less than 72 degree(defined by the author)
* `isInRange`, if is in the range of test Dubins shot
* `createSuccessor`

```c++
// R = 6, 6.75 DEG
const float Node3D::dy[] = { 0,        -0.0415893,  0.0415893};
const float Node3D::dx[] = { 0.7068582,   0.705224,   0.705224};
const float Node3D::dt[] = { 0,         0.1178097,   -0.1178097};
```

<img src="images/node3d.jpg" alt="node3d" style="zoom:23%;" />

```c++
bool Node3D::isInRange(const Node3D& goal) const {
    int random = rand() % 10 + 1;
    // rand(), Return a random integer between 0 and RAND_MAX inclusive
    // #define	RAND_MAX	2147483647
    float dx = std::abs(x - goal.x) / random;
    float dy = std::abs(y - goal.y) / random;
    return (dx * dx) + (dy * dy) < Constants::dubinsShotDistance;
}
```

random?

```c++
Node3D* Node3D::createSuccessor(const int i) {
    float xSucc;
    float ySucc;
    float tSucc;

    // calculate successor positions forward
    if (i < 3) {
        xSucc = x + dx[i] * cos(t) - dy[i] * sin(t);
        ySucc = y + dx[i] * sin(t) + dy[i] * cos(t);
        tSucc = Helper::normalizeHeadingRad(t + dt[i]);
    }
    // backwards
    else {
        xSucc = x - dx[i - 3] * cos(t) - dy[i - 3] * sin(t);
        ySucc = y - dx[i - 3] * sin(t) + dy[i - 3] * cos(t);
        tSucc = Helper::normalizeHeadingRad(t - dt[i - 3]);
    }

    return new Node3D(xSucc, ySucc, tSucc, g, 0, this, i);
}
```

<img src="images/update3d.jpg" alt="update" style="zoom:20%;" />

```c++
static inline float normalizeHeadingRad(float t) {
  if (t < 0) {
    t = t - 2.f * M_PI * (int)(t / (2.f * M_PI));
    return 2.f * M_PI + t;
  }

  return t - 2.f * M_PI * (int)(t / (2.f * M_PI));
}
```

just a normalize function if $t<0$;

```c++
void Node3D::updateG() {
    // forward driving
    if (prim < 3) {
        // if i < 3
        // penalize turning
        // if the direction changed from last motion
        if (pred->prim != prim) {
            // penalize change of direction, from backward to forward
            if (pred->prim > 2) {
                // penaltyTurning = 10.5, penaltyCOD = 2.0
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;
            } else {
                // if still forward, just a minor direction changing
                // penaltyTurning = 10.5
                g += dx[0] * Constants::penaltyTurning;
            }
        } else {
            g += dx[0];
        }
    }
    // reverse driving
    else {
        // penalize turning and reversing
        if (pred->prim != prim) {
            // penalize change of direction
            if (pred->prim < 3) {
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
            } else {
                g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
            }
        } else {
            g += dx[0] * Constants::penaltyReversing;
        }
    }

}
```

for `updateG` function, take forward motion as an example,

* if the motion of last step is the same as this one, $g=g+dx[0]$
* else, will add other penalty
  * if the two motion is both forward, add a slight penalty
  * else, add a heavier penalty

I doubt about whether this method can satisfy dynamic constraint.

#### 2.4.2 Dubins

`dubinsShot` seems to be the one out of some complex data structure, so let's analyse it secondly.




### 2.5 Non-linear optimization

### 2.6 Non-parametric interpolation

## 3. Tricks