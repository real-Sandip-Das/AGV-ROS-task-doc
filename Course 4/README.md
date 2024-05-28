# Lecture Notes for Course 4

- Normally, ROS uses the PC's system clock as time source(a.k.a. ***wall time***)
- For simulations or playback of logged data, it is convenient to work with a simulated time (to enable pause, slow-down etc functionalities)

Working with a simulated clock

- set the `/use_sim_time` parameter:

```bash
#Set the /use_sim_time parameter
rosparam set use_sim_time true
```

- Publish the time on the topic /clock:
  - either from gazebo(enabled by default, because if you're launching gazebo, you must be doing some simulation)
  - from ROS bag (use option `--clock`)

## ROS Time APIs

### `ros::Time`

```cpp
ros::Time begin = ros::Time::now();
double secs = begin.toSec();
```

pretty much self explanatory code snippet

### `ros::Duration`

```cpp
ros::Duration duration(0.5); // 0.5s
ros::Duration passed = ros::Time()::now()-begin;
```

pretty much self explanatory code snippet

### `ros::Rate`

```cpp
ros::Rate rate(10); // 10Hz
```

pretty much self explanatory code snippet

Note: \
if wall time is required, replace `ros::Time`, `ros::Duration`, `ros::Rate` with `ros::WallTime`, `ros::WallDuration` and `ros::WallRate` respectively

## ROS Bags

- File extension: `*.bag`, binary format
- used for recording and storing message data/logs
- intended to be used for visualization, analysis and debugging later on

ROS provides command line tools for recording topics in a bag

```bash
#Recording topics in a bag
rosbag record --all #records all the topics
rosbag record topic_1 topic_2 topic_3 #only records the topics named topic_1, topic_2 and topic_3
#Use Ctrl+C to stop recording
#The recording will be saved with the start date and time as *.bag file name in the current folder

#Show information(e.g. length in time) about a bag
rosbag info bag_name.bag
#Publish the contents of a bag(a.k.a. playback)
rosbag play bag_name.bag
#Publishing with options
#e.g. with a certain rate factor
rosbag play --rate=0.5 bag_name.bag
#or looping the playback
rosbag play --loop bag_name.bag #use Ctrl+C to exit
#or simply using simulated time
rosbag play --clock bag_name.bag
```

## Debugging Strategies

Debugging can be done with strategies already learned in, say, PDS or earlier lectures of this class

- Compile and run code often to catch bugs early on the way to building a bigger application
- Understand compilation and runtime error messages
- Use analysis tools to check data flow(e.g. `rosnode info`, `rostopic echo`, `roswtf`, `rqt_graph` etc.)
- Visualize and plot data (`rviz`, `rqt_multiplot` etc)
- Divide program into smaller steps and check intermediate results by logging (`ROS_INFO`, `ROS_DEBUG` etc, equivalent to print debugging)
- Do argument and return value checks and catch exceptions
- Extend and optimize only when the basic version works
- If it doesn't make sense, do `catkin clean --all`

Debugging using Tools:

- Build in debug mode, and use GDB or Valgrind to Debug

Building in debug mode:

```bash
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

- Use debugger breakpoints in IDEs
- Write unit tests and Integration tests to Discover problems in basic parts of the application

Unit tests:

- Unit tests feed known input to, and expect a known output from the code
- Tests the interface of the code, forces the developer to reason about its contract
- Leads to better designed code

Integration tests:
Checks how any two (typically unit tested) components integrate together

There are two types of Integration tests:

- White Box Integration testing(in most cases, done by developers when they test their own applications)
- Black Box Integration testing(in most cases, done by other people appointed specifically for testing)

Automated tests reduce the risk of bugs before they occur, so it's a really important investment of time before going on to build the whole application
