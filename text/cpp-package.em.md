The Hello World Package
=======================

Time to create a package which actually does something. This chapter describes the setup and building of a package containing a very simple C++ node. Much of ROS is built with C++, so even if you intend to mostly work in Python, it's worth knowing the basics of how to work with roscpp.

Setting Up
----------

Create, build, and source a new workspace:

~~~bash
$ source /opt/ros/@(rosdistro)/setup.bash
$ mkdir -p ~/cpp_tutorial_ws/src && cd ~/cpp_tutorial_ws
$ catkin_init_workspace src
$ catkin_make
$ source devel/setup.bash
~~~

We're going to create a new package, but this time, we'll specify two packages which it depends on:

~~~bash
$ cd src
$ catkin_create_pkg hello roscpp std_msgs
~~~

You can add more dependencies later on by manually editing the `package.xml` and `CMakeLists.txt` files. But when just starting on new packages, it's great to let the helper tool set it up for you.

The Node Source
---------------

To add a basic node, create a file called `hello_node.cpp` in the package's src directory:

~~~Cpp
#include "ros/ros.h"
#include "std_msgs/String.h"

ros::Publisher pub;
ros::Timer timer;

void timerCallback(const ros::TimerEvent&)
{
  std_msgs::String msg;
  msg.data = "Hello World!";

  pub.publish(msg);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  pub = n.advertise<std_msgs::String>("chatter", 1000);
  timer = n.createTimer(ros::Duration(1.0), timerCallback);

  ROS_INFO("Initialization complete, now spinning.");
  ros::spin();
  return 0;
}
~~~

What's going on here? Not much! Three main elements get initialized as part of the node's setup:

* A `NodeHandle` gets created, which is the interface to node-oriented features of roscpp.
* The `advertise` method of the `NodeHandle` is used to create a publisher, which will be used to publish ROS messages of type `String` to other nodes.
* The `createTimer` method of the `NodeHandle` is used to trigger a periodic callback.

When these setup steps are complete, the node enters a spinning state, where the ROS library continuously services its internal event loop. In a more complicated node, this would mean handling arriving messages, services calls, timers, even user-defined callbacks (for example, from other threads). For now, though, it's just servicing the 1 second timer which has been established.

Building the Package
--------------------

This isn't going to build until we make a few small changes to the CMakeLists.txt file. Open it up, and uncomment the two sections which look like so:

~~~Cmake
# Declare a cpp executable
add_executable(hello_node src/hello_node.cpp)

# Specify libraries to link a library or executable target against
target_link_libraries(hello_node
  ${catkin_LIBRARIES}
)
~~~

That's all there is! Now return to the root of your workspace, and build it:

~~~bash
$ cd ~/cpp_tutorial_ws
$ catkin_make
~~~

If all went well, you should have successfully built the `hello_node` binary.

Running It
----------

The binary which has been built is buried in the `devel/lib/hello/` directory. You should never need to execute a ROS binary manually---it will always be done using the `rosrun` or `roslaunch` tools.

Create a directory it your package for launch files, and create one there, for example:

~~~bash
$ roscd hello
$ mkdir launch
$ touch launch/hello.launch
~~~

Give your launch file the following contents:

~~~xml
<launch>
  <node pkg="hello" type="hello_node" name="hello_node" />
</launch>
~~~

Now launch it:

~~~bash
$ roslaunch hello hello.launch
~~~

If everything has gone well, `roslaunch` has done the following things:

* Detected that `rosmaster` is not running,
* Started `rosmaster`, and
* Executed the `hello_node` binary inside the `hello` package.

ROS is now running in this terminal, so open up a secondary terminal where we can interact with it. If you've added the `source` line to your .bashrc, then this new terminal is already good to go---try these commands:

~~~bash
$ rosnode list
$ rostopic list
$ rostopic echo /chatter
~~~

If these commands can't be found, then first `source /opt/ros/@(rosdistro)/setup.bash` and try again.

What's going on here? First, a list of all running nodes, then a list of available data topics, and finally, a spool of messages being published to the `/chatter` topic, which is what is published-to by the `hello_node` created and launched above.

At this point, you have the basic knowledge required to interact with many basic ROS systems. At its lowest level, ROS is a group of processes communicating by passing messages across topics. 

What about `rosrun`?
--------------------

You can also run package scripts and binaries with `rosrun`, but in most situations, it's better to use `roslaunch`:

* rosrun will find a specified binary within a given package, and execute it, but a `rosmaster` needs to already be running.
* roslaunch uses a `.launch` xml file to specify multiple nodes to be launched. It can also set parameters, define groups and namespaces, work with arguments and environment variables, and include other `launch` files.
* Packages should provide one or more example roslaunch files anyway, so you might as well create them upfront and develop using them.
