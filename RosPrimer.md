# ROS Primer



# ROS Contents

The Robotic Operationg Sysstem ROS provides

- drivers to access hardware
- a message passing system (rosmessages being pushed to and read from topics)
- a parameter handling system (rosparams being set in a central store and read/written by nodes)
- the concept of a _node_: A small unit of functionality where different nodes communicate with each other using messages
- a set of default interfaces so that different nodes can understand each other
- command line utilities to help understand, what's going on withitn the ROS system
- visualization tools (same purpose)
- a build and dependency management system (`catkin`)
- tooling to start up complex sets of nodes to perform more complex tasks



# Namespaces

To start a node in a namespace, run

    ROS_NAMESPACE=ns rosrun package node

If a node normally publishes to `foo`, it will now be publishing to `/ns/foo`.



# Remap Topics

Remap topics for a node when launching:

    rosrun package node name:=new_name

will now publish to `new_name`, if it internally used the name `name`.



# Set parametesr

set globally:

    rosparam set name value

set for a node:

    rosrun package node _name:=value
