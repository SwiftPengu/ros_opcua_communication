# ros_opcua_impl_python_opcua

This package provides OPC UA server implementation for connection to ROS. Server provides access to all ROS topics, services and actions from an  OPC UA Client. 

## Getting started with ros_opcua_impl_python_opcua package

First start some ROS Nodes for something to be visalized in the server. For example one can start `turtlesim_node`:
```
rosrun turtlesim turtlesim_node 
```

Then start the server (the starting will take some time, wait until you see the output) using:
```
roslaunch ros_opcua_impl_python_opcua rosopcua.launch
```

Then start the *UaExpert OPC UA* Client and connect to the server. Server can be accessed per default from the local network on port `21554`. If you are running everything localy use `opc.tcp://localhost:21554` as server URL.

After successful connection you can see all ROS Services, Topics and Actions mapped to the OPC UA. To move the turtle from exaple choose `Objects->ROS-Services->turtle1/teleport_absolute` with right clieck and choose call. Enter the new possition of the turtle and see how turtle moves.

In `Objects->ROS-Topics->turtle1->pose` one can follow the position of the turtle in real time. To check the full effect of this try to move turtle using [Robot Steering](https://wiki.ros.org/rqt_robot_steering) rqt-Plugin.

# Filtering topics
## Explanation
Subscribed and thereby published ROS topics can be filtered by populating the `FILTERS` environment variable.

If no filters are specified, all topics are subscribed and published. If filters are specified, then the filters specify the list of topics to include, with the topics to remove excluded.

Filters are defined as a semicolon-separated list of ROS topic tree segments.
Topic segments can be prepended by '+' (default) or '-' to specify inclusion or exclusion respectively.
Additions are processed first, then removals will happen on that set.

## EBNF

        filters := (filter ';' )* ;
        filter := ( '+' | '-' )? topic ;
        topic := '/' | ('/' IDENTIFIER )+ ;
        IDENTIFIER := [A-Za-z0-9]+ ;


## Examples
* `+/;-/foo` Include everything, except everything under /foo
* `+/foo` Include everything in /foo (and nothing more)
* `/foo` Include everything in /foo (and nothing more)