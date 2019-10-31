# Introduction
TNO fork of https://github.com/iirob/ros_opcua_communication.git

This fork adds the following:
* Support for ros-melodic
* Various bugfixes, tested on the Franka Panda robot
* Some refactoring to make code easier to read an debug

# Docker image
## General
The docker image hosts the server using Python 2.

* To configure, set the `ROS_MASTER_URI` ENV variable inside the container to the appropriate `roscore` instance.
* By default, the OPC-UA server will be listening on the `opc.tcp://0.0.0.0:21554/` endpoint.

# Environment variables:
* PORT, the port to listen for OPC-UA connections (defaults to `21554`).
* HOST, the hostname/interface to bind to (defaults to all interfaces or `0.0.0.0`).
* NS, the ROS namespace to subscribe to (defaults to `/`).

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