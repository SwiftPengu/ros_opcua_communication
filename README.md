# Introduction
TNO internal fork of https://github.com/iirob/ros_opcua_communication.git

# Docker image
## General
The docker image hosts the server using Python 2.

* To configure, set the `ROS_MASTER_URI` ENV variable inside the container to the appropriate `roscore` instance.
* By default, the OPC-UA server will be listening on the `opc.tcp://0.0.0.0:21554/` endpoint.

## ENV vars:
* PORT, the port to listen for OPC-UA connections (defaults to `21554`).
* HOST, the hostname/interface to bind to (defaults to all interfaces or `0.0.0.0`).
* NS, the ROS namespace to subscribe to (defaults to `/`).