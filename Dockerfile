# still have to try Python3 on 'stretch' (ROS source instructions assume debian stretch)
FROM ros:melodic-ros-base-stretch


RUN apt-get update && apt-get install -y python-pip build-essential && pip install opcua==0.98.7
RUN apt-get install -y ros-melodic-gazebo-msgs ros-melodic-moveit-msgs
RUN apt-get install ros-melodic-controller-manager-msgs ros-melodic-tf2-msgs ros-melodic-control-msgs

COPY ros_opcua_impl_python_opcua /server

CMD cd /server/scripts && python ros_server.py