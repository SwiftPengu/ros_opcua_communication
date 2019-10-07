# still have to try Python3 on 'stretch' (ROS source instructions assume debian stretch)
FROM ros:melodic-ros-base-stretch

RUN apt-get update && apt-get install -y python-pip && pip install opcua==0.98.7
COPY ros_opcua_impl_python_opcua /server

CMD cd /server/scripts && python ros_server.py