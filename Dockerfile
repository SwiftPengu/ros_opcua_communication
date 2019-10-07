# still have to try Python3 on 'stretch' (ROS source instructions assume debian stretch)
FROM ros:melodic-ros-base-stretch

# Required to correctly install rospy
# ENV ROS_PYTHON_VERSION 3
RUN apt-get update && apt-get install -y python-pip && pip install opcua==0.98.7
# RUN apt-get update && apt-get install -y dialog python3-rospy # python3 python3-wheel python3-setuptools python3-pip
# RUN rosdep init && rosdep update

COPY ros_opcua_impl_python_opcua /server
# RUN rosdep install --os=debian:stretch --from-paths /server --ignore-src --rosdistro melodic -y

CMD cd /server/scripts && python ros_server.py