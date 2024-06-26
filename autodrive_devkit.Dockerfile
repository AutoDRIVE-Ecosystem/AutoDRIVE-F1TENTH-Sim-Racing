####################################################
#
#   AutoDRIVE Devkit Dockerfile
#
####################################################

# Set base image
FROM osrf/ros:humble-desktop

# Install display drivers
RUN apt update && apt install -y libgdal-dev ffmpeg libsm6 libxext6

# Install Python
RUN apt update && apt install -y software-properties-common
RUN add-apt-repository ppa:deadsnakes/ppa
RUN apt update && apt install -y python3.8
RUN echo 'alias python3=python3.8' >> ~/.bashrc && . ~/.bashrc
RUN apt update && apt install -y python3-pip

# Install Python dependencies
RUN pip3 install attrdict
RUN pip3 install numpy
RUN pip3 install pillow
RUN pip3 install opencv-contrib-python
RUN pip3 install eventlet==0.33.3
RUN pip3 install Flask==1.1.1
RUN pip3 install Flask-SocketIO==4.1.0
RUN pip3 install python-socketio==4.2.0
RUN pip3 install python-engineio==3.13.0
RUN pip3 install gevent-websocket==0.10.1
RUN pip3 install Jinja2==3.0.3
RUN pip3 install itsdangerous==2.0.1
RUN pip3 install werkzeug==2.0.3
# RUN pip3 install gevent==21.1.2
# RUN pip3 install greenlet==1.0.0
RUN pip3 install transforms3d


# Install ROS dependencies
RUN apt update && apt install -y ros-$ROS_DISTRO-tf-transformations

# Copy AutoDRIVE Devkit (ROS 2 API)
COPY autodrive_devkit /home/autodrive_devkit
RUN cd /home/autodrive_devkit && colcon build
RUN . /home/autodrive_devkit/install/setup.bash

# Set work directory and expose port
WORKDIR /home/autodrive_devkit
EXPOSE 4567