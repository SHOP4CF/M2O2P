FROM ubuntu:focal

EXPOSE 54000
# We want to be able to source
SHELL ["/bin/bash", "-c"]

# install ros build tools
RUN apt-get update \
	&& apt install curl -y \
	&& apt-get install -y iputils-ping \
	&& apt-get install -y make cmake \
	&& apt-get install -y gcc \
	&& apt-get install -y git \
	&& apt-get install -y libcurl4-openssl-dev \
	&& apt-get install -y libasio-dev \
	&& apt-get install -y libtinyxml2-dev \
	&& apt-get install -y python3-vcstools \
	&& apt-get install -y wget \
	&& apt-get install -y libcurlpp-dev python3-opencv \
	&& apt-get install -y software-properties-common \
	&& apt-get install -y build-essential libyaml-cpp-dev \
	&& apt-get install -y libboost-program-options-dev \
	&& apt-get install -y libcurlpp-dev \
	&& apt-get install -y gnupg2 \
	&& apt-get install -y lsb-release \
	&& apt-get install -y libwebsocketpp-dev

#RUN apt install curl -y
#RUN apt-get install -y iputils-ping
#RUN apt-get install -y make cmake
#RUN apt-get install -y gcc
#RUN apt-get install -y git
#RUN apt-get install -y libcurl4-openssl-dev
#RUN apt-get install -y libasio-dev
#RUN apt-get install -y libtinyxml2-dev
#RUN apt-get install -y python3-vcstools
#RUN apt-get install -y wget
#RUN apt-get install -y libcurlpp-dev python3-opencv
#RUN apt-get install -y software-properties-common
#RUN apt-get install -y build-essential libyaml-cpp-dev
#RUN apt-get install -y libboost-program-options-dev
#RUN apt-get install -y libcurlpp-dev
#RUN apt-get install -y gnupg2
#RUN apt-get install -y lsb-release
#RUN apt install -y libwebsocketpp-dev

#RUN apt-get install -y libfastcdr-dev
RUN rm -rf /var/lib/apt/lists/*
RUN add-apt-repository ppa:deadsnakes/ppa

RUN apt-get update && apt-get install nano

# Updating packages to fix vulnerabilities of the docker image

#RUN apt-get install libnss3=2:3.49.1-1ubuntu1.6
#RUN apt-get install libpq5=12.9-0ubuntu0.20.04.1
#RUN apt-get install icu-devtools=66.1-2ubuntu2.1
#RUN apt-get install libicu-dev=
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install
#RUN apt-get install

# Install ROS2
RUN apt update && apt-get install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt update
RUN apt-get install -y python3-colcon-common-extensions
RUN apt-get install -y python3-distutils
RUN apt-get install -y python3-apt
RUN apt-get install -y python3-pip
RUN apt-get install libpq-dev
RUN pip install psycopg2
#RUN apt-get install python3-distutils --reinstall 
RUN apt-get install -y ros-foxy-desktop
RUN source /opt/ros/foxy/setup.bash
RUN apt-get install -y python3-argcomplete
RUN mkdir -p /home/ros2_ws/src
ENV ROS2_WS /home/ros2_ws
RUN cd /home/ros2_ws/src && git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
#Install demos
#RUN cd ${ROS2_WS}/src \
#	&& git clone --branch foxy https://github.com/ros2/demos.git \
#	&& git clone --branch foxy https://github.com/ros2/examples.git \
#	&& git clone --branch foxy https://github.com/ros2/example_interfaces.git \
#	&& rm -r demos/topic_monitor \
#	&& rm -r demos/intra_process_demo \
#	&& rm -r demos/image_tools \
#	&& rm -r examples/rclcpp \
#	&& rm -r demos/pendulum_control \
#	&& rm -r demos/pendulum_msgs

RUN apt-get update && apt-get upgrade -y

# clone ros package repo
ENV ROS2_WS  /home/ros2_ws
RUN mkdir -p ${ROS2_WS}/src/ac
COPY ./ ${ROS2_WS}/src/ac

WORKDIR ${ROS2_WS}

#RUN mkdir -p ${ROS2_WS}/src/ros2
#RUN cd src/ \
#	&& git clone https://github.com/eProsima/firos2
	
#RUN cd src/firos2/thirdparty \
#	&& git clone https://github.com/chriskohlhoff/asio.git \
#	&& git clone https://github.com/curl/curl.git \
#	&& git clone https://github.com/jpbarrette/curlpp.git \
#	&& git clone https://github.com/leethomason/tinyxml2.git \
#	&& git clone --recursive https://github.com/eProsima/Integration-Service-Deprecated.git \ 
#	&& mv -v /home/ros2_ws/src/firos2/thirdparty/Integration-Service-Deprecated/* /home/ros2_ws/src/firos2/thirdparty/integration-service

# source and build repo with colcon
RUN cd ${ROS2_WS} \
    && source /opt/ros/foxy/setup.bash \
	&& colcon build --symlink-install \
	&& source ${ROS2_WS}/install/setup.bash

WORKDIR ${ROS2_WS}/src/ac/launch
#COPY ./start.bash ${ROS2_WS}/src/ac/src
#COPY ./start.bash ${ROS2_WS}
COPY ./start.bash ${ROS2_WS}/src/ac/launch
RUN chmod +x start.bash

CMD ["./start.bash"]

#CMD ros2 run ac gloves_discretizing_online
#CMD ["bash"]
#ENTRYPOINT bash -c "source /home/ros2_ws/install/setup.bash && ros2 run ac gloves_discretizing_online __log_level:=debug"
#ENTRYPOINT bash -c "source /home/ros2_ws/install/setup.bash && ros2 run ac gloves_discretizing_online"


