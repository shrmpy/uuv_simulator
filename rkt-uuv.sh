#!/usr/bin/env bash
set -e

# For reference uuvsimulator.github.io

if [ "$EUID" -ne 0 ]; then
    echo "This script uses functionality which requires root privileges"
    exit 1
fi

# base image 
acbuild --debug begin docker://ubuntu:xenial

# In the event of the script exiting, end the build
trap "{ export EXT=$?; acbuild --debug end && exit $EXT; }" EXIT

# Assign a VNC password with env variable
VNC_PASSWORD=secret

# Enable restricted, universe, and multiverse repos
acbuild --debug run -- /bin/sh -c "apt-get update; apt-get install -y software-properties-common wget;" 
acbuild --debug run -- apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
acbuild --debug run -- /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu xenial main" >/etc/apt/sources.list.d/ros-latest.list'
acbuild --debug run -- /bin/sh -c "wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -"
acbuild --debug run -- /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" >/etc/apt/sources.list.d/gazebo-stable.list'
acbuild --debug run -- add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu/ xenial restricted universe multiverse"
acbuild --debug run -- add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates restricted universe multiverse"

# Install dependencies
acbuild --debug run -- apt-get update
acbuild --debug run -- apt-get install -y \
                    dbus-x11 x11vnc xvfb supervisor \
                    dwm suckless-tools stterm \
                    ros-lunar-desktop \
                    gazebo9 libgazebo9-dev \
                    ros-lunar-gazebo9-* \
                    python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential 

acbuild run -- x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret
acbuild run -- chmod 444 /etc/vncsecret
acbuild port add vnc tcp 5900

# Make the container's entrypoint the supervisord
acbuild copy ./supervisord.conf /etc/supervisor/conf.d/supervisord.conf
acbuild set-exec -- /usr/bin/supervisord -c /etc/supervisor/conf.d/supervisord.conf

# Download UUV
acbuild --debug run -- rosdep init
acbuild --debug run -- rosdep update
acbuild --debug run -- /bin/sh -c 'mkdir -p /root/catkin_ws/src; . /opt/ros/lunar/setup.sh; \
                    rosinstall /root/catkin_ws/src /opt/ros/lunar https://raw.githubusercontent.com/uuvsimulator/uuv_simulator/master/ros_lunar.rosinstall; \
                    . /root/catkin_ws/src/setup.sh; \
                    rosdep install --from-paths /root/catkin_ws/src --ignore-src --rosdistro=lunar -y --skip-keys \
                           "gazebo gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control gazebo_ros_pkgs"; \
                    cd /root/catkin_ws; catkin_make install;' 

##                    . /usr/share/gazebo-9/setup.sh; . /opt/ros/lunar/setup.sh; . /root/catkin_ws/devel/setup.sh;"

# Write the result
acbuild --debug set-name uuv
acbuild --debug label add version 0.0.20
acbuild --debug write --overwrite uuv-0.0.20-linux-amd64.aci

