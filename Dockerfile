FROM ubuntu:xenial

ARG VNC_PASSWORD=secret
ENV VNC_PASSWORD=${VNC_PASSWORD} \
    HOME=/root 
WORKDIR ${HOME}

# Allow universe and multiverse repos
RUN apt-get update; apt-get install -y software-properties-common wget; \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116; \
    echo "deb http://packages.ros.org/ros/ubuntu xenial main" >/etc/apt/sources.list.d/ros-latest.list; \
    wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -; \
    echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable xenial main" >/etc/apt/sources.list.d/gazebo-stable.list; \
    add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu/ xenial restricted universe multiverse"; \
    add-apt-repository "deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates restricted universe multiverse";

# Install dependencies
RUN apt-get update; apt-get install -y \
                    dbus-x11 x11vnc xvfb supervisor \
                    dwm suckless-tools stterm \
                    ros-lunar-desktop \
                    gazebo9 libgazebo9-dev \
                    ros-lunar-gazebo9-* \
                    python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential; \ 
    rosdep init; \
    mkdir -p /etc/supervisor/conf.d; \
    x11vnc -storepasswd $VNC_PASSWORD /etc/vncsecret; \
    chmod 444 /etc/vncsecret; 

#    apt-get autoclean; \
#    apt-get autoremove; \
#    rm -rf /var/lib/apt/lists/*; 

COPY supervisord.conf /etc/supervisor/conf.d
EXPOSE 5900
CMD ["/usr/bin/supervisord","-c","/etc/supervisor/conf.d/supervisord.conf"]

# Download UUV (todo as non-root)
RUN rosdep update; \
    mkdir -p /root/catkin_ws/src; . /opt/ros/lunar/setup.sh; \
    rosinstall /root/catkin_ws/src /opt/ros/lunar https://raw.githubusercontent.com/uuvsimulator/uuv_simulator/master/ros_lunar.rosinstall; \
    . /root/catkin_ws/src/setup.sh; \
    rosdep install --from-paths /root/catkin_ws/src --ignore-src --rosdistro=lunar -y --skip-keys \
                           "gazebo gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control gazebo_ros_pkgs"; \
    cd /root/catkin_ws; catkin_make install;

##                    . /usr/share/gazebo-9/setup.sh; . /opt/ros/lunar/setup.sh; . /root/catkin_ws/devel/setup.sh;"

