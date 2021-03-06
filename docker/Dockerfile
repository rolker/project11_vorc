# Ubuntu 18.04 with nvidia-docker2 beta opengl support.
ARG BASEIMG=ubuntu:bionic
FROM $BASEIMG

# Set ROS distribution
ARG DIST=melodic

# Set Gazebo verison
ARG GAZ=gazebo9

# Tools useful during development.
RUN apt update \
 && apt install -y \
        build-essential \
        cppcheck \
        curl \
        cmake \
        lsb-release \
        g++-6 \
        gcc-6 \
        gdb \
        git \
        mercurial \
        python-future \
        python-gdal \
        python-pip \
        python-ipdb \
        python3-dbg \
        python3-pip \
        python3-venv \
        ruby \
        software-properties-common \
        sudo \
        vim \
        wget \
        libeigen3-dev \
        pkg-config \
        protobuf-compiler \
        xterm \
 && apt clean

RUN sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-6 10

RUN sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-6 10
 
RUN export DEBIAN_FRONTEND=noninteractive \
 && apt update \
 && apt install -y \
    tzdata \
 && ln -fs /usr/share/zoneinfo/America/Los_Angeles /etc/localtime \
 && dpkg-reconfigure --frontend noninteractive tzdata \
 && apt clean

# Get ROS melodic and Gazebo 9.
COPY docker/keys/gazebo.key /tmp/gazebo.key
COPY docker/keys/ros.key /tmp/ros.key
RUN /bin/sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
 && apt-key add /tmp/ros.key \
 && /bin/sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" > /etc/apt/sources.list.d/gazebo-stable.list' \
 && apt-key add /tmp/gazebo.key \
 && apt update \
 && apt install -y \
    python-rosdep \
    python-catkin-tools \
    qtbase5-dev \
    qtpositioning5-dev \
    libqt5svg5-dev \
    libgeos++-dev \
    libgles2-mesa-dev \
    ${GAZ} \
    lib${GAZ}-dev \
    ros-${DIST}-ros-base \
    ros-${DIST}-xacro \
    ros-${DIST}-gazebo-ros \
    ros-${DIST}-gazebo-plugins \
    ros-${DIST}-geodesy \
    ros-${DIST}-grid-map \
    ros-${DIST}-global-planner \
    ros-${DIST}-gmapping \
    ros-${DIST}-hector-gazebo-plugins \
    ros-${DIST}-image-view \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joy \
    ros-${DIST}-joy-teleop \
    ros-${DIST}-key-teleop \
    ros-${DIST}-mavros \
    ros-${DIST}-move-base \
    ros-${DIST}-octomap-server \
    ros-${DIST}-octovis \
    ros-${DIST}-pid \
    ros-${DIST}-pointcloud-to-laserscan \
    ros-${DIST}-robot-localization \
    ros-${DIST}-robot-state-publisher \
    ros-${DIST}-rosmon \
    ros-${DIST}-rviz \
    ros-${DIST}-smach-ros \
    ros-${DIST}-stereo-image-proc \
    ros-${DIST}-teleop-tools \
    ros-${DIST}-teleop-twist-keyboard \
    ros-${DIST}-tf2-ros \
    ros-${DIST}-tf2-geometry-msgs \
    ros-${DIST}-tf-conversions \
    ros-${DIST}-velodyne-simulator \
 && rosdep init \
 && apt clean

RUN rosdep update

RUN pip install filterpy

# Set USER and GROUP
ARG USER=developer
ARG GROUP=developer

# Add a user with the same user_id as the user outside the container
# Requires a docker build argument `user_id`.

RUN curl -SsL https://github.com/boxboat/fixuid/releases/download/v0.4/fixuid-0.4-linux-amd64.tar.gz | tar -C /usr/local/bin -xzf - && \
    chown root:root /usr/local/bin/fixuid && \
    chmod 4755 /usr/local/bin/fixuid && \
    mkdir -p /etc/fixuid && \
    printf "user: $USER\ngroup: $GROUP\n" > /etc/fixuid/config.yml

RUN addgroup --gid 1000 $USER && \
    adduser --uid 1000 --ingroup $USER --home /home/$USER --shell /bin/sh --disabled-password --gecos "" $USER

RUN adduser $USER sudo \
 && echo "$USER ALL=NOPASSWD: ALL" >> /etc/sudoers.d/$USER

 
# Commands below run as the developer user.
USER $USER:$GROUP

# When running a container start in the developer's home folder.
WORKDIR /home/$USER

# Create workspace
RUN mkdir -p project11

# Copy the VRX repository from the local file system
# We can't use the USER:GROUP variables until Docker adds support to --chown
COPY --chown=developer:developer . project11/

RUN sudo chmod -R a+rwx /root/.ros
RUN sudo ln -s /home/developer/.ros/log /root/.ros/log

COPY --chown=developer:developer  catkin_ws/src/local/project11_vorc/docker/ros_entrypoint.sh /

# install opencv (previously cmake'ed and make'd)
RUN /bin/bash -c "cd project11/catkin_ws/src/local/opencv_dnn/opencv-build && sudo make install"

# Compile the VRX project.
RUN /bin/bash -c ". /opt/ros/${DIST}/setup.bash && cd project11/catkin_ws && catkin build"


# Source all the needed environment files.
RUN /bin/sh -c 'echo ". /opt/ros/${DIST}/setup.bash" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". /usr/share/gazebo/setup.sh" >> ~/.bashrc' \
 && /bin/sh -c 'echo ". ~/project11/catkin_ws/devel/setup.sh" >> ~/.bashrc'

ENTRYPOINT ["fixuid"]

CMD ["/ros_entrypoint.sh"]

#CMD ["/bin/bash"]
# Customize your image here.


# ...
