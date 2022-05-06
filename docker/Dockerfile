FROM ros:galactic-ros-base

ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

ENV DEBIAN_FRONTEND noninteractive
ENV WORKSPACE=/workspace

RUN apt-get -qq update && \
    apt-get -qq upgrade && \
    apt-get -qq install apt-utils build-essential vim

RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Install Dependencies
RUN apt-get -qq install ros-galactic-desktop python3-dev python3-opencv python3-wxgtk4.0 python3-pip python3-matplotlib python3-pygame python3-lxml python3-yaml ros-galactic-mavros rsync gazebo11 libgazebo11-dev ros-galactic-gazebo-ros-pkgs python-is-python3

# Install MavProxy
RUN pip install MAVProxy pexpect
RUN pip3 install numpy-stl scikit-image

# Clear apt cache to save on space
RUN rm -rf /var/lib/apt/lists/*

# Setup Geographic dataset
RUN mkdir -p $WORKSPACE/data && \
    cd $WORKSPACE/data && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    chmod 700 ./install_geographiclib_datasets.sh && \
    ./install_geographiclib_datasets.sh

# Install simulation
RUN cd $WORKSPACE && \
    git clone --recurse-submodules -j8 --depth 1 https://github.com/ArduPilot/ardupilot.git

RUN cd $WORKSPACE && \
    git clone --recurse-submodules -j8 --depth 1 https://github.com/SwiftGust/ardupilot_gazebo.git

# Add hummingbird springs as default location (https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html)
RUN echo "HUMMINGBIRD=35.824822,-106.6577015,2036,0" >> $WORKSPACE/ardupilot/Tools/autotest/locations.txt
RUN echo "ARROYO_DEL_OSO=35.138824,-106.556401,1643,0" >> $WORKSPACE/ardupilot/Tools/autotest/locations.txt
RUN echo "BALLOON_FIESTA=35.195006,-106.596133,1528,0" >> $WORKSPACE/ardupilot/Tools/autotest/locations.txt
RUN echo "ROOSEVELT_SPRINGS=38.498443,-112.8528725,230,0" >> $WORKSPACE/ardupilot/Tools/autotest/locations.txt

# Install pexpect for ardupilot build
RUN pip install pexpect

RUN cd $WORKSPACE/ardupilot && \
    ./waf configure && ./waf copter

RUN cd $WORKSPACE/ardupilot_gazebo && \
    mkdir build && cd build && cmake .. && make -j4 && make install

RUN mkdir $WORKSPACE/gazebo
RUN mkdir $WORKSPACE/misc

COPY ./gazebo $WORKSPACE/gazebo
COPY ./misc $WORKSPACE/misc

RUN source /opt/ros/galactic/setup.bash && \
    cd $WORKSPACE/gazebo && \
    colcon build

COPY ./templates $WORKSPACE/templates

# Setup Entrypoint
COPY ./docker/entrypoint.sh /
COPY ./docker/entrypoint.py /

ENTRYPOINT ["/entrypoint.py"]
