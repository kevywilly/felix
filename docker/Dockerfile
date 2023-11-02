FROM humble:r32.7.4 as stage1

WORKDIR /

ENV ROS_ROOT=/opt/ros/humble
ENV ROS_DISTRO=humble

WORKDIR /

#COPY dependencies/rosmaster /rosmaster
COPY ./felix.sh /

#WORKDIR /rosmaster

#RUN python3 setup.py install
#RUN cd ../ && rm -rf /rosmaster

RUN apt-get update && apt-get -y install sqlite3 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip
RUN pip3 install --no-cache-dir --verbose \ 
  pyserial \
  flask \
  flask_cors \
  pandas

FROM stage1 AS stage2

ENV ROS_ROOT=/opt/ros/humble
ENV ROS_DISTRO=humble
ENV ROS_PKG=desktop-full

WORKDIR ${ROS_ROOT}

RUN mkdir src

ARG SKIP_KEYS="fastcdr rti-connext-dds-6.0.1 rti-connext-dds-5.3.1 urdfdom_headers libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv"

RUN rosinstall_generator desktop_full --deps --rosdistro ${ROS_DISTRO} --exclude teleop_twist_joy \
> ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

RUN cat ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall \
    && vcs import src < ros2.${ROS_DISTRO}.${ROS_PKG}.rosinstall

RUN apt-get update \
    # && rosdep init \
    # && rosdep update \
    && rosdep install -y \
      --ignore-src \
      --from-paths src \
      --rosdistro ${ROS_DISTRO} \
      --skip-keys "$SKIP_KEYS"

RUN colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    && rm -Rf src build log

RUN rm -Rf /var/lib/apt/lists/* && apt-get clean

#RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#    && git clone https://github.com/ros-perception/vision_opencv.git && cd vision_opencv && git checkout humble && cd .. \
#    && source ${ROS_ROOT}/install/setup.bash && cd ${ROS_ROOT} \
#    && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} --skip-keys "$SKIP_KEYS" \
#    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
#    && rm -Rf src build log \
#&& rm -rf /var/lib/apt/lists/* \
#&& apt-get clean


FROM stage1

RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source /felix/install/setup.bash" >> /root/.bashrc

# DOCKER_BUILDKIT=1 docker build -f Dockerfile -t felix:latest .
# https://www.ros.org/reps/rep-2001.html#id25
