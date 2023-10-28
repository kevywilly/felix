FROM humble:r32.7.4

WORKDIR /

ENV ROS_ROOT=/opt/ros/humble
ENV ROS_DISTRO=humble

COPY dependencies/rosmaster /rosmaster
RUN cd /rosmaster && python3 setup.py install
RUN rm -rf /rosmaster


RUN python3 -m pip install --upgrade pip
RUN pip3 install --no-cache-dir --verbose \ 
  pyserial \
  flask \
  flask_cors \
  pydantic

RUN echo "source /ros_entrypoint.sh" >> /root/.bashrc
RUN echo "source /felix/install/setup.bash" >> /root/.bashrc


# ARG SKIP_KEYS="fastcdr rti-connext-dds-6.0.1 rti-connext-dds-5.3.1 urdfdom_headers libopencv-dev libopencv-contrib-dev libopencv-imgproc-dev python-opencv python3-opencv navigation2"

#RUN apt-get update && mkdir -p ${ROS_ROOT}/src && cd ${ROS_ROOT}/src \
#    && git clone https://github.com/ros-perception/vision_opencv.git && cd vision_opencv && git checkout humble && cd .. \
#    && source ${ROS_ROOT}/install/setup.bash && cd ${ROS_ROOT} \
#    && rosdep install -y -r --ignore-src --from-paths src --rosdistro ${ROS_DISTRO} --skip-keys "$SKIP_KEYS" \
#    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
#    && rm -Rf src build log \
#&& rm -rf /var/lib/apt/lists/* \
#&& apt-get clean

