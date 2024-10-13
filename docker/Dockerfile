FROM ros:iron

RUN apt-get update && \
    apt-get install -y \
    ros-iron-rviz2 \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-xacro && \
    rm -rf /var/lib/apt/lists/*

COPY . ./ros_ws/src
COPY docker/start.sh /start.sh
WORKDIR /ros_ws

RUN bash -c "source /opt/ros/iron/setup.bash && colcon build"
ENTRYPOINT ["/start.sh"]

