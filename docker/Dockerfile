FROM ros:iron

RUN apt-get update && \
    apt-get install -y \
    ros-iron-rviz2 \
    ros-iron-gazebo-ros-pkgs \
    ros-iron-xacro && \
    rm -rf /var/lib/apt/lists/*

# don't copy things that might unnecessarily invalidate the cache
# this can be made cleaner when COPY --exclude is ready.
# see https://docs.docker.com/reference/dockerfile/#copy---exclude$
# then it can simply be
# COPY --exclude docker --exclude .* --exclude CONTRIBUTING.md . /ros_ws/src
WORKDIR /ros_ws
COPY launch ./src/launch
COPY meshes ./src/meshes
COPY rviz ./src/rviz
COPY urdf ./src/urdf
COPY worlds ./src/worlds
COPY package.xml CMakeLists.txt LICENSE APACHE-2.0-LICENSE README.md ./src/

COPY docker/start.sh /start.sh

RUN bash -c "source /opt/ros/iron/setup.bash && colcon build"
ENTRYPOINT ["/start.sh"]
