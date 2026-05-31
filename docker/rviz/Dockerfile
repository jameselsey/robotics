FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-jazzy-rmw-cyclonedds-cpp \
        ros-jazzy-slam-toolbox \
        ros-jazzy-nav2-rviz-plugins \
    && rm -rf /var/lib/apt/lists/*

COPY cyclonedds.xml /etc/cyclonedds.xml

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=file:///etc/cyclonedds.xml
ENV QT_X11_NO_MITSHM=1
ENV LIBGL_ALWAYS_SOFTWARE=1

CMD ["bash", "-lc", "source /opt/ros/jazzy/setup.bash && rviz2"]
