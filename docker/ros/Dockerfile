FROM ros:jazzy-ros-base@sha256:066420e07f60aa18262f2479981def87ebcfcec42eefb0c0c57c4a46098348ca AS dependencies
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake python3-colcon-common-extensions python3-pytest \
    python3-pytest-cov python3-yaml python3-pyaudio python3-numpy python3-gpiozero \
    ros-jazzy-ament-cmake-pytest ros-jazzy-xacro liburdfdom-tools \
    ros-jazzy-tf-transformations ros-jazzy-robot-state-publisher \
    ros-jazzy-joint-state-publisher ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox ros-jazzy-foxglove-bridge \
    ros-jazzy-v4l2-camera ros-jazzy-topic-tools ros-jazzy-teleop-twist-joy \
    ros-jazzy-image-transport-plugins \
    && rm -rf /var/lib/apt/lists/*
WORKDIR /workspace
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-venv portaudio19-dev alsa-utils ffmpeg python3-gi \
    gir1.2-gstreamer-1.0 gir1.2-gst-plugins-base-1.0 gstreamer1.0-plugins-good \
    ros-jazzy-rmw-cyclonedds-cpp && rm -rf /var/lib/apt/lists/*
RUN python3 -m venv --system-site-packages /opt/robot-venv
ENV PATH=/opt/robot-venv/bin:$PATH

FROM dependencies AS python-dependencies
COPY docker/ros/requirements.lock /tmp/requirements.lock
RUN python -m pip install --no-cache-dir --ignore-installed --require-hashes \
    --no-build-isolation -r /tmp/requirements.lock && rm /tmp/requirements.lock

FROM python-dependencies AS build
SHELL ["/bin/bash", "-c"]
COPY vendor.repos /opt/vendor/vendor.repos
RUN mkdir -p /opt/vendor/src && vcs import /opt/vendor/src < /opt/vendor/vendor.repos \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/vendor && python -m colcon build --merge-install \
       --cmake-args -DBUILD_TESTING=OFF
COPY src /opt/robot/src
RUN source /opt/ros/jazzy/setup.bash && source /opt/vendor/install/setup.bash \
    && cd /opt/robot && python -m colcon build --merge-install
COPY docker/ros/entrypoint.sh docker/ros/robot-start.py /opt/robot/bin/
RUN chmod +x /opt/robot/bin/entrypoint.sh
ENTRYPOINT ["/opt/robot/bin/entrypoint.sh"]

FROM build AS test
COPY docker/ros/test-requirements.lock /tmp/test-requirements.lock
RUN python -m pip install --no-cache-dir --require-hashes -r /tmp/test-requirements.lock
WORKDIR /workspace
COPY tools tools
COPY test test
COPY Makefile pyproject.toml .dockerignore docker-compose.yml compose.hardware.yaml compose.nova.yaml vendor.repos ./
COPY docker docker
RUN ln -s /opt/robot/src /workspace/src
CMD ["bash", "/workspace/tools/check.sh"]

FROM python-dependencies AS runtime
COPY --from=build /opt/vendor/install /opt/vendor/install
COPY --from=build /opt/robot/install /opt/robot/install
COPY --from=build /opt/robot/bin /opt/robot/bin
ENV HOME=/state/home ROS_HOME=/state/ros ROS_LOG_DIR=/state/logs
WORKDIR /opt/robot
USER 1000:1000
ENTRYPOINT ["/opt/robot/bin/entrypoint.sh"]
CMD ["python3", "/opt/robot/bin/robot-start.py"]

FROM runtime AS rviz
USER root
RUN apt-get update && apt-get install -y --no-install-recommends ros-jazzy-rviz2 \
    && rm -rf /var/lib/apt/lists/*
USER 1000:1000
CMD ["rviz2"]
