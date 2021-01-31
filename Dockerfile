ARG ROSDIST=foxy
FROM ros:$ROSDIST
ENV ROSDIST foxy

ENV DEBIAN_FRONTEND noninteractive

RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash"

# GitHub CI/CD complains if apt-utils is not installed:
# https://github.com/phusion/baseimage-docker/issues/319#issuecomment-573368959
RUN apt-get update && \
    apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" && \
    apt-get install -y --no-install-recommends ros-foxy-example-interfaces

ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src
COPY . src/
RUN rm src/start.sh src/Dockerfile

# RUN /bin/bash -c "apt-get update && rosdep install -i --from-path src --rosdistro ${ROSDIST} -y"