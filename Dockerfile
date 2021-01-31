ARG ROSDIST=foxy
FROM ros:$ROSDIST
ENV ROSDIST foxy

ARG DEBIAN_FRONTEND=noninteractive

RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash"
RUN apt-get update && \
    apt-get install -y --no-install-recommends apt-utils && \
    apt-get install -y --no-install-recommends ros-foxy-example-interfaces

ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src
COPY . src/
RUN rm src/start.sh src/Dockerfile

# RUN /bin/bash -c "apt-get update && rosdep install -i --from-path src --rosdistro ${ROSDIST} -y"