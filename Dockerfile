ARG ROSDIST=foxy
FROM ros:$ROSDIST
ENV ROSDIST foxy

ENV DEBIAN_FRONTEND noninteractive

RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash"

# TODO: remove (use rosdep install instead)
RUN apt-get update && \
    apt-get install -y apt-utils 2>&1 | grep -v "debconf: delaying package configuration, since apt-utils is not installed" && \
    apt-get install -y --no-install-recommends ros-foxy-example-interfaces && \
    apt-get install -y vim && \
    apt-get install -y python3-pip

ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src
COPY ./automatic_fuzzing src/automatic_fuzzing/
COPY ./tutorial_interfaces src/tutorial_interfaces/


# RUN /bin/bash -c "apt-get update && rosdep install -i --from-path src --rosdistro ${ROSDIST} -y"

# TODO: remove
RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash && colcon build"

COPY ./type_splitter src/type_splitter/

RUN pip3 install src/type_splitter/
RUN rm -rf src/type_splitter/