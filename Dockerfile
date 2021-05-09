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

# AFL is broken on Ubuntu 20.04...
# Meanwhile, let's install it manually
# TODO: apt-get install afl -y
# when it gets fixed
RUN git clone "https://github.com/google/AFL" && make -C AFL/ install && rm -rf AFL/

ENV ROS_WS /opt/ros_ws
WORKDIR $ROS_WS
RUN mkdir -pv $ROS_WS/src

# Copy packages
COPY ./example_packages/client_service_example src/client_service_example/
COPY ./example_packages/tutorial_interfaces src/tutorial_interfaces/
COPY ./example_packages/parameters_example_package src/parameters_example_package/
COPY ./example_packages/cpp_parameters src/cpp_parameters/
COPY ./example_packages/publisher_subscriber_example src/publisher_subscriber_example/

COPY ./example_packages/example_fuzz.yaml ./fuzz.yaml
# COPY ./example_packages/geometry2 src/geometry2/

# RUN /bin/bash -c "apt-get update && rosdep install -i --from-path src --rosdistro ${ROSDIST} -y"

# TODO: remove
RUN /bin/bash -c "source /opt/ros/${ROSDIST}/setup.bash && colcon build"

# Copy the python package, install it and erase the source files
COPY ./ros2_automatic_fuzzer src/ros2_automatic_fuzzer/
RUN pip3 install -e src/ros2_automatic_fuzzer/
# RUN rm -rf src/ros2_automatic_fuzzer/
