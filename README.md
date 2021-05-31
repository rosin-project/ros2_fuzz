# Automatic fuzzing for ROS 2

![Python tests](https://github.com/rosin-project/ros2_fuzz/workflows/Python%20tests/badge.svg)
![Hits](https://visitor-badge.glitch.me/badge?page_id=jnxf._automatic_fuzzing)
[![GitHub stars](https://img.shields.io/github/stars/rosin-project/ros2_fuzz.svg)](https://GitHub.com/rosin-project/ros2_fuzz/stargazers/)
[![GitHub forks](https://img.shields.io/github/forks/rosin-project/ros2_fuzz.svg)](https://GitHub.com/rosin-project/ros2_fuzz/network/)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/rosin-project/ros2_fuzz.svg)](https://github.com/rosin-project/ros2_fuzz)
[![GitHub contributors](https://img.shields.io/github/contributors/rosin-project/ros2_fuzz.svg)](https://GitHub.com/rosin-project/ros2_fuzz/graphs/contributors/)
[![GitHub license](http://img.shields.io/github/license/rosin-project/ros2_fuzz.svg)](https://github.com/rosin-project/ros2_fuzz/blob/master/LICENSE)

An automatic fuzzing tool for ROS 2 C++ projects. The tool comprises two different commands: `auto_detector` and `ros2_fuzzer`.

## Installation

Install the `ros_automatic_fuzzer` folder with pip:

```bash
pip3 install -e ros2_automatic_fuzzer
```

Alternatively, start the `start.sh` command, which spawn a Docker container with both the examples and the tool.

## Usage

1. Navigate to your ROS working space.
2. Run `auto_detector`. This generates a `fuzz.yaml` file with `TODO` gaps.
3. Fill the missing `TODO`s in the `fuzz.yaml` file and complete it.
4. Run `ros2_fuzzer` and follow the instructions.
5. Add the generated fuzzers to their CMakeLists.txt.
6. Make a clean build with the `CC` and `CXX` environment variables pointing to AFL.
7. Run the AFL fuzzers.

Check the following sections for detailed instructions for each step.

### Step 2. The `auto_detector` command

The `auto_detector` command generates a YAML file called `fuzz.yaml` which contains descriptions for three types of artifacts: topics, services, and action servers. The detection process relies on regular expressions in C++, and therefore it is not bullet-proof.

Optional arguments:

- `--path PATH`. The path where to search for ROS artifacts. By default it is the working directory.

Optional flags:

- `-f` or `--overwrite` to force overwriting the file.
- `-v` or `--verbose` to increase the output verbosity.

Typical bash invocation:

```bash
auto_detector
```

### Step 3. The YAML format

The `fuzz.yaml` file contains descriptions for topics, services, and action servers. Some of the fields can be automatically extracted in the previous step thanks to the `auto_detector` command, but others must be manually introduced. There may be `TODO` blanks that must be filled.

The format is simple: there are three optional categories: `topics`, `services` and `actions`. Each of those is a dictionary, with the keys being the name of the artifact. Each artifact contains the following descriptors:

- `headers_file` (compulsory). A string pointing to the `hpp` file where the type of the artifact is defined (the topic type, the service type, or the action type). Sometimes it can be directly inferred.
- `source` (compulsory). A relative path to the `fuzz.yaml` file where the artifact to be fuzzed is located. It must be a C++ source code file.
- `type` (compulsory). The type of the fuzzed artifact. It must conform to the `ros2 interface show` syntax. That is, with `::` as a separator and providing the full type. Correct examples are `example_interfaces::srv::AddTwoInts` and `std_msgs::msg::String`, but not `example_interfaces/srv/AddTwoInts` nor `String`.
- `parameters` (optional). A list with all the parameters of the artifact. It can be inferred with the `auto_detector` command.

Follows a concrete example. You can also check the syntax that is followed in [the YAML schema](ros2_automatic_fuzzer/yaml_utils/schema.yaml), which all `fuzz.yaml` files must conform to.

```yaml
topics:
  minimal_topic:
    headers_file: std_msgs/msg/string.hpp
    source: src/publisher_subscriber_example/src/publisher_member_function.cpp
    type: std_msgs::msg::String
    parameters: []
  topic:
    headers_file: std_msgs/msg/string.hpp
    source: src/parameters_example_package/src/fuzz_target.cpp
    type: std_msgs::msg::String
    parameters: []
services:
  add_two_ints:
    headers_file: example_interfaces/srv/add_two_ints.hpp
    node_name: minimal_subscriber
    source: src/parameters_example_package/src/add_two_ints_server.cpp
    type: example_interfaces::srv::AddTwoInts
    parameters: []
  add_three_ints:
    headers_file: tutorial_interfaces/srv/add_three_ints.hpp
    source: src/client_service_example/src/add_three_ints_server.cpp
    type: tutorial_interfaces::srv::AddThreeInts
    parameters: []
```

### Step 4. The `ros2_fuzzer` command

It consumes the `fuzz.yaml` file to generate C++ fuzzers for the selected artifacts. It allows generating fuzzers for all or some of them. Simply follow the steps on the screen. It may require calling `ros2 interface show`, and thus sourcing the ROS setup bash (with `. install/setup.bash`) may be required.

Optional arguments:

- `--path PATH`. The path where to search for a `fuzz.yaml` file. By default it is on the working directory.

Optional flags:

- `-v` or `--verbose` to increase the output verbosity.

Typical bash invocation:

```bash
ros2_fuzzer
```

### Step 5. Adding the fuzzers to the `CMakeLists.txt` files

The `ros2_fuzzer` command generates files of the `*_generated.cpp` form, which have to be linked to their `CMakeList.txt` files to be compiled.

For example, for the following `fuzz.yaml` file:

```yaml
services:
  add_three_ints:
    headers_file: tutorial_interfaces/srv/add_three_ints.hpp
    source: src/client_service_example/src/add_three_ints_server.cpp
    type: tutorial_interfaces::srv::AddThreeInts
    parameters: []
```

The following code can be added into its `CMakeLists.txt` file (the `generated_fuzzer` keyword can be changed):

```cmake
add_executable(generated_fuzzer src/add_three_ints_server_generated.cpp)
ament_target_dependencies(generated_fuzzer rclcpp tutorial_interfaces)
install(TARGETS generated_fuzzer DESTINATION lib/${PROJECT_NAME})
```

### Step 6. Rebuilding with AFL

To fuzz your C++ artifacts, it is necessary to recompile the projects so that they include instrumentalization annotations on the byte code to be used in the fuzzing search. We have decided to use [AFL](https://github.com/google/AFL), an state-of-the-art fuzzer backed by Google.

If you haven't done so, you can install AFL with

```bash
apt install afl
```

To use it, set the `CC` and `CXX` environment variables to point to AFL and build the projects. We add the `--cmake-clean-cache` flag to prevent stale build files. Of course, you can use a more sofisticated way to build your projects, but make sure that the AFL's instrumentalization takes place.

```bash
export CC=afl-gcc
export CXX=afl-g++
colcon build --cmake-clean-cache
```

### Step 7. Running the fuzzers

Navigate to `install/<package>/lib/<package>/`, where `<package>` is the name of your ROS package (or wherever the installation files are placed) and start the fuzzing search.

The command requires an `inputs/` folder, with some files with content. You can use random values extracted from `/dev/urandom`, for instance:

```bash
mkdir inputs
head -c 50 /dev/urandom > inputs/input0.txt
```

Now execute the `afl-fuzz` command (from AFL). Use the name of the executable found in that folder (try an `ls`) as the last parameter. The example shown in step #5 would be fuzzed with the following command:

```bash
afl-fuzz -i inputs/ -o outputs/ -m none -- ./generated_fuzzer
```

You should just change the `generated_fuzzer` execution file to yours to make AFL work.

## License

[MIT](https://choosealicense.com/licenses/mit/)

## Acknowledgements

<a href="http://rosin-project.eu">
  <img src="http://rosin-project.eu/wp-content/uploads/rosin_ack_logo_wide.png" alt="rosin_logo" height="60">
</a>

Supported by ROSIN - ROS-Industrial Quality-Assured Robot Software
Components. More information:
<a href="http://rosin-project.eu">rosin-project.eu</a>

<img src="http://rosin-project.eu/wp-content/uploads/rosin_eu_flag.jpg" alt="eu_flag" height="45" align="left" >

This project has received funding from the European Union’s Horizon
2020 research and innovation programme under grant agreement no. 732287.
