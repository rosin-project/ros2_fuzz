# Automatic fuzzing for ROS 2

![Python tests](https://github.com/JnxF/automatic_fuzzing/workflows/Python%20tests/badge.svg)
![Hits](https://visitor-badge.glitch.me/badge?page_id=jnxf._automatic_fuzzing)
[![GitHub stars](https://img.shields.io/github/stars/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/stargazers/)
[![GitHub forks](https://img.shields.io/github/forks/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/network/)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/JnxF/automatic_fuzzing.svg)](https://github.com/JnxF/automatic_fuzzing)
[![GitHub contributors](https://img.shields.io/github/contributors/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/graphs/contributors/)
[![GitHub license](http://img.shields.io/github/license/JnxF/automatic_fuzzing.svg)](https://github.com/JnxF/automatic_fuzzing/blob/master/LICENSE)

An automatic fuzzing tool for ROS 2 C++ projects.

## Installation

TODO

```bash
pip install ros2_fuzzer
```

## Usage

The tool comprises two different commands: `auto_detector` and `ros2_fuzzer`:

1. Navigate to your ROS working space.
2. Run `auto_detector`. This generates a `fuzz.yaml` file with `TODO` gaps.
3. Fill the missing `TODO`s in the `fuzz.yaml` file accordingly.
4. Run `ros2_fuzzer` and follow the steps.

## YAML format

The `fuzz.yaml` file contains descriptions for topics, services and action servers.

Some of the fields can be automatically extracted from the code, but others must be manually introduced. The blanks are marked with the `TODO` keyword.

Follows a concrete example. The used syntax follows [the YAML schema](ros2_automatic_fuzzer/yaml_utils/schema.yaml), which the `fuzz.yaml` file conforms to.

TODO

```yaml
topics:
  topic:
    headers_file: TODO
    node_name: TODO
    source: src/parameters_example_package/src/fuzz_target.cpp
    type: std_msgs::msg::String
    parameters: []
services:
  add_two_ints:
    headers_file: TODO
    node_name: TODO
    source: src/parameters_example_package/src/add_two_ints_server.cpp
    type: example_interfaces::srv::AddTwoInts
    parameters: []
  add_three_ints:
    headers_file: tutorial_interfaces/srv/add_three_ints.hpp
    node_name: TODO
    source: src/automatic_fuzzing/src/add_three_ints_server.cpp
    type: tutorial_interfaces::srv::AddThreeInts
    parameters: []
```

## CLIs

### `auto_detector`

TODO

```bash
usage: auto_detector [-h] [--path PATH] [-f] [-v]

Automatic C++ ROS 2 components finder

optional arguments:
  -h, --help       show this help message and exit
  --path PATH      path to search for ROS artifacts (default = the working
                   directory)
  -f, --overwrite  forces overwrite
  -v, --verbose    increase output verbosity
```

### `ros2_fuzzer`

TODO

```bash
usage: ros2_fuzzer [-h] [--path PATH] [-v]

ROS 2 automatic fuzzer

optional arguments:
  -h, --help     show this help message and exit
  --path PATH    Path where the fuzz.yaml file is located
  -v, --verbose  increase output verbosity
```

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
