# Type splitter

A hand-made grammar parser for the [simplified and official version](https://index.ros.org/doc/ros2/Concepts/About-ROS-Interfaces/) of the Interface Definition Language (IDL) for ROS 2.

## Installation

```bash
pip install .
```

## How to use

```
usage: ros2_automatic_fuzzer [-h] [-v] [-p PATH] topic_name

ROS 2 automatic topic fuzzer

positional arguments:
  topic_name            full topic name (i.e. "tutorial_interfaces/srv/AddThreeInts")

optional arguments:
  -h, --help            show this help message and exit
  -v, --verbose         increase output verbosity
  -p PATH, --path PATH  source code path
```

## How to fuzz a topic?
1. Install ROS with `. install/setup.bash`.
2. 


## Tests

```bash
python3 -m unittest discover .
```