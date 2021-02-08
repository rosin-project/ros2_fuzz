# Automatic fuzzing
![Python tests](https://github.com/JnxF/automatic_fuzzing/workflows/Python%20tests/badge.svg)
![Hits](https://visitor-badge.glitch.me/badge?page_id=jnxf._automatic_fuzzing)
[![GitHub stars](https://img.shields.io/github/stars/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/stargazers/)
[![GitHub forks](https://img.shields.io/github/forks/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/network/)
[![GitHub repo size in bytes](https://img.shields.io/github/repo-size/JnxF/automatic_fuzzing.svg)](https://github.com/JnxF/automatic_fuzzing)
[![GitHub contributors](https://img.shields.io/github/contributors/JnxF/automatic_fuzzing.svg)](https://GitHub.com/JnxF/automatic_fuzzing/graphs/contributors/)
[![GitHub license](http://img.shields.io/github/license/JnxF/automatic_fuzzing.svg)](https://github.com/JnxF/automatic_fuzzing/blob/master/LICENSE)

Based on the [Creating custom ROS 2 msg and srv files](https://index.ros.org/doc/ros2/Tutorials/Custom-ROS2-Interfaces/) tutorial, we aim to automatize fuzzing on ROS 2.

## Installation

Have Docker installed.

## Dependencies
[![](dependencies.png)](https://dreampuf.github.io/GraphvizOnline/#digraph%20G%20%7B%0A%20%20node%20%5Bshape%3Dbox%5D%3B%0A%20%20TypeParser%20-%3E%20FuzzTargetProcesser%20%5Blabel%3D%22ROSType%22%5D%3B%0A%20%20FuzzTargetProcesser%20-%3E%20TemplateGenerator%20%5Blabel%3D%22FuzzTarget%22%5D%3B%0A%20%20TemplateGenerator%20-%3E%20%22%20%22%20%5Blabel%3D%22cpp%20file%22%5D%3B%0A%20%20%22%20%22%20%5Bshape%3Dnone%5D%3B%0A%7D)

## Usage

```
source start.sh
colcon build
```

## License
[MIT](https://choosealicense.com/licenses/mit/)