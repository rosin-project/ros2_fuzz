from .type_parser import *
from .template_generator import *
from .fuzzing_descriptor import *
import sys
import argparse
import logging

def main():
    # Argument parser
    parser = argparse.ArgumentParser(
        prog="ros2_automatic_fuzzer", description="ROS 2 automatic topic fuzzer"
    )
    parser.add_argument(
        "topic_name", help='topic name (i.e. "tutorial_interfaces/srv/AddThreeInts")'
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )
    args = parser.parse_args()
    topic_name = args.topic_name
    is_verbose = args.verbose

    # Change logging
    if is_verbose:
        logging.basicConfig(level=logging.DEBUG)

    # 1. Parse the topic
    ros_type: ROSType = TypeParser.parse_topic(topic_name)
    if not ros_type:
        sys.exit()

    # 2. Create a fuzz target
    fuzz_target: FuzzTarget = FuzzTargetProcesser().process(ros_type)
    if not fuzz_target:
        sys.exit()

    # 3. Generate the cpp file
    TemplateGenerator.generate_cpp_file(fuzz_target)
