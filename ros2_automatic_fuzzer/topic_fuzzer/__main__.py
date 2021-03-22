import argparse
import logging
import os
import sys

from .type_parser import *


def main():
    # Argument parser
    parser = argparse.ArgumentParser(
        prog="topic_fuzzer", description="ROS 2 automatic topic fuzzer"
    )
    parser.add_argument(
        "topic_name",
        help='used as "/<topic_name>" (i.e. "topic")',
    )
    parser.add_argument(
        "topic_type",
        help='full topic type (i.e. "tutorial_interfaces/msg/Num")',
    )
    parser.add_argument(
        "topic_type_path",
        help='topic type path to be included (i.e. "tutorial_interfaces/msg/num.hpp")',
    )
    parser.add_argument(
        "publisher_path",
        help='source file where the publisher is defined (i.e. "publisher.cpp")',
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )

    args = parser.parse_args()
    topic_name = args.topic_name
    topic_type = args.topic_type
    topic_type_path = args.topic_type_path
    publisher_path = args.publisher_path
    is_verbose = args.verbose

    # Change logging
    if is_verbose:
        logging.basicConfig(level=logging.DEBUG)

    # 1. Parse the topic
    ros_type: ROSType = TypeParser.parse_topic(topic_type)
    if not ros_type:
        sys.exit()

    # 2.
