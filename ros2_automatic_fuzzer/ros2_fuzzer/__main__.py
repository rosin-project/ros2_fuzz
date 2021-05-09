import sys
import argparse
import logging
import os
import logging

sys.path.append("..")

from yaml_utils.yaml_utils import read_and_validate_yaml_file
from .input_components import ask_for_components
from .service_fuzzer import generate_service_template
from .topic_fuzzer import generate_topic_template


def usage() -> str:
    parser = argparse.ArgumentParser(
        prog="ros2_fuzzer", description="ROS 2 automatic fuzzer"
    )
    parser.add_argument(
        "--path",
        help="Path where the fuzz.yaml file is located",
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )

    args = parser.parse_args()
    path = args.path if args.path else os.getcwd()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    return path


def main():
    path = usage()
    yaml_obj: dict = read_and_validate_yaml_file(path)

    services: dict = yaml_obj["services"]
    topics: dict = yaml_obj["topics"]

    for (name, value) in ask_for_components(services=services, topics=topics):
        is_service = (name, value) in services.items()
        is_topic = (name, value) in topics.items()

        if is_service:
            logging.info(f"Creating fuzzer for service `{name}`")
            generate_service_template(
                source=value["source"],
                ros_type_str=value["type"],
                headers_file=value["headers_file"],
            )

        if is_topic:
            logging.info(f"Creating fuzzer for topic `{name}`")
            generate_topic_template(
                source=value["source"],
                ros_type_str=value["type"],
                headers_file=value["headers_file"],
            )
