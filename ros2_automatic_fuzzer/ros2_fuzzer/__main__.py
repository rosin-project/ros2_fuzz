import sys
import argparse
import os

from zenlog import log as logging
from logging import INFO, DEBUG

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
    logging.level(DEBUG if args.verbose else INFO)

    return path


def main():
    path = usage()
    yaml_obj: dict = read_and_validate_yaml_file(path)

    services: dict = yaml_obj["services"] if "services" in yaml_obj else {}
    topics: dict = yaml_obj["topics"] if "topics" in yaml_obj else {}
    actions: dict = yaml_obj["actions"] if "actions" in yaml_obj else {}

    for (name, value) in ask_for_components(
        services=services, topics=topics, actions=actions
    ):
        is_service = (name, value) in services.items()
        is_topic = (name, value) in topics.items()
        is_action = (name, value) in actions.items()

        if is_service:
            destination_path = generate_service_template(
                source=value["source"],
                ros_type_str=value["type"],
                headers_file=value["headers_file"],
            )
            logging.info(f"{name}: created fuzzer for the service")
            logging.info(f"└── {destination_path}")

        elif is_topic:
            destination_path = generate_topic_template(
                source=value["source"],
                ros_type_str=value["type"],
                headers_file=value["headers_file"],
            )
            logging.info(f"{name}: created fuzzer for the topic")
            logging.info(f"└── {destination_path}")

        elif is_action:
            # TODO
            pass

    logging.info("Fuzzer(s) generated successfully")
    logging.warning("Please link the fuzzers to their CMakeLists.txt files,")
    logging.warning(
        "recompile the projects with instrumentalization and start the fuzzers."
    )
