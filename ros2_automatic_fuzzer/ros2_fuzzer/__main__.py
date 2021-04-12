import argparse
import logging
import os
import logging
from .service_fuzzer.__main__ import generate_template
from .yaml_verification import read_and_validate_yaml_file


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
    yaml_obj = read_and_validate_yaml_file(path)

    services: dict = yaml_obj["services"]
    topics: dict = yaml_obj["topics"]

    # Create fuzzers for services
    for service_name, service in services.items():
        logging.info(f"Creating fuzzer for {service_name}")
        generate_template(
            source=service["source"],
            ros_type_str=service["type"],
            headers_file=service["headers_file"]
        )
