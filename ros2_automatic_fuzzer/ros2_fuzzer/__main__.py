import argparse
import logging
import os
import json
import logging
import yamale


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


def ensure_yaml_exists(yaml_file_path: str) -> bool:
    if not os.path.exists(yaml_file_path):
        logging.error(
            "No fuzz.yaml file was found\n" "Have you run the auto_detecter command?"
        )
        exit(-1)
    logging.debug(f"YAML file found at {yaml_file_path}")


def read_schema_file():
    yaml_path = os.path.join(os.path.dirname(__file__), "schema.yaml")
    schema = yamale.make_schema(yaml_path)
    logging.debug("YAML schema read")
    return schema


def read_yaml_file(path: str):
    res = yamale.make_data(path)
    logging.debug(f"YAML file loaded")
    return res


def validate_yaml(yaml_obj, schema):
    yamale.validate(schema, yaml_obj)
    logging.debug("YAML file validated")


def verify_yaml_file(yaml_file_path: str):
    schema = read_schema_file()
    yaml_objs = read_yaml_file(yaml_file_path)
    validate_yaml(yaml_objs, schema)
    # yaml_obj is a list of (object, path)
    # so we return the first item's object
    return yaml_objs[0][0]


def read_and_validate_yaml_file(path: str) -> dict:
    yaml_file_path = os.path.join(path, "fuzz.yaml")
    ensure_yaml_exists(yaml_file_path)
    yaml_obj = verify_yaml_file(yaml_file_path)

    if "TODO" in json.dumps(yaml_obj):
        logging.warning(
            "The 'TODO' keyword was found in the yaml file\n"
            "Did you forget to fill in the blanks?"
        )

    services: dict = yaml_obj["services"]
    topics: dict = yaml_obj["topics"]
    logging.info(
        f"{len(services.keys())} services detected: {', '.join(services.keys())}"
    )
    logging.info(f"{len(topics.keys())} topics detected: {', '.join(topics.keys())}")

    return yaml_obj


def main():
    path = usage()
    yaml_obj = read_and_validate_yaml_file(path)
