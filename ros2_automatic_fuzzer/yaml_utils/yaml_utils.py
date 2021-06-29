from zenlog import log as logging
import os
import json
from zenlog import log as logging
import yamale


def ensure_yaml_exists(yaml_file_path: str) -> bool:
    if not os.path.exists(yaml_file_path):
        logging.error(
            "No fuzz.yaml file was found\n" "Have you run the auto_detector command?"
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

    services_keys = (yaml_obj["services"] if "services" in yaml_obj else {}).keys()
    topics_keys = (yaml_obj["topics"] if "topics" in yaml_obj else {}).keys()
    actions_keys = (yaml_obj["actions"] if "actions" in yaml_obj else {}).keys()

    logging.debug(
        f"{len(topics_keys)} topics detected: {', '.join([f'`{s}`' for s in topics_keys])}"
    )
    logging.debug(
        f"{len(services_keys)} services detected: {', '.join([f'`{s}`' for s in services_keys])}"
    )
    logging.debug(
        f"{len(actions_keys)} actions detected: {', '.join([f'`{s}`' for s in actions_keys])}"
    )

    return yaml_obj
