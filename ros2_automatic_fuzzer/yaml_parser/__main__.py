import json
import logging

import yaml
from jsonschema import validate


def read_schema_file():
    with open("schema.json") as fd:
        schema = json.load(fd)
        logging.info("Schema loaded")
        return schema


def read_yaml_file(path: str):
    with open(path) as fd:
        res = yaml.load(fd, Loader=yaml.FullLoader)
        logging.info("YAML file loaded")
        return res


def validate_yaml(yaml_obj, schema):
    validate(yaml_obj, schema=schema)
    logging.info("YAML file validated")


def main():
    logging.info("Validating yaml file")
    schema = read_schema_file()
    yaml_obj = read_yaml_file("example_yaml.yaml")
    validate_yaml(yaml_obj, schema)
