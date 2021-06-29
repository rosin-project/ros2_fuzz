import sys
from zenlog import log as logging
import os
from PyInquirer import prompt
import yaml

sys.path.append("..")

from yaml_utils.yaml_utils import read_and_validate_yaml_file


def yes_no_question(message: str, default=True):
    return prompt(
        {
            "type": "confirm",
            "message": message,
            "name": "question",
            "default": False,
        }
    )["question"]


def detect_parameters(rootDir: str):
    yaml_obj: dict = read_and_validate_yaml_file(rootDir)

    services: dict = yaml_obj["services"]
    topics: dict = yaml_obj["topics"]

    logging.info("Detecting services' parameters")
    for service_name, service_value in services.items():
        if "parameters" in service_value and service_value["parameters"] != []:
            question_prompt = f"*Overwrite* parameters for service `{service_name}`?"
        else:
            question_prompt = f"Detect parameters for service `{service_name}`?"

        if yes_no_question(question_prompt, False):
            # TODO
            pass

    for topic_name, topic_value in topics.items():
        if yes_no_question(f"Detect parameters for topic `{topic_name}`?", False):
            # TODO
            pass

    yaml_path = os.path.join(rootDir, "fuzz.yaml")
    with open(yaml_path, "w") as outfile:
        yaml.dump(yaml_obj, outfile, sort_keys=False)
        logging.info("The file `fuzz.yaml` has been overwritten")


def ask_detect_parameters(rootDir: str):
    if yes_no_question("Do you want to autodetect parameters?", False):
        detect_parameters(rootDir)
