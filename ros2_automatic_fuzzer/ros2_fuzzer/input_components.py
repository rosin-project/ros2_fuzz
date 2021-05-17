import os
from zenlog import log as logging
from PyInquirer import Separator, prompt


def component_options(components: dict):
    def validate_component(component):
        source = component["source"]
        original_file = os.path.basename(source)
        ros_type_str = component["type"]
        headers_file = component["headers_file"]

        if "TODO" in [source, ros_type_str, headers_file, original_file]:
            return "There are unfilled TODOs!"
        if not os.path.isfile(source):
            return f"The source `{source}` is not a valid path"
        return ""

    mapped_components = [
        (component, component_name, validate_component(component))
        for (component_name, component) in components.items()
    ]
    return [
        {"name": component_name, "disabled": error}
        if error
        else {"name": component_name, "value": [component_name, component]}
        for (component, component_name, error) in mapped_components
    ]


def ask_for_components(services: dict, topics: dict, actions: dict):
    services_choices = component_options(services)
    topics_choices = component_options(topics)
    actions_choices = component_options(actions)

    all_choices = services_choices + topics_choices + actions_choices

    there_are_valid_choices = any("disabled" not in choice for choice in all_choices)
    if not there_are_valid_choices:
        logging.error(
            "There are no available components to fuzz!\n"
            "Check for TODOs in the fuzz.yaml file."
        )
        exit(-1)

    choices = [
        Separator("Topics"),
        *topics_choices,
        Separator("Services"),
        *services_choices,
        Separator("Actions"),
        *actions_choices,
    ]

    questions = [
        {
            "type": "checkbox",
            "message": "What do you want to fuzz?",
            "name": "to_fuzz_components",
            "choices": choices,
        }
    ]

    return prompt(questions)["to_fuzz_components"]
