import os
from zenlog import log as logging
import re
import yaml


def find_yaml_components(rootDir: str, overwrite: bool) -> None:
    yaml_path = os.path.join(rootDir, "fuzz.yaml")

    if os.path.exists(yaml_path) and not overwrite:
        logging.warning("The file fuzz.yaml already exists")
        logging.warning("Use the -f flag to force overwriting")
        return

    logging.debug("Starting directory exploration")
    cppfiles = []
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fname in fileList:
            if fname.endswith("cpp"):
                cppfiles.append(os.path.join(dirName, fname))

    logging.debug(f"Logged {len(cppfiles)} .cpp files")

    # Catches expressions of the type create_publisher<A>("B"
    # being A the type being catched, and B the name of the service
    create_publisher_regex = (
        r"create_publisher\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    )

    # Catches expressions of the type create_service<A>("B"
    # being A the type being catched, and B the name of the service
    create_service_regex = (
        r"create_service\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*\"(?P<name>[^\"]+)\""
    )

    # Catches expressions of the type create_server<A>(..., "B")
    # being A the type being catched, and B the name of the server
    create_action_regex = (
        r"create_server\s*<\s*(?P<type>[^>]+)\s*>\s*\(\s*[^,]+,\s*\"(?P<name>[^\"]+)\""
    )

    logging.debug("Checking file contents")
    found_publishers = dict()
    found_services = dict()
    found_actions = dict()

    finding_patterns = [
        (create_publisher_regex, found_publishers),
        (create_service_regex, found_services),
        (create_action_regex, found_actions),
    ]

    for filepath in cppfiles:
        try:
            with open(filepath) as f:
                contents = f.read()
                for (regex, container) in finding_patterns:
                    instance = re.search(regex, contents)
                    if instance:
                        logging.debug(f"Found instance at {filepath}")

                        name = instance.group("name")
                        type = instance.group("type")

                        if "::" not in type:
                            logging.warning(f"The `{type}` type may be incomplete")

                        container[name] = {
                            "headers_file": map_type_to_headers_file(type),
                            "source": os.path.relpath(filepath, start=rootDir),
                            "type": type,
                            "parameters": [],
                        }
        except:
            pass

    # Generate results
    yaml_result = {}
    if found_publishers:
        yaml_result["topics"] = found_publishers

    if found_services:
        yaml_result["services"] = found_services

    if found_actions:
        yaml_result["actions"] = found_actions

    if len(found_publishers) + len(found_services) + len(found_actions) == 0:
        logging.error(
            "No component has been found\n"
            "Are you in (or have you provided) the correct path?"
        )
        exit(-1)

    if os.path.exists(yaml_path) and overwrite:
        logging.warning("Overwriting the fuzz.yaml file")

    with open(yaml_path, "w") as outfile:
        yaml.dump(yaml_result, outfile, sort_keys=False)

    logging.info("The file `fuzz.yaml` has been generated")
    logging.warning("Fill the TODOs accordingly before continuing!")


def map_type_to_headers_file(type: str) -> str:
    mapping = {
        "std_msgs::msg::Char": "char",
        "std_msgs::msg::Float64": "float_64",
        "std_msgs::msg::Int32": "int_32",
        "std_msgs::msg::Int8MultiArray": "int_8_multi_array",
        "std_msgs::msg::UInt16MultiArray": "u_int_16_multi_array",
        "std_msgs::msg::UInt8": "u_int_8",
        "std_msgs::msg::ColorRGBA": "color_rgba",
        "std_msgs::msg::Float64MultiArray": "float_64_multi_array",
        "std_msgs::msg::Int32MultiArray": "int_32_multi_array",
        "std_msgs::msg::MultiArrayDimension": "multi_array_dimension",
        "std_msgs::msg::UInt32": "u_int_32",
        "std_msgs::msg::UInt8MultiArray": "u_int_8_multi_array",
        "std_msgs::msg::Bool": "bool",
        "std_msgs::msg::Empty": "empty",
        "std_msgs::msg::Header": "header",
        "std_msgs::msg::Int64": "int_64",
        "std_msgs::msg::MultiArrayLayout": "multi_array_layout",
        "std_msgs::msg::UInt32MultiArray": "u_int_32_multi_array",
        "std_msgs::msg::Byte": "byte",
        "std_msgs::msg::Float32": "float_32",
        "std_msgs::msg::Int16": "int_16",
        "std_msgs::msg::Int64MultiArray": "int_64_multi_array",
        "std_msgs::msg::String": "string",
        "std_msgs::msg::UInt64": "u_int_64",
        "std_msgs::msg::ByteMultiArray": "byte_multi_array",
        "std_msgs::msg::Float32MultiArray": "float_32_multi_array",
        "std_msgs::msg::Int16MultiArray": "int_16_multi_array",
        "std_msgs::msg::Int8": "int_8",
        "std_msgs::msg::UInt16": "u_int_16",
        "std_msgs::msg::UInt64MultiArray": "u_int_64_multi_array",
    }

    if type in mapping:
        return "std_msgs/msg/" + mapping[type] + ".hpp"
    else:
        return "TODO"
