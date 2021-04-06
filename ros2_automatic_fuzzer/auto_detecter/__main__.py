import os
import logging
import re
import yaml


def main():
    # TODO: remove
    logging.basicConfig(level=logging.DEBUG)

    logging.info("Starting directory exploration")
    cppfiles = []
    rootDir = os.getcwd()
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fname in fileList:
            if fname.endswith("cpp"):
                cppfiles.append(os.path.join(dirName, fname))

    logging.info(f"Logged {len(cppfiles)} .cpp files")
    for file in cppfiles:
        logging.debug(f"\t {file}")

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

    logging.info("Checking file contents")
    found_publishers = dict()
    found_services = dict()
    for filepath in cppfiles:
        with open(filepath) as f:
            contents = f.read()
            for (regex, container) in [
                (create_publisher_regex, found_publishers),
                (create_service_regex, found_services),
            ]:
                instance = re.search(regex, contents)
                if instance:
                    container[instance.group("name")] = {
                        "type": instance.group("type"),
                        "headers_file": "TODO",
                        "source": filepath,
                        "node_name": "TODO",
                        "parameters": [],
                    }

    logging.info("Writing YAML file")
    yaml_result = {
        "topics": found_publishers,
        "services": found_services
    }
    with open('generated.yaml', 'w') as outfile:
        yaml.dump(yaml_result, outfile)
    
    logging.info("Written YAML file")