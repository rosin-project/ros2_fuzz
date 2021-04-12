import os
import logging
import re
import yaml
import argparse


def usage():
    parser = argparse.ArgumentParser(
        prog="auto_detector", description="Automatic C++ ROS 2 components finder"
    )
    parser.add_argument(
        "--path",
        help="path to search for ROS artifacts (default = the working directory)",
    )
    parser.add_argument(
        "-f", "--overwrite", help="forces overwrite", action="store_true"
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )

    args = parser.parse_args()
    path = args.path if args.path else os.getcwd()
    logging.basicConfig(level=logging.DEBUG if args.verbose else logging.INFO)

    return path, args.overwrite


def main():
    rootDir, overwrite = usage()
    yaml_path = os.path.join(rootDir, "fuzz.yaml")

    if os.path.exists(yaml_path) and not overwrite:
        logging.error("The file fuzz.yaml already exists")
        logging.error("Use the -f flag to force overwriting")
        exit(-1)

    logging.debug("Starting directory exploration")
    cppfiles = []
    for dirName, subdirList, fileList in os.walk(rootDir):
        for fname in fileList:
            if fname.endswith("cpp"):
                cppfiles.append(os.path.join(dirName, fname))

    logging.debug(f"Logged {len(cppfiles)} .cpp files")
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

    logging.debug("Checking file contents")
    found_publishers = dict()
    found_services = dict()
    for filepath in cppfiles:
        try:
            with open(filepath) as f:
                contents = f.read()
                for (regex, container) in [
                    (create_publisher_regex, found_publishers),
                    (create_service_regex, found_services),
                ]:
                    instance = re.search(regex, contents)
                    if instance:
                        container[instance.group("name")] = {
                            "headers_file": "TODO",
                            "node_name": "TODO",
                            "source": os.path.relpath(filepath, start=rootDir),
                            "type": instance.group("type"),
                            "parameters": [],
                        }
        except:
            pass

    # Generate results
    yaml_result = {"topics": found_publishers, "services": found_services}

    if len(found_publishers) + len(found_services) == 0:
        logging.error(
            "No publisher nor service has been found\n"
            "Are you in (or have you provided) the correct path?"
        )
        exit(-1)

    if os.path.exists(yaml_path) and overwrite:
        logging.warning("Overwriting the fuzz.yaml file")

    with open(yaml_path, "w") as outfile:
        yaml.dump(yaml_result, outfile, sort_keys=False)

    logging.info("The file `fuzz.yaml` has been generated")
    logging.info("Please fill the TODOs accordingly before start fuzzing")
