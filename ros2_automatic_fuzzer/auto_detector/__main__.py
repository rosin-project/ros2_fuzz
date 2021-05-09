import os
import logging
import argparse

from .find_yaml_components import find_yaml_components
from .detect_parameters import ask_detect_parameters


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
    find_yaml_components(rootDir=rootDir, overwrite=overwrite)
    ask_detect_parameters(rootDir=rootDir)
