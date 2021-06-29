import os

from zenlog import log as logging
from logging import INFO, DEBUG


import argparse

from .find_yaml_components import find_yaml_components
from .detect_parameters import ask_detect_parameters


def usage():
    parser = argparse.ArgumentParser(
        prog="auto_detector", description="Automatic C++ ROS 2 components finder"
    )
    parser.add_argument(
        "--path",
        help="path to search for ROS artifacts and where to generate the fuzz.yaml file. "
        "By default it is the working directory",
    )
    parser.add_argument(
        "-f", "--overwrite", help="forces overwrite", action="store_true"
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )

    args = parser.parse_args()
    path = args.path if args.path else os.getcwd()
    logging.level(DEBUG if args.verbose else INFO)

    return path, args.overwrite


def main():
    rootDir, overwrite = usage()
    find_yaml_components(rootDir=rootDir, overwrite=overwrite)
    ask_detect_parameters(rootDir=rootDir)
