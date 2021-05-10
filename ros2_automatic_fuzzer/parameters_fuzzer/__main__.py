import argparse
from zenlog import log as logging
from logging import INFO, DEBUG

import os
import re
import signal
import subprocess
import time


def get_parameters(node_name: str, package_name: str, executable_name: str):
    # Fork the execution
    # - The child starts the node in the background with "ros2 run _____"
    # - The parent waits a bit and starts getting the parameters
    #   When it has finished, it kills the child

    parent_pid = os.getpid()
    pid = os.fork()

    # Parent code
    if pid != 0:
        logging.info("PARENT: Start parent process")

        # Give some time buffer to initialize
        time.sleep(3)

        # ros2 param list
        try:
            ros2_param_result = subprocess.check_output(
                ["ros2", "param", "list", node_name],
                stderr=subprocess.STDOUT,
                encoding="utf8",
            )
        except:
            logging.error(f"Couldn't run `ros2 param list`\n")
            os.kill(pid, signal.SIGKILL)
            os._exit(0)

        # If it is empty, something wrong happened
        if "Node not found" in ros2_param_result:
            logging.error(f"The node `{node_name}` is not found.")

        ros2_param_result = [param.strip() for param in ros2_param_result.splitlines()]

        # Iterate through every parameter
        logging.info("Gathering parameters info")
        result = []
        for param in ros2_param_result:
            logging.info(f"- {param}")
            param_info = subprocess.check_output(
                ["ros2", "param", "describe", node_name, param],
                stderr=subprocess.STDOUT,
                encoding="utf-8",
            )
            print(["ros2", "param", "describe", node_name, param])
            type = re.search(
                "Type:\s+([^\n]+)",
                param_info,
            ).group(1)
            logging.info(f"\t {type}")
            result.append((param, type))

        # Kill the child
        os.kill(pid, signal.SIGKILL)

        # Return
        return result

    # Child code
    else:
        try:
            res = subprocess.check_output(
                ["ros2", "run", package_name, executable_name]
            )
            logging.error(
                f"`ros2 run {package_name} {executable_name}` has stopped.\n"
                "Have you misspelled anything?"
            )
            os.kill(parent_pid, signal.SIGKILL)
            os._exit(0)

        except:
            logging.error(
                f"`ros2 run {package_name} {executable_name}` has ended abrubtly\n"
                "Have you sourced install/setup.bash?"
            )
            os.kill(parent_pid, signal.SIGKILL)
            os._exit(0)


def main():
    # Argument parser
    parser = argparse.ArgumentParser(
        prog="parameters_fuzzer", description="ROS 2 automatic parameters fuzzer"
    )
    parser.add_argument(
        "node_name",
        help='(i.e. "minimal_subscriber")',
    )
    parser.add_argument(
        "package_name",
        help='(i.e. "mypackage")',
    )
    parser.add_argument(
        "executable_name",
        help='(i.e. "listener")',
    )
    parser.add_argument(
        "-v", "--verbose", help="increase output verbosity", action="store_true"
    )

    args = parser.parse_args()
    node_name = args.node_name
    package_name = args.package_name
    executable_name = args.executable_name
    is_verbose = args.verbose

    # Change verbosity
    logging.level(DEBUG if is_verbose else INFO)

    # 1. Get parameters
    parameters = get_parameters(
        node_name=node_name, package_name=package_name, executable_name=executable_name
    )
    print(parameters)

    # 2. Create fuzzers

    # 3. Populate template
