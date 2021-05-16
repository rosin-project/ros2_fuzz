import os
from zenlog import log as logging
from pathlib import Path
from jinja2 import FileSystemLoader, Environment
from .fuzzing_descriptor import FuzzTarget


def generate_cpp_file(fuzz_target: FuzzTarget, source_file: str, template_name: str):
    __location__ = os.path.realpath(
        os.path.join(os.getcwd(), os.path.dirname(__file__))
    )
    plain_source_file_name = Path(source_file).name
    without_extension = os.path.splitext(plain_source_file_name)[0]

    # Read template
    env = Environment(loader=FileSystemLoader(__location__))
    template = env.get_template(template_name)
    logging.debug("Template read")

    # Populate template
    template_arguments = fuzz_target.get_mapping()
    template_arguments["FILE_NAME"] = plain_source_file_name
    fuzzing_path = os.path.join(os.path.dirname(__file__), "fuzzing_api.hpp")
    template_arguments["FUZZING_API"] = open(fuzzing_path).read()
    template = template.render(template_arguments)
    logging.debug("Template populated")

    # Write the populated file
    full_path = os.path.join(
        os.path.dirname(source_file), without_extension + "_generated.cpp"
    )
    try:
        with open(full_path, "w") as fd:
            fd.write(template)
            logging.debug(f"Template written with {fuzz_target.client_name} client")
    except Exception:
        logging.error("Couldn't write generated file", exc_info=True)

    return full_path
