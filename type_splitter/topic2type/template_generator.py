import os
import logging
from pathlib import Path
from jinja2 import FileSystemLoader, Environment
from .fuzzing_descriptor import FuzzTarget


class TemplateGenerator:
    @staticmethod
    def generate_cpp_file(
        fuzz_target: FuzzTarget, source_file: str
    ):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__))
        )
        plain_source_file_name = Path(source_file).name
        without_extension = os.path.splitext(plain_source_file_name)[0]

        # Read template
        env = Environment(loader=FileSystemLoader(__location__))
        template = env.get_template("template.jinx.cpp")
        logging.info("Template read")

        # Populate template
        template_arguments = fuzz_target.get_mapping()
        template_arguments["FILE_NAME"] = plain_source_file_name
        template = template.render(template_arguments)
        logging.info("Template populated")

        # Write the populated file
        try:
            with open(os.path.join(os.getcwd(), without_extension + "_generated.cpp"), "w") as fd:
                fd.write(template)
                logging.info(f"Template written with {fuzz_target.client_name} client")
        except Exception:
            logging.error("Couldn't write generated file", exc_info=True)