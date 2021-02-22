import os
import logging
from jinja2 import FileSystemLoader, Environment
from .fuzzing_descriptor import FuzzTarget


class TemplateGenerator:
    @staticmethod
    def generate_cpp_file(fuzz_target: FuzzTarget, destination_path: str):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__))
        )
        
        # Read template
        env = Environment(loader=FileSystemLoader(__location__))
        template = env.get_template("template.jinx.cpp")
        logging.info("Template read")

        # Populate template
        template = template.render(fuzz_target.get_mapping())
        logging.info("Template populated")

        # Write the populated file
        try:
            with open(os.path.join(destination_path, "generated.cpp"), "w") as fd:
                fd.write(template)
                logging.info(f"Template written with {fuzz_target.client_name} client")
        except Exception:
            logging.error("Couldn't write generated file", exc_info=True)