import os
import logging
from .type_parser import Field, ROSType
from .fuzzing_descriptor import FuzzTargetProcesser, FuzzTarget

class TemplateGenerator:
    @staticmethod
    def generate_cpp_file(fuzz_target: FuzzTarget, destination_path : str):
        __location__ = os.path.realpath(
            os.path.join(os.getcwd(), os.path.dirname(__file__))
        )

        # Read template
        try:
            with open(os.path.join(__location__, "template.cpp"), "r") as fd:
                template = fd.read()
                logging.info("Template read")

                # Populate template
                for key, value in fuzz_target.get_mapping().items():
                    template = template.replace(key, value)

                # Write the populated file
                try:
                    with open(os.path.join(destination_path, "generated.cpp"), "w") as fd:
                        logging.info(f"Template written with {fuzz_target.client_name} client")
                        fd.write(template)
                except Exception:
                    logging.error("Couldn't write generated file", exc_info=True)

        except Exception:
            logging.error("Couldn't read template", exc_info=True)