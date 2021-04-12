import logging

from .fuzzing_descriptor import *
from .template_generator import *
from .type_parser import TypeParser


def generate_template(
    source: str, ros_type_str: str, headers_file: str
):
    original_file = os.path.basename(source)
    
    # Check that there are no TODOs
    if "TODO" in [source, ros_type_str, headers_file, original_file]:
        logging.error(
            f"Cannot create the fuzzer for {original_file}\n" "There are missing TODOs"
        )
        return
    
    # Check that the source files exist
    if not os.path.isfile(source):
        logging.error(f"The source {source} is not a valid path")
        return

    topic_name = ros_type_str.replace("::", "/")

    ros_type = TypeParser.parse_topic(topic_name)
    fuzz_target = FuzzTargetProcessor().process(
        ros_type, headers_file=headers_file, original_file=original_file
    )
    TemplateGenerator.generate_cpp_file(
        fuzz_target=fuzz_target,
        source_file=source,
    )
