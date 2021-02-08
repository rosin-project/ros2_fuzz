from .type_parser import *
from .template_generator import *
from .fuzzing_descriptor import *

import logging

if __name__ == "__main__":
    logging.basicConfig(level=logging.DEBUG)

    ros_type: ROSType = TypeParser.parse("AddThreeInts")
    print("ros_type:", ros_type)
    fuzz_target: FuzzTarget = FuzzTargetProcesser().process(ros_type)
    print("fuzz_target:", fuzz_target)
    TemplateGenerator.generate_cpp_file(fuzz_target)
