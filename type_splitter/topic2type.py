import subprocess
import re

# primitives = {
#     "bool",
#     "byte",
#     "char",
#     "float32",
#     "float64",
#     "int8",
#     "uint8",
#     "int16",
#     "uint16",
#     "int32",
#     "uint32",
#     "int64",
#     "uint64",
#     "string",
#     "wstring",
# }


class Splitter:
    @staticmethod
    def split_message(type_name) -> tuple[list[str], list[str]]:
        # res = subprocess.check_output(['ros2', 'interface', 'show', type_name])
        res = b"int64 a\nint64 b\nint64 c\n---\nint64 sum\n"
        res = res.decode("utf-8")
        res = res.splitlines()
        whereDivide = res.index("---")
        request, response = res[:whereDivide], res[whereDivide + 1 :]
        return request, response

    @staticmethod
    def process_field(l) -> dict[str, str]:
        regex = r"^(?P<type>[\w/]+)(<=(?P<stringSize>\d+))?(?P<array>\[((?P<minimumLengthArray><=)?(?P<arraySize>\d+))?\])?\s+(?P<name>[^\s=]+)((\s|=)(?P<default>.*?))?$"
        res = re.match(regex, l)
        if not res:
            return None
        res = res.groupdict()
        return res
