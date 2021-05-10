from __future__ import annotations

from zenlog import log as logging
import re
import subprocess
from typing import Optional


class FieldOptions:
    def __init__(
        self,
        is_array: bool,
        default: str,
        string_max_size: Optional[int],
        array_size: Optional[int],
        is_bounded_array_size: Optional[bool],
    ) -> None:
        self.is_array = is_array
        self.default = default
        self.string_max_size = string_max_size
        self.array_size = array_size
        self.is_bounded_array_size = is_bounded_array_size


class Field:
    def __init__(self, name: str, type: ROSType, options: FieldOptions = None) -> None:
        self.name = name
        self.type = type
        self.options = options

    def __repr__(self) -> str:
        return f"{self.name} : {self.type}"


class ROSType:
    def __init__(
        self, type_name: str, is_primitive: bool = False, fields: list[Field] = []
    ) -> None:
        self.type_name = type_name
        self.fields = fields
        self.is_primitive = is_primitive

    def __repr__(self) -> str:
        if self.is_primitive:
            return self.type_name
        else:
            return (
                self.type_name
                + ":\n"
                + "\n".join(["\t" + repr(f) for f in self.fields])
            )


class TypeParser:
    PRIMITIVES = {
        "bool",
        "byte",
        "char",
        "float32",
        "float64",
        "int8",
        "uint8",
        "int16",
        "uint16",
        "int32",
        "uint32",
        "int64",
        "uint64",
        "string",
        "wstring",
    }

    @staticmethod
    def line2field(line: str) -> Field:
        regex = r"^(?P<typeName>[\w/]+)(<=(?P<stringMaxSize>\d+))?(?P<array>\[((?P<boundedArraySize><=)?(?P<arraySize>\d+))?\])?\s+(?P<name>[^\s=]+)((\s|=)(?P<default>.*?))?$"
        regex_match = re.match(regex, line)
        if not regex_match:
            return None

        g = regex_match.groupdict()
        name = g["name"]
        type_name = g["typeName"]
        type = TypeParser.parse(type_name)

        is_array = g["array"] is not None
        default = g["default"]
        string_max_size = (
            int(g["stringMaxSize"]) if g["stringMaxSize"] is not None else None
        )
        array_size = int(g["arraySize"]) if g["arraySize"] is not None else None
        is_bounded_array_size = g["boundedArraySize"] is not None

        options = FieldOptions(
            is_array=is_array,
            default=default,
            string_max_size=string_max_size,
            array_size=array_size,
            is_bounded_array_size=is_bounded_array_size,
        )

        return Field(name=name, type=type, options=options)

    @staticmethod
    def parse(type_name: str, prepath: str = "") -> ROSType:
        # Base case
        if type_name in TypeParser.PRIMITIVES:
            return ROSType(type_name=type_name, is_primitive=True)

        # Recursive case
        full_topic_path = prepath + type_name
        try:
            ros2_process_result = subprocess.check_output(
                ["ros2", "interface", "show", full_topic_path],
                stderr=subprocess.STDOUT,
            )
        except:
            logging.error(
                f"Couldn't call `ros2 interface show {full_topic_path}`\n"
                "Have you sourced install/setup.bash?"
            )
            exit(-1)

        ros2_process_result = ros2_process_result.decode("utf-8")

        # If it is a *.srv file content (request --- response),
        # isolate and process only the request part
        if "---" in ros2_process_result:
            ros2_process_result = ros2_process_result.split("---")[0]
        ros2_process_result = ros2_process_result.splitlines()
        fields = [TypeParser.line2field(line) for line in ros2_process_result]
        fields = [field for field in fields if field is not None]
        return ROSType(type_name=type_name, fields=fields)

    @staticmethod
    def parse_type(type_name: str) -> ROSType:
        if "/" not in type_name:
            logging.error(
                f"The type name `{type_name}` does not contain any slash (/).\n"
                f'Please write the whole path (i.e. "tutorial_interfaces/srv/AddThreeInts")'
            )
            exit(-1)

        prepath, type_name = type_name.rsplit("/", 1)
        prepath += "/"

        return TypeParser.parse(type_name=type_name, prepath=prepath)
