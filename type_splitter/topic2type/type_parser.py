from __future__ import annotations
import subprocess
import re
import logging
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

    PREPATH = "tutorial_interfaces/srv/"

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
    def parse(type_name: str) -> ROSType:
        # Base case
        if type_name in TypeParser.PRIMITIVES:
            return ROSType(type_name=type_name, is_primitive=True)

        # Recursive case
        try:
            ros_interface_output = subprocess.check_output(
                ["ros2", "interface", "show", TypeParser.PREPATH + type_name]
            )
        except:
            logging.error(
                f"Couldn't call ros2 interface show {TypeParser.PREPATH + type_name}",
                exc_info=True,
            )
            return None

        ros_interface_output = ros_interface_output.decode("utf-8").splitlines()
        fields = [TypeParser.line2field(line) for line in ros_interface_output]
        fields = [field for field in fields if field is not None]
        return ROSType(type_name=type_name, fields=fields)
