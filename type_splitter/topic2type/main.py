from __future__ import annotations
import subprocess
import re

primitives = {
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


class FieldOptions:
    pass


class Field:
    def __init__(self, name: str, type: Type, options: FieldOptions) -> None:
        self.name = name
        self.type = type
        self.options = options

    def __repr__(self) -> str:
        return f"{self.name} : {self.type}"


class Type:
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
            return self.type_name + ":\n" + "\n".join([repr(f) for f in self.fields])


prepath = "geometry_msgs/msg/"


def type_name2type(type_name: str) -> Type:
    # Base case
    if type_name in primitives:
        return Type(type_name=type_name, is_primitive=True)

    # Recursive case
    ros_interface_output = subprocess.check_output(
        ["ros2", "interface", "show", prepath + type_name]
    )
    ros_interface_output = ros_interface_output.decode("utf-8").splitlines()
    fields = [line2field(line) for line in ros_interface_output]
    fields = [field for field in fields if field is not None]
    return Type(type_name=type_name, fields=fields)


def line2field(line: str) -> Field:
    regex = r"^(?P<typeName>[\w/]+)(<=(?P<stringMaxSize>\d+))?(?P<array>\[((?P<boundedArraySize><=)?(?P<arraySize>\d+))?\])?\s+(?P<name>[^\s=]+)((\s|=)(?P<default>.*?))?$"
    regex_match = re.match(regex, line)
    if not regex_match:
        return None

    g = regex_match.groupdict()
    name = g["name"]
    type_name = g["typeName"]
    type = type_name2type(type_name)
    options = FieldOptions()

    return Field(name=name, type=type, options=options)


if __name__ == "__main__":
    print(type_name2type("Twist"))
