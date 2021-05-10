from __future__ import annotations

from zenlog import log as logging
import re

from .type_parser import Field, ROSType


class FuzzTarget:
    def __init__(
        self, imports: str, client_name: str, request_code: str, node_type: str
    ) -> None:
        self.imports = imports
        self.client_name = client_name
        self.request_code = request_code
        self.node_type = node_type

    def get_mapping(self) -> dict[str, str]:
        return {
            "IMPORTS": self.imports,
            "CLIENT_NAME": self.client_name,
            "REQUEST_CODE": self.request_code,
            "NODE_TYPE": self.node_type,
        }


class FuzzTargetProcessor:
    PRIMITIVES_CPP_TYPES = {
        "bool": "bool",
        "byte": "uint8_t",
        "char": "char",
        "float32": "float",
        "float64": "double",
        "int8": "int8_t",
        "uint8": "uint8_t",
        "int16": "int16_t",
        "uint16": "uint16_t",
        "int32": "int32_t",
        "uint32": "uint32_t",
        "int64": "int64_t",
        "uint64": "uint64_t",
        "string": "std::string",
        "wstring": "std::string",
    }

    def __init__(self) -> None:
        self.variable_counter = 0

    def get_fresh_variable(self) -> str:
        fresh_variable = "_v" + str(self.variable_counter)
        self.variable_counter += 1
        return fresh_variable

    @staticmethod
    def normalize_client_name(raw_client_name: str) -> str:
        return re.sub(r"(?<!^)(?=[A-Z])", "_", raw_client_name).lower()

    def fuzz_field(self, field: Field, parent="request", indent=1) -> str:
        logging.debug(f"Generating field {field.name}")
        fresh = self.get_fresh_variable()
        preindent = "    " * indent
        res = preindent + f"// {field.name}\n"

        # Primitive type
        if field.type.is_primitive:
            cpp_type = FuzzTargetProcessor.PRIMITIVES_CPP_TYPES[field.type.type_name]
            res += preindent + f"{cpp_type} {fresh};\n"
            res += (
                preindent
                + f"if (!get{field.type.type_name.capitalize()}({fresh})) return;\n"
            )
        # Composite type
        else:
            res += preindent + f"{field.type.type_name} {fresh};\n"
            for subfield in field.type.fields:
                res += preindent + self.fuzz_field(
                    subfield, parent=fresh, indent=indent + 1
                )
        res += preindent + f"{parent}->{field.name} = {fresh};\n"
        return res

    def process(
        self, t: ROSType, headers_file: str, original_file: str, ros_type_str: str
    ) -> FuzzTarget:
        logging.debug(f"Processing {t.type_name} type")
        imports = "\n".join(
            [
                f'#include "{headers_file}"',
                f'#include "{original_file}"',
            ]
        )
        request_code = "\n".join([self.fuzz_field(field) for field in t.fields])

        return FuzzTarget(
            imports=imports,
            client_name=FuzzTargetProcessor.normalize_client_name(t.type_name),
            request_code=request_code,
            node_type=ros_type_str,
        )
