from __future__ import annotations
import logging
from .type_parser import ROSType, Field
import re


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
            "{{ IMPORTS }}": self.imports,
            "{{ CLIENT_NAME }}": self.client_name,
            "{{ REQUEST_CODE }}": self.request_code,
            "{{ NODE_TYPE }}": self.node_type,
        }


class FuzzTargetProcesser:
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
        logging.info(f"Generating field {field.name}")
        fresh = self.get_fresh_variable()
        preindent = "    " * indent
        res = preindent + f"// {field.name}\n"
        res += preindent + f"{field.type.type_name} {fresh};\n"
        # Primitive type
        if field.type.is_primitive:
            res += preindent + f"get{field.type.type_name.capitalize()}({fresh});\n"
        # Composite type
        else:
            for subfield in field.type.fields:
                res += preindent + self.fuzz_field(
                    subfield, parent=fresh, indent=indent + 1
                )
        res += preindent + f"{parent}->{field.name} = {fresh};\n"
        return res

    def process(self, t: ROSType) -> FuzzTarget:
        logging.info(f"Processing {t.type_name} type")
        request_code = "\n".join([self.fuzz_field(field) for field in t.fields])

        return FuzzTarget(
            imports='#include "tutorial_interfaces/srv/add_three_ints.hpp"',
            client_name=FuzzTargetProcesser.normalize_client_name(t.type_name),
            request_code=request_code,
            node_type="tutorial_interfaces::srv::AddThreeInts",
        )