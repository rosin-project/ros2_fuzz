import os
import unittest

from ros2_fuzzer.service_fuzzer.type_parser import TypeParser


class TypeParserTests(unittest.TestCase):
    @staticmethod
    def relative_file(path):
        return os.path.join(os.path.dirname(os.path.abspath(__file__)), path)

    def test_positive_tests(self):
        with open(self.relative_file("type_parser_positive_tests.txt")) as f:
            positive_tests = f.readlines()
        for field in positive_tests:
            with self.subTest(field=field):
                self.assertIsNotNone(TypeParser.line2field(field))

    def test_negative_tests(self):
        with open(self.relative_file("type_parser_negative_tests.txt")) as f:
            negative_tests = f.readlines()
        for field in negative_tests:
            with self.subTest(field=field):
                self.assertIsNone(TypeParser.line2field(field))

    def test_positive_primitive_type(self):
        field = "string my_string"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "string")
        self.assertEqual(processed.name, "my_string")
        self.assertFalse(processed.options.is_array)

    def test_default_with_equal(self):
        field = "int64 X=[25, 10, 3]"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "int64")
        self.assertEqual(processed.name, "X")
        self.assertEqual(processed.options.default, "[25, 10, 3]")
        self.assertFalse(processed.options.is_array)

    def test_default_with_space(self):
        field = "int64 X [25, 10, 3]"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "int64")
        self.assertEqual(processed.name, "X")
        self.assertEqual(processed.options.default, "[25, 10, 3]")
        self.assertFalse(processed.options.is_array)

    def test_bounded_string(self):
        field = "string<=15 length"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "string")
        self.assertEqual(processed.name, "length")
        self.assertEqual(processed.options.string_max_size, 15)
        self.assertFalse(processed.options.is_array)

    def test_fixed_array_length(self):
        field = "int64[5] values"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "int64")
        self.assertEqual(processed.name, "values")
        self.assertEqual(processed.options.array_size, 5)
        self.assertTrue(processed.options.is_array)
        self.assertFalse(processed.options.is_bounded_array_size)

    def test_bounded_array_length(self):
        field = "int64[<=5] values"
        processed = TypeParser.line2field(field)
        self.assertEqual(processed.type.type_name, "int64")
        self.assertEqual(processed.name, "values")
        self.assertEqual(processed.options.array_size, 5)
        self.assertTrue(processed.options.is_array)
        self.assertTrue(processed.options.is_bounded_array_size)
