from topic2type import Splitter
import unittest


class process_field_tests(unittest.TestCase):
    def test_positive_tests(self):
        with open("positive_tests.txt") as f:
            positive_tests = f.readlines()
        for field in positive_tests:
            with self.subTest(field=field):
                self.assertIsNotNone(Splitter.process_field(field))

    def test_negative_tests(self):
        with open("negative_tests.txt") as f:
            negative_tests = f.readlines()
        for field in negative_tests:
            with self.subTest(field=field):
                self.assertIsNone(Splitter.process_field(field))

    def test_positive_primitive_type(self):
        field = "string my_string"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "string")
        self.assertEqual(processed["name"], "my_string")
        self.assertIsNone(processed["array"])

    def test_positive_composite_type(self):
        field = "another_pkg/YetAnotherMessage val"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "another_pkg/YetAnotherMessage")
        self.assertEqual(processed["name"], "val")
        self.assertIsNone(processed["array"])

    def test_default_with_equal(self):
        field = "int64 X=[25, 10, 3]"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "int64")
        self.assertEqual(processed["name"], "X")
        self.assertEqual(processed["default"], "[25, 10, 3]")
        self.assertIsNone(processed["array"])

    def test_default_with_space(self):
        field = "int64 X [25, 10, 3]"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "int64")
        self.assertEqual(processed["name"], "X")
        self.assertEqual(processed["default"], "[25, 10, 3]")
        self.assertIsNone(processed["array"])

    def test_bounded_string(self):
        field = "string<=15 length"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "string")
        self.assertEqual(processed["name"], "length")
        self.assertEqual(processed["stringSize"], "15")
        self.assertIsNone(processed["array"])

    def test_fixed_array_length(self):
        field = "int64[5] values"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "int64")
        self.assertEqual(processed["name"], "values")
        self.assertEqual(processed["arraySize"], "5")
        self.assertIsNotNone(processed["array"])
        self.assertIsNone(processed["minimumLengthArray"])

    def test_bounded_array_length(self):
        field = "int64[<=5] values"
        processed = Splitter.process_field(field)
        self.assertEqual(processed["type"], "int64")
        self.assertEqual(processed["name"], "values")
        self.assertEqual(processed["arraySize"], "5")
        self.assertIsNotNone(processed["array"])
        self.assertIsNotNone(processed["minimumLengthArray"])
