# SPDX-FileCopyrightText: 2023-2023 Blender Authors
#
# SPDX-License-Identifier: Apache-2.0

# ./blender.bin --background -noaudio --python tests/python/bl_pyapi_prop_collection.py -- --verbose
import bpy
from bpy.props import (
    BoolProperty,
    BoolVectorProperty,
    FloatProperty,
    FloatVectorProperty,
    IntProperty,
    IntVectorProperty,
    EnumProperty,
    CollectionProperty,
)
import unittest
import numpy as np
import ctypes

id_inst = bpy.context.scene
id_type = bpy.types.Scene


# -----------------------------------------------------------------------------
# Utility Functions

def as_sequence_check_buffer(inst):
    """
    Wrap the buffer instance as a subclass of its type that tracks whether is has been used as a sequence.

    Once Python 3.12 is in use, this class could be replaced with a wrapper around any buffer that redirects all python
    calls to the wrapped buffer and defines a __buffer__ method that returns the buffer.
    """
    class SequenceCheckBuffer(type(inst)):
        """
        __getitem__ and __setitem__ are called by PySequence_GetItem and PySequence_SetItem (and also PyObject_GetItem
        and PyObject_SetItem).
        """
        def __getitem__(self, key):
            self.used_as_sequence = True
            return super().__getitem__(key)

        def __setitem__(self, key, value):
            self.used_as_sequence = True
            super().__setitem__(key, value)
    if isinstance(inst, np.ndarray):
        wrapped = inst.view(SequenceCheckBuffer)
    elif isinstance(inst, ctypes.Array):
        wrapped = SequenceCheckBuffer.from_buffer(inst)
    else:
        # Notably, `memoryview` cannot be subclassed.
        raise TypeError("Unsupported buffer type %s" % type(inst))
    # __init__ and __new__ are not reliably called, so set the `used_as_sequence` attribute here.
    wrapped.used_as_sequence = False
    return wrapped


def ctypes_void_p_array_as_forced_numeric(cls):
    """
    ctypes.c_void_p arrays return None instead of zero when accessed as a sequence, which breaks tests.

    This decorator returns a subtype of the input class, but where __getitem__ checks for `None` and then returns `0`
    instead.
    """
    assert issubclass(cls, ctypes.Array)
    assert cls._type_ == ctypes.c_void_p

    class ForcedNumericCVoidPArray(cls):
        def __getitem__(self, key):
            item = super().__getitem__(key)
            if item is None:
                return 0
            else:
                return item

    return ForcedNumericCVoidPArray


class TestPropertyGroup(bpy.types.PropertyGroup):
    VECTOR_SIZE = 3
    _ENUM_ITEMS = (("0", "Item 0", ""), ("1", "Item 1", ""))
    _ENUM_IDENTIFIER_TO_INDEX = {item[0]: i for i, item in enumerate(_ENUM_ITEMS)}

    DEFAULT_INT = 0  # Must be either 0 or 1 so that bool and enum tests can work properly.
    DEFAULT_BOOL = bool(DEFAULT_INT)
    DEFAULT_FLOAT = float(DEFAULT_INT)

    test_bool: BoolProperty(default=DEFAULT_BOOL)
    test_bool_vector: BoolVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_BOOL] * VECTOR_SIZE)
    test_float: FloatProperty(default=DEFAULT_FLOAT)
    test_float_vector: FloatVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_FLOAT] * VECTOR_SIZE)
    test_int: IntProperty(default=DEFAULT_INT)
    test_int_vector: IntVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_INT] * VECTOR_SIZE)
    test_unsigned_int: IntProperty(subtype='UNSIGNED', default=DEFAULT_INT)
    test_enum: EnumProperty(items=_ENUM_ITEMS, default=_ENUM_ITEMS[DEFAULT_INT][0])

    @property
    def test_enum_value(self):
        """
        Helper that gets the index of the current `self.test_enum` item or -1 if the current item is undefined (when the
        actual index of `self.test_enum` has been set out of bounds of the enum's items).

        This avoids any assumptions about how the default setter for enum properties stores the current index.
        """
        return self._ENUM_IDENTIFIER_TO_INDEX.get(self.test_enum, -1)


# -----------------------------------------------------------------------------
# Tests

class TestPropCollectionForeachGetSet(unittest.TestCase):
    def assertSequenceEqual(self, seq1, seq2, msg=None, seq_type=None):
        """
        Replace any NumPy arrays with lists to avoid comparison issues in TestCase.assertSequenceEqual() and then call
        super().assertSequenceEqual() with the new list arguments.

        `np.array([1,2,3]) == [1,2,3]` creates a `np.array([True, True, True])` rather than `True` or `False`, which
        causes TestCase.assertSequenceEqual() to raise a ValueError when attempting to do `if seq1 == seq2:`.
        """
        seq1 = seq1.tolist() if isinstance(seq1, np.ndarray) else seq1
        seq2 = seq2.tolist() if isinstance(seq2, np.ndarray) else seq2
        super().assertSequenceEqual(seq1, seq2, msg, seq_type)

    def assertSequenceAlmostEqual(self, seq1, seq2, places=None, msg=None, delta=None):
        """
        Similar to assertSequenceEqual, but intended for comparing float values.

        Compares using unittest.TestCase.assertAlmostEqual().
        """
        # Replace any numpy arrays with lists, so they can be compared with `==`.
        seq1 = seq1.tolist() if isinstance(seq1, np.ndarray) else seq1
        seq2 = seq2.tolist() if isinstance(seq2, np.ndarray) else seq2
        if not seq1 == seq2:
            self.assertEqual(len(seq1), len(seq2), msg=msg)
            for v1, v2 in zip(seq1, seq2):
                self.assertAlmostEqual(v1, v2, places=places, msg=msg, delta=delta)

    def setUp(self):
        bpy.utils.register_class(TestPropertyGroup)
        id_type.test_collection = CollectionProperty(type=TestPropertyGroup)
        self.collection = id_inst.test_collection
        self.num_items = 5
        self.num_vector_items = self.num_items * TestPropertyGroup.VECTOR_SIZE
        for _ in range(self.num_items):
            id_inst.test_collection.add()

    def tearDown(self):
        # Custom Properties save their data in ID properties by default. Deleting the Custom Property registration will
        # leave the ID properties behind, so ensure the ID properties are removed by clearing the collection before
        # deleting its registration.
        id_inst.test_collection.clear()
        del id_type.test_collection
        bpy.utils.unregister_class(TestPropertyGroup)

    @staticmethod
    def make_boolean_test_sequences(next_ndarray, next_list):
        return [
            list(map(bool, next_list())),
            next_ndarray(bool),  # "?", bool
        ]

    @staticmethod
    def make_int_test_sequences(next_ndarray, next_list):
        sequences = [
            next_list(),
            next_ndarray(np.byte),       # "b", signed char
            next_ndarray(np.short),      # "h", short
            next_ndarray(np.intc),       # "i", int
            next_ndarray(np.int_),       # "l", long
            next_ndarray(np.longlong),   # "q", long long
            next_ndarray(np.ubyte),      # "B", unsigned char
            next_ndarray(np.ushort),     # "H", unsigned short
            next_ndarray(np.uintc),      # "I", unsigned int
            next_ndarray(np.uint),       # "L", unsigned long
            next_ndarray(np.ulonglong),  # "Q", unsigned long long
        ]

        # ssize_t ("n") format buffers are not supported by NumPy or ctypes.
        ssize_t_initializer = next_list()
        ssize_t_array = (ctypes.c_ssize_t * len(ssize_t_initializer))(*ssize_t_initializer)
        # ctypes exposed its ssize_t arrays as buffers with format set to one of the other integer types with
        # the same itemsize instead of the "n" format. A memoryview using the "n" format can be created by
        # casting.
        ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")
        sequences.append(ssize_t_array_as_n)

        # size_t ("N") format buffers are not supported by NumPy or ctypes.
        size_t_initializer = next_list()
        size_t_array = (ctypes.c_size_t * len(size_t_initializer))(*size_t_initializer)
        # ctypes exposed its size_t arrays as buffers with format set to one of the other integer types with the
        # same itemsize instead of the "N" format. A memoryview using the "N" format can be created by casting.
        size_t_array_as_n = memoryview(size_t_array).cast("b").cast("N")
        sequences.append(size_t_array_as_n)

        # void* ("P") format buffers are not supported by NumPy but are supported by ctypes.
        void_p_initializer = next_list()
        # ctypes void_p arrays return None instead of `0` when read as a sequence, which breaks foreach_set and
        # breaks comparing results with `self.assertSequenceEqual()`. We create a subclass which always returns
        # `0` whenever the c_void_p array superclass would return `None`.
        void_p_array_type = ctypes_void_p_array_as_forced_numeric(ctypes.c_void_p * len(void_p_initializer))
        void_p_array = void_p_array_type(*void_p_initializer)
        # Likely has a byteorder/size/alignment prefix so only check for "P" in the format string.
        assert "P" in memoryview(void_p_array).format
        sequences.append(void_p_array)

        return sequences

    @staticmethod
    def make_float_test_sequences(next_ndarray, next_list):
        return [
            list(map(float, next_list())),
            next_ndarray(np.half),        # "e", 16-bit-precision floating-point (no corresponding C type)
            next_ndarray(np.single),      # "f", float
            next_ndarray(np.double),      # "d", double
            next_ndarray(np.longdouble),  # "g", long double
        ]

    def make_subtest_sequences(self, prop_name, is_set):
        # To correctly handle all properties with the same test setup ('BOOLEAN' and 'ENUM' being the most restrictive),
        # we can only use `0` and `1` as fill values.
        default_int = TestPropertyGroup.DEFAULT_INT
        assert default_int in {0, 1}
        if is_set:
            last_fill_value = default_int
            # When setting values, each time we set, we need to set to different values from before so that we can check
            # that setting the values worked.

            def next_fill_value():
                nonlocal last_fill_value
                if last_fill_value == 0:
                    last_fill_value = 1
                else:
                    last_fill_value = 0
                return last_fill_value
        else:
            # When getting values, the initial values in the sequence must differ from the default values of the
            # properties, so that we can check that getting the values worked.
            def next_fill_value():
                return 1 if default_int == 0 else 0

        prop = TestPropertyGroup.bl_rna.properties[prop_name]
        item_length = prop.array_length if getattr(prop, "is_array", False) else 1
        sequence_length = self.num_items * item_length

        # Helper to create NumPy ndarrays filled with `next_fill_value()`
        def next_ndarray(dtype):
            return np.full(sequence_length, next_fill_value(), dtype=dtype)

        # Helper to create lists filled with `next_fill_value()`
        def next_list():
            return [next_fill_value()] * sequence_length

        match prop.type:
            case 'BOOLEAN':
                return self.make_boolean_test_sequences(next_ndarray, next_list)
            case 'INT' | 'ENUM':
                return self.make_int_test_sequences(next_ndarray, next_list)
            case 'FLOAT':
                return self.make_float_test_sequences(next_ndarray, next_list)
            case _:
                raise TypeError("Unsupported property type '%s'" % prop.type)

    def prop_as_sequence(self, prop_name):
        """
        Helper to get a flat sequence of a specific property through iteration instead of foreach_get.
        """
        if "vector" in prop_name:
            return [x for group in self.collection for x in getattr(group, prop_name)]
        elif prop_name == "test_enum":
            # Enum properties are a special case where foreach_get/set access the enum as an index, but accessing the
            # enum property directly is done using the string identifiers of the enum's items. TestPropertyGroup has an
            # extra `test_enum_value` property that gets the index of the current item instead of its string identifier.
            return [getattr(group, "test_enum_value") for group in self.collection]
        else:
            return [getattr(group, prop_name) for group in self.collection]

    def do_get_subtest(self, prop_name, seq):
        expected_sequence = self.prop_as_sequence(prop_name)

        accessed_as_buffer = False
        if isinstance(seq, (np.ndarray, ctypes.Array)):
            # View the sequence as a subtype that allows us to check whether the sequence was accessed as a sequence or
            # as a buffer.
            buffer_with_sequence_check = as_sequence_check_buffer(seq)
            self.collection.foreach_get(prop_name, buffer_with_sequence_check)
            # If the sequence wasn't used at all, this can result in a false positive, but assertions should fail in
            # that case.
            accessed_as_buffer = not buffer_with_sequence_check.used_as_sequence
        elif isinstance(seq, memoryview):
            # memoryview objects are not writable as a sequence, only readable, so if they are not accessed as a buffer,
            # writing to them as a sequence will fail (the memoryview type only implements PyObject_SetItem and not
            # PySequence_SetItem).
            try:
                self.collection.foreach_get(prop_name, seq)
            except TypeError:
                # Most likely the memoryview was accessed as a sequence.
                return False
            else:
                accessed_as_buffer = True
        else:
            self.collection.foreach_get(prop_name, seq)

        if "float" in prop_name:
            self.assertSequenceAlmostEqual(seq, expected_sequence)
        else:
            self.assertSequenceEqual(seq, expected_sequence)

        return accessed_as_buffer

    def do_set_subtest(self, prop_name, seq):
        accessed_as_buffer = False
        if isinstance(seq, (np.ndarray, ctypes.Array)):
            # View the sequence as a subtype that allows us to check whether the sequence was accessed as a sequence or
            # as a buffer.
            buffer_with_sequence_check = as_sequence_check_buffer(seq)
            self.collection.foreach_set(prop_name, buffer_with_sequence_check)
            # If the sequence wasn't used at all, this can result in a false positive, but assertions should fail in
            # that case.
            accessed_as_buffer = not buffer_with_sequence_check.used_as_sequence
        elif 0:  # isinstance(seq, memoryview):
            # There is currently no way to tell if a memoryview was accessed as a sequence or as a buffer. Once Python
            # 3.12 is in use, this may become possible because Python 3.12 makes the buffer protocol accessible to
            # Python.
            self.collection.foreach_set(prop_name, seq)
        else:
            self.collection.foreach_set(prop_name, seq)

        result_sequence = self.prop_as_sequence(prop_name)
        expected_sequence = seq

        if "float" in prop_name:
            self.assertSequenceAlmostEqual(result_sequence, expected_sequence)
        else:
            self.assertSequenceEqual(result_sequence, expected_sequence)
        return accessed_as_buffer

    @staticmethod
    def get_sequence_description(seq):
        """
        Helper to get a description of an input sequence, to be printed when a subtest fails.
        """
        if isinstance(seq, list):
            return "list ('%s')" % (type(seq[0]).__name__ if seq else "None")
        elif isinstance(seq, np.ndarray):
            element_type = seq.dtype
            return "%s ('%s')" % (element_type, seq.data.format)
        elif isinstance(seq, ctypes.Array):
            element_type_name = seq._type_.__name__
            return "%s ('%s')" % (element_type_name, memoryview(seq).format)
        elif isinstance(seq, memoryview):
            return "memoryview ('%s')" % seq.format
        else:
            raise TypeError("Unsupported type '%s'" % type(seq))

    def do_foreach_getset_subtests(self, *prop_names, is_set):
        subtest_func = self.do_set_subtest if is_set else self.do_get_subtest
        for prop_name in prop_names:
            with self.subTest(prop_name=prop_name):
                sequences = self.make_subtest_sequences(prop_name, is_set=is_set)
                # Due to varying itemsizes of C types depending on the current system, and the itemsize of a property
                # not being exposed to Blender's Python API, we only check that at least one sequence was considered a
                # compatible buffer.
                at_least_one_accessed_as_buffer = False

                for seq in sequences:
                    with self.subTest(seq_type=self.get_sequence_description(seq)):
                        accessed_as_buffer = subtest_func(prop_name, seq)
                        at_least_one_accessed_as_buffer |= accessed_as_buffer

                self.assertTrue(at_least_one_accessed_as_buffer, "at least one sequence should be accessed as a buffer")

    def do_foreach_get_subtests(self, *prop_names):
        self.do_foreach_getset_subtests(*prop_names, is_set=False)

    def do_foreach_set_subtests(self, *prop_names):
        self.do_foreach_getset_subtests(*prop_names, is_set=True)

    # Test methods

    def test_foreach_get_bool(self):
        self.do_foreach_get_subtests("test_bool", "test_bool_vector")

    def test_foreach_set_bool(self):
        self.do_foreach_set_subtests("test_bool", "test_bool_vector")

    def test_foreach_get_float(self):
        self.do_foreach_get_subtests("test_float", "test_float_vector")

    def test_foreach_set_float(self):
        self.do_foreach_set_subtests("test_float", "test_float_vector")

    def test_foreach_get_int(self):
        self.do_foreach_get_subtests("test_int", "test_int_vector", "test_unsigned_int")

    def test_foreach_set_int(self):
        self.do_foreach_set_subtests("test_int", "test_int_vector", "test_unsigned_int")

    @unittest.expectedFailure  # See #92621
    def test_foreach_get_enum(self):
        self.do_foreach_get_subtests("test_enum")

    @unittest.expectedFailure  # See #92621
    def test_foreach_set_enum(self):
        self.do_foreach_set_subtests("test_enum")


if __name__ == '__main__':
    import sys
    sys.argv = [__file__] + (sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    unittest.main()
