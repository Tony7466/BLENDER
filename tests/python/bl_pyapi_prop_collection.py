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

id_inst = bpy.context.scene
id_type = bpy.types.Scene


# -----------------------------------------------------------------------------
# Utility Classes

class SequenceCheckBuffer(np.ndarray):
    """
    np.ndarray subtype that tracks whether it has been used as a sequence.

    Used in tests that expect the buffer to be accessed as a buffer without falling back to being accessed as a
    sequence.
    """
    def __array_finalize__(self, obj):
        self.used_as_sequence = False

    def __getitem__(self, key):
        self.used_as_sequence = True
        return super().__getitem__(key)

    def __setitem__(self, key, value):
        self.used_as_sequence = True
        super().__setitem__(key, value)


class TestPropertyGroup(bpy.types.PropertyGroup):
    VECTOR_SIZE = 3
    _ENUM_ITEMS = (("0", "Item 0", ""), ("1", "Item 1", ""))
    _ENUM_IDENTIFIER_TO_INDEX = {item[0]: i for i, item in enumerate(_ENUM_ITEMS)}

    test_bool: BoolProperty(default=False)
    test_bool_vector: BoolVectorProperty(size=VECTOR_SIZE, default=[False, False, False])
    test_float: FloatProperty(default=-1.0)
    test_float_vector: FloatVectorProperty(size=VECTOR_SIZE, default=[-1.0, -2.0, -3.0])
    test_int: IntProperty(default=-1)
    test_int_vector: IntVectorProperty(size=VECTOR_SIZE, default=[-1, -2, -3])
    test_unsigned_int: IntProperty(subtype='UNSIGNED', default=2 ** 31 - 1)
    test_enum: EnumProperty(items=_ENUM_ITEMS, default=_ENUM_ITEMS[0][0])

    @property
    def test_enum_value(self):
        """
        Helper that gets the index of the current `self.test_enum` item or -1 if the current item is undefined (when the
        actual index of `self.test_enum` has been set out of bounds of the enum's items).

        This avoids any assumptions about how the default setter for enum properties stores the current index.
        """
        return self._ENUM_IDENTIFIER_TO_INDEX.get(self.test_enum, -1)
    # It's not currently possible to create a custom property that is stored as a short or double, though these types
    # are uncommon to be used by RNA properties to begin with.


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

    def do_getset_test(self, seq_or_ndarray, foreach_attribute, is_get):
        """
        Helper function to reduce duplicate code for foreach_get/foreach_set tests that are expected to pass.
        """
        is_buffer = isinstance(seq_or_ndarray, np.ndarray)
        if is_buffer:
            # np.ndarray are viewed as the SequenceCheckBuffer subclass so that tests can check whether the buffer was
            # accessed as a sequence.
            foreach_arg = seq_or_ndarray.view(SequenceCheckBuffer)
        else:
            with self.assertRaises(TypeError, msg="Buffers that are not np.ndarray are not allowed because it will be"
                                                  " impossible to tell whether the buffer was accessed using the"
                                                  " sequence protocol or the buffer protocol in foreach_get"
                                                  "/foreach_set."):
                # Raises TypeError for any argument which is not a buffer.
                memoryview(seq_or_ndarray)
            foreach_arg = seq_or_ndarray

        if is_get:
            self.collection.foreach_get(foreach_attribute, foreach_arg)
        else:
            self.collection.foreach_set(foreach_attribute, foreach_arg)

        # Enum properties are a special case where we can't get property values directly.
        prop_name = "test_enum_value" if foreach_attribute == "test_enum" else foreach_attribute

        if "vector" in foreach_attribute:
            flat_collection_sequence = [x for group in self.collection for x in getattr(group, prop_name)]
        else:
            flat_collection_sequence = [getattr(group, prop_name) for group in self.collection]

        # Order the arguments so that the expected sequence is always the second argument.
        if is_get:
            self.assertSequenceEqual(seq_or_ndarray, flat_collection_sequence)
        else:
            self.assertSequenceEqual(flat_collection_sequence, seq_or_ndarray)

        if is_buffer:
            # To ensure that buffer code is being tested, check that the buffer was not accessed as a sequence.
            self.assertFalse(foreach_arg.used_as_sequence)

    def do_get_test(self, *args, **kwargs):
        self.do_getset_test(*args, **kwargs, is_get=True)

    def do_set_test(self, *args, **kwargs):
        self.do_getset_test(*args, **kwargs, is_get=False)

    def test_buffer_get_bool(self):
        self.do_get_test(np.full(self.num_items, True, dtype=bool), "test_bool")

    def test_buffer_set_bool(self):
        self.do_set_test(np.full(self.num_items, True, dtype=bool), "test_bool")

    def test_sequence_get_bool(self):
        self.do_get_test([None] * self.num_items, "test_bool")

    def test_sequence_set_bool(self):
        self.do_set_test([True] * self.num_items, "test_bool")

    def test_buffer_get_bool_vector(self):
        self.do_get_test(np.full(self.num_vector_items, True, dtype=bool), "test_bool_vector")

    def test_buffer_set_bool_vector(self):
        self.do_set_test(np.full(self.num_vector_items, True, dtype=bool), "test_bool_vector")

    def test_sequence_get_bool_vector(self):
        self.do_get_test([None] * self.num_vector_items, "test_bool_vector")

    def test_sequence_set_bool_vector(self):
        self.do_set_test([True] * self.num_vector_items, "test_bool_vector")

    def test_buffer_get_float(self):
        self.do_get_test(np.zeros(self.num_items, dtype=np.float32), "test_float")

    def test_buffer_set_float(self):
        self.do_set_test(np.arange(self.num_items, dtype=np.float32), "test_float")

    def test_sequence_get_float(self):
        self.do_get_test([None] * self.num_items, "test_float")

    def test_sequence_set_float(self):
        self.do_set_test(range(self.num_items), "test_float")

    def test_buffer_get_float_vector(self):
        self.do_get_test(np.zeros(self.num_vector_items, dtype=np.float32), "test_float_vector")

    def test_buffer_set_float_vector(self):
        self.do_set_test(np.arange(self.num_vector_items, dtype=np.float32), "test_float_vector")

    def test_sequence_get_float_vector(self):
        self.do_get_test([None] * self.num_vector_items, "test_float_vector")

    def test_sequence_set_float_vector(self):
        self.do_set_test(range(self.num_vector_items), "test_float_vector")

    def test_buffer_get_int(self):
        self.do_get_test(np.zeros(self.num_items, dtype=np.int32), "test_int")

    def test_buffer_set_int(self):
        self.do_set_test(np.arange(self.num_items, dtype=np.int32), "test_int")

    def test_sequence_get_int(self):
        self.do_get_test([None] * self.num_items, "test_int")

    def test_sequence_set_int(self):
        self.do_set_test(range(self.num_items), "test_int")

    def test_buffer_get_int_vector(self):
        self.do_get_test(np.zeros(self.num_vector_items, dtype=np.int32), "test_int_vector")

    def test_buffer_set_int_vector(self):
        self.do_set_test(np.arange(self.num_vector_items, dtype=np.int32), "test_int_vector")

    def test_sequence_get_int_vector(self):
        self.do_get_test([None] * self.num_vector_items, "test_int_vector")

    def test_sequence_set_int_vector(self):
        self.do_set_test(range(self.num_vector_items), "test_int_vector")

    def test_buffer_get_unsigned_int(self):
        self.do_get_test(np.zeros(self.num_items, dtype=np.uint32), "test_unsigned_int")

    def test_buffer_set_unsigned_int(self):
        self.do_set_test(np.arange(self.num_items, dtype=np.uint32), "test_unsigned_int")

    def test_sequence_get_unsigned_int(self):
        self.do_get_test([None] * self.num_items, "test_unsigned_int")

    def test_sequence_set_unsigned_int(self):
        self.do_set_test(range(self.num_items), "test_unsigned_int")

    @unittest.expectedFailure  # See #92621
    def test_buffer_get_enum(self):
        # Note: RNA enum properties commonly vary in itemsize, int32 may not be correct for other properties.
        self.do_get_test(np.ones(self.num_items, dtype=np.int32), "test_enum")

    @unittest.expectedFailure  # See #92621
    def test_buffer_set_enum(self):
        # Note: RNA enum properties commonly vary in itemsize, int32 may not be correct for other properties.
        self.do_set_test(np.ones(self.num_items, dtype=np.int32), "test_enum")

    @unittest.expectedFailure  # See #92621
    def test_sequence_get_enum(self):
        self.do_get_test([None] * self.num_items, "test_enum")

    @unittest.expectedFailure  # See #92621
    def test_sequence_set_enum(self):
        self.do_get_test([1] * self.num_items, "test_enum")


if __name__ == '__main__':
    import sys
    sys.argv = [__file__] + (sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    unittest.main()
