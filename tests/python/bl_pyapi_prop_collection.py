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
    test_unsigned_int: IntProperty(subtype='UNSIGNED', default=255)
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

    def assertSequenceFallbackNotUsed(self, buffer_with_sequence_check):
        """
        Assert that a buffer with sequence checking has not been used as a sequence. Used to check that a buffer was not
        used by foreach_get/foreach_set's sequence fallback.
        """
        self.assertFalse(buffer_with_sequence_check.used_as_sequence, "Buffer was considered incompatible and the"
                                                                      " sequence fallback was used.")

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

    def get_num_items(self, prop_name):
        """
        Helper to get the flattened number of items of a property.
        """
        return self.num_vector_items if "vector" in prop_name else self.num_items

    def get_sequence(self, prop_name):
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

    @staticmethod
    def get_nth_arange(n, num_items, dtype=None):
        """
        Helper to create the nth np.arange with num_items items.
        """
        start = n * num_items
        end_excl = start + num_items
        return np.arange(start, end_excl, dtype=dtype)

    def do_test_sequence_get(self, prop_name):
        expected_sequence = self.get_sequence(prop_name)
        sequence = [None] * self.get_num_items(prop_name)

        self.collection.foreach_get(prop_name, sequence)

        if "float" in prop_name:
            self.assertSequenceAlmostEqual(sequence, expected_sequence)
        else:
            self.assertSequenceEqual(sequence, expected_sequence)

    def do_test_sequence_set(self, prop_name, sequence=None):
        if sequence is None:
            sequence = range(self.get_num_items(prop_name))

        self.collection.foreach_set(prop_name, sequence)

        result_sequence = self.get_sequence(prop_name)
        expected_sequence = sequence
        if "float" in prop_name:
            self.assertSequenceAlmostEqual(result_sequence, expected_sequence)
        else:
            self.assertSequenceEqual(result_sequence, expected_sequence)

    def test_sequence_get_bool(self):
        for prop_name in ("test_bool", "test_bool_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_get(prop_name)

    def test_sequence_set_bool(self):
        for prop_name in ("test_bool", "test_bool_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_set(prop_name, [True] * self.get_num_items(prop_name))

    def test_buffer_get_bool(self):
        for prop_name in ("test_bool", "test_bool_vector"):
            with self.subTest(prop_name=prop_name):
                expected_sequence = self.get_sequence(prop_name)
                buffer = np.full(self.get_num_items(prop_name), True, dtype=bool)
                buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                self.collection.foreach_get(prop_name, buffer_with_sequence_check)

                self.assertSequenceFallbackNotUsed(buffer_with_sequence_check)
                self.assertSequenceEqual(buffer, expected_sequence)

    def test_buffer_set_bool(self):
        for prop_name in ("test_bool", "test_bool_vector"):
            with self.subTest(prop_name=prop_name):
                buffer = np.full(self.get_num_items(prop_name), True, dtype=bool)
                buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                self.collection.foreach_set(prop_name, buffer_with_sequence_check)

                self.assertSequenceFallbackNotUsed(buffer_with_sequence_check)
                result_sequence = self.get_sequence(prop_name)
                expected_sequence = buffer
                self.assertSequenceEqual(result_sequence, expected_sequence)

    def test_sequence_get_float(self):
        for prop_name in ("test_float", "test_float_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_get(prop_name)

    def test_sequence_set_float(self):
        for prop_name in ("test_float", "test_float_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_set(prop_name)

    def test_buffer_get_float(self):
        float_types = (np.half, np.single, np.double, np.longdouble)
        for prop_name in ("test_float", "test_float_vector"):
            with self.subTest(prop_name=prop_name):
                expected_sequence = self.get_sequence(prop_name)

                compatible_itemsizes = set()
                for float_type in float_types:
                    buffer = np.zeros(self.get_num_items(prop_name), dtype=float_type)
                    buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                    self.collection.foreach_get(prop_name, buffer_with_sequence_check)

                    self.assertSequenceAlmostEqual(buffer, expected_sequence)
                    if not buffer_with_sequence_check.used_as_sequence:
                        compatible_itemsizes.add(buffer.itemsize)

                self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                               "compatible itemsizes were '%s'" % compatible_itemsizes)

    def test_buffer_set_float(self):
        float_types = (np.half, np.single, np.double, np.longdouble)
        for prop_name in ("test_float", "test_float_vector"):
            with self.subTest(prop_name=prop_name):
                num_items = self.get_num_items(prop_name)

                compatible_itemsizes = set()
                for i, float_type in enumerate(float_types):
                    # Ensure each buffer has different contents from the previous buffer.
                    buffer = self.get_nth_arange(i, num_items, dtype=float_type)
                    buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                    self.collection.foreach_set(prop_name, buffer_with_sequence_check)

                    result_sequence = self.get_sequence(prop_name)
                    expected_sequence = buffer
                    self.assertSequenceAlmostEqual(result_sequence, expected_sequence)
                    if not buffer_with_sequence_check.used_as_sequence:
                        compatible_itemsizes.add(buffer.itemsize)

                self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                               "compatible itemsizes were '%s'" % compatible_itemsizes)

    def test_sequence_get_int(self):
        for prop_name in ("test_int", "test_int_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_get(prop_name)

    def test_sequence_set_int(self):
        for prop_name in ("test_int", "test_int_vector"):
            with self.subTest(prop_name=prop_name):
                self.do_test_sequence_set(prop_name)

    def test_buffer_get_int(self):
        for prop_name in ("test_int", "test_int_vector"):
            with self.subTest(prop_name=prop_name):
                expected_sequence = self.get_sequence(prop_name)
                num_items = self.get_num_items(prop_name)

                int_types = (np.byte, np.short, np.intc, np.int_, np.longlong)
                compatible_itemsizes = set()
                for int_type in int_types:
                    buffer = np.zeros(num_items, dtype=int_type)
                    buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                    self.collection.foreach_get(prop_name, buffer_with_sequence_check)

                    self.assertSequenceEqual(buffer, expected_sequence)
                    if not buffer_with_sequence_check.used_as_sequence:
                        compatible_itemsizes.add(buffer.itemsize)
                self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                               "compatible itemsizes were '%s'" % compatible_itemsizes)

                # ssize_t ("n") format is not supported by NumPy.
                ssize_t_array = (ctypes.c_ssize_t * num_items)()
                # ctypes exposed its ssize_t arrays as buffers with one of the other integer types with the same size
                # instead of the "n" format. A memoryview using the "n" format can be created by casting.
                ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")
                # memoryview objects are not writable as a sequence (only readable), so they are only expected to work
                # when they are the correct itemsize to be considered a compatible buffer.
                if ssize_t_array_as_n.itemsize in compatible_itemsizes:
                    self.collection.foreach_get(prop_name, ssize_t_array_as_n)

                    self.assertSequenceEqual(ssize_t_array_as_n, expected_sequence)
                else:
                    with self.assertRaises(TypeError):
                        self.collection.foreach_get(prop_name, ssize_t_array_as_n)

    def test_buffer_set_int(self):
        for prop_name in ("test_int", "test_int_vector"):
            with self.subTest(prop_name=prop_name):
                num_items = self.get_num_items(prop_name)

                # ssize_t ("n") format is not supported by NumPy.
                ssize_t_array = (ctypes.c_ssize_t * num_items)(*self.get_nth_arange(0, num_items))
                # ctypes exposes its ssize_t arrays as buffers with one of the other integer types with the same size
                # instead of the "n" format. A memoryview using the "n" format can be created by casting.
                ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")

                self.collection.foreach_set(prop_name, ssize_t_array_as_n)

                result_sequence = self.get_sequence(prop_name)
                expected_sequence = ssize_t_array_as_n
                self.assertSequenceEqual(result_sequence, expected_sequence)
                # `memoryview` cannot be subclassed, so it is not possible to check if it was accessed as a buffer or
                # sequence.

                int_types = (np.byte, np.short, np.intc, np.int_, np.longlong)
                compatible_itemsizes = set()
                # ssize_t has been done beforehand, so start at 1.
                for i, int_type in enumerate(int_types, start=1):
                    # Ensure each buffer has different contents from the previous buffer.
                    buffer = self.get_nth_arange(i, num_items, dtype=int_type)
                    buffer_with_sequence_check = as_sequence_check_buffer(buffer)

                    self.collection.foreach_set(prop_name, buffer_with_sequence_check)

                    result_sequence = self.get_sequence(prop_name)
                    self.assertSequenceEqual(result_sequence, buffer)
                    if not buffer_with_sequence_check.used_as_sequence:
                        compatible_itemsizes.add(buffer.itemsize)
                self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                               "compatible itemsizes were '%s'" % compatible_itemsizes)

    def test_sequence_get_unsigned_int(self):
        self.do_test_sequence_get("test_unsigned_int")

    def test_sequence_set_unsigned_int(self):
        self.do_test_sequence_set("test_unsigned_int")

    def test_buffer_get_unsigned_int(self):
        expected_sequence = self.get_sequence("test_unsigned_int")
        num_items = self.get_num_items("test_unsigned_int")

        np_uint_types = (np.ubyte, np.ushort, np.uintc, np.uint, np.ulonglong)
        uint_buffers = [np.zeros(num_items, dtype=uint_type) for uint_type in np_uint_types]
        # void* ('P') format arrays are not supported by NumPy but are supported by ctypes.
        void_p_array_type = ctypes.c_void_p * num_items
        uint_buffers.append(void_p_array_type())
        compatible_itemsizes = set()
        for buffer in uint_buffers:
            buffer_with_sequence_check = as_sequence_check_buffer(buffer)

            self.collection.foreach_get("test_unsigned_int", buffer_with_sequence_check)

            self.assertSequenceEqual(buffer, expected_sequence)
            if not buffer_with_sequence_check.used_as_sequence:
                # View as a memoryview so there's a common interface to get the itemsize.
                compatible_itemsizes.add(memoryview(buffer).itemsize)
        self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                       "compatible itemsizes were '%s'" % compatible_itemsizes)

        # size_t ('N') format is not supported by NumPy.
        size_t_array_type = ctypes.c_size_t * num_items
        # ctypes exposed its size_t arrays as buffers with one of the other integer types with the same size instead of
        # the "N" format. A memoryview using the "N" format can be created by casting.
        size_t_array_as_n = memoryview(size_t_array_type()).cast("b").cast("N")
        # memoryview objects are not writable as a sequence (only readable), so they are only expected to work when they
        # are the correct itemsize to be considered a compatible buffer.
        if size_t_array_as_n.itemsize in compatible_itemsizes:
            self.collection.foreach_get("test_unsigned_int", size_t_array_as_n)

            self.assertSequenceEqual(size_t_array_as_n, expected_sequence)
        else:
            with self.assertRaises(TypeError):
                self.collection.foreach_get("test_unsigned_int", size_t_array_as_n)

    def test_buffer_set_unsigned_int(self):
        num_items = self.get_num_items("test_unsigned_int")

        # ssize_t ("n") format is not supported by NumPy.
        ssize_t_array_type = ctypes.c_ssize_t * num_items
        ssize_t_array = ssize_t_array_type(*self.get_nth_arange(0, num_items))
        # ctypes exposed its ssize_t arrays as buffers with one of the other integer types with the same size instead of
        # the "n" format. A memoryview using the "n" format can be created by casting.
        ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")
        self.collection.foreach_set("test_unsigned_int", ssize_t_array_as_n)
        result_sequence = self.get_sequence("test_unsigned_int")
        expected_sequence = ssize_t_array_as_n
        self.assertSequenceEqual(result_sequence, expected_sequence)
        # `memoryview` cannot be subclassed, so it is not possible to check if it was accessed as a buffer or
        # sequence.

        # void* ("P") format arrays are not supported by NumPy but are supported by ctypes.
        void_p_array_type = ctypes.c_void_p * num_items
        # ctypes's conversion to void* only accepts Python int, but the NumPy array returned by get_nth_arange()
        # contains NumPy scalar types, so convert the NumPy array to a list containing Python ints.
        void_p_array = void_p_array_type(*self.get_nth_arange(1, num_items).tolist())
        uint_buffers = [void_p_array]
        np_uint_types = (np.ubyte, np.ushort, np.uintc, np.uint, np.ulonglong)
        for i, uint_type in enumerate(np_uint_types, start=2):
            # Ensure each buffer has different contents from the previous buffer.
            uint_buffers.append(self.get_nth_arange(i, num_items, dtype=uint_type))

        compatible_itemsizes = set()
        for buffer in uint_buffers:
            buffer_with_sequence_check = as_sequence_check_buffer(buffer)
            self.collection.foreach_set("test_unsigned_int", buffer_with_sequence_check)
            result_sequence = self.get_sequence("test_unsigned_int")
            self.assertSequenceEqual(result_sequence, buffer)
            if not buffer_with_sequence_check.used_as_sequence:
                compatible_itemsizes.add(memoryview(buffer).itemsize)
        self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                       "compatible itemsizes were '%s'" % compatible_itemsizes)

    @unittest.expectedFailure  # See #92621
    def test_sequence_get_enum(self):
        self.do_test_sequence_get("test_enum")

    @unittest.expectedFailure  # See #92621
    def test_sequence_set_enum(self):
        self.do_test_sequence_set("test_enum", [1] * self.get_num_items("test_enum"))

    @unittest.expectedFailure  # See #92621
    def test_buffer_get_enum(self):
        expected_sequence = self.get_sequence("test_enum")
        num_items = self.get_num_items("test_enum")

        int_types = (np.byte, np.short, np.intc, np.int_, np.longlong)
        compatible_itemsizes = set()
        for int_type in int_types:
            buffer = np.zeros(num_items, dtype=int_type)
            buffer_with_sequence_check = as_sequence_check_buffer(buffer)

            self.collection.foreach_get("test_enum", buffer_with_sequence_check)

            self.assertSequenceEqual(buffer, expected_sequence)
            if not buffer_with_sequence_check.used_as_sequence:
                compatible_itemsizes.add(buffer.itemsize)
        self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                       "compatible itemsizes were '%s'" % compatible_itemsizes)

        # ssize_t ("n") format is not supported by NumPy.
        ssize_t_array = (ctypes.c_ssize_t * num_items)()
        # ctypes exposed its ssize_t arrays as buffers with one of the other integer types with the same size instead of
        # the "n" format. A memoryview using the "n" format can be created by casting.
        ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")
        # memoryview objects are not writable as a sequence (only readable), so they are only expected to work when they
        # are the correct itemsize to be considered a compatible buffer.
        if ssize_t_array_as_n.itemsize in compatible_itemsizes:
            self.collection.foreach_get("test_enum", ssize_t_array_as_n)

            self.assertSequenceEqual(ssize_t_array_as_n, expected_sequence)
        else:
            with self.assertRaises(TypeError):
                self.collection.foreach_get("test_enum", ssize_t_array_as_n)

    @unittest.expectedFailure  # See #92621
    def test_buffer_set_enum(self):
        num_items = self.get_num_items("test_enum")

        # ssize_t ("n") format is not supported by NumPy.
        ssize_t_array = (ctypes.c_ssize_t * num_items)(*self.get_nth_arange(0, num_items))
        # ctypes exposed its ssize_t arrays as buffers with one of the other integer types with the same size instead of
        # the "n" format. A memoryview using the "n" format can be created by casting.
        ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")

        self.collection.foreach_set("test_enum", ssize_t_array_as_n)

        result_sequence = self.get_sequence("test_enum")
        expected_sequence = ssize_t_array_as_n
        self.assertSequenceEqual(result_sequence, expected_sequence)
        # `memoryview` cannot be subclassed, so it is not possible to check if it was accessed as a buffer or
        # sequence.

        int_types = (np.byte, np.short, np.intc, np.int_, np.longlong)
        compatible_itemsizes = set()
        for i, int_type in enumerate(int_types, start=1):
            # Ensure each buffer has different contents from the previous buffer.
            buffer = self.get_nth_arange(i, num_items, dtype=int_type)
            buffer_with_sequence_check = as_sequence_check_buffer(buffer)

            self.collection.foreach_set("test_enum", buffer_with_sequence_check)

            result_sequence = self.get_sequence("test_enum")
            self.assertSequenceEqual(result_sequence, buffer)
            if not buffer_with_sequence_check.used_as_sequence:
                compatible_itemsizes.add(buffer.itemsize)
        self.assertEqual(len(compatible_itemsizes), 1, "There should only be one compatible itemsize, but the"
                                                       "compatible itemsizes were '%s'" % compatible_itemsizes)


if __name__ == '__main__':
    import sys
    sys.argv = [__file__] + (sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    unittest.main()
