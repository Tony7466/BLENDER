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
from itertools import chain

id_inst = bpy.context.scene
id_type = bpy.types.Scene

# If an enum property is set to an integer value for which no enum item exists then an empty string is returned and
# looking up the integer value of the enum property would fail. In such cases, return this value instead.
MISSING_ENUM_VALUE = -1


# -----------------------------------------------------------------------------
# Utility Classes and Functions

class SequenceCheckNdarray(np.ndarray):
    """
    np.ndarray subclass that tracks whether is has been accessed as a sequence.

    __getitem__ and __setitem__ are called by PySequence_GetItem and PySequence_SetItem (and also PyObject_GetItem
    and PyObject_SetItem).

    Once Python 3.12 is in use, this class could be replaced with a wrapper around any buffer that redirects all python
    calls other than __getitem__ and __setitem__ to the wrapped buffer and defines a __buffer__ method that returns the
    wrapped buffer.
    """

    def __array_finalize__(self, obj):
        """
        Initialize the new instance, like what would normally be done in __init__ or __new__.

        NumPy calls __array_finalize__ in each of its different methods of creating arrays because __init__ and __new__
        are not called by some methods.
        """
        self.used_as_sequence = False

    def __getitem__(self, key):
        """
        Called by PySequence_GetItem (and also PyObject_GetItem)
        """
        self.used_as_sequence = True
        return super().__getitem__(key)

    def __setitem__(self, key, value):
        """
        Called by PySequence_SetItem (and also PyObject_SetItem)
        """
        self.used_as_sequence = True
        super().__setitem__(key, value)


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


def dtype_byteorder_swap_standard_size(dtype):
    """
    Return a copy of the input NumPy dtype, but with the byteorder swapped and in standard size.

    Used to help test buffer formats with "<" or ">" prefixes that do not match native byteorder.
    """
    dtype = np.dtype(dtype)  # Allow passing in anything that can be coerced into a dtype.
    # assert dtype.byteorder != '|', "Byteorder is not applicable to %s" % dtype
    return dtype.newbyteorder('S')


def dtype_explicit_endian_standard_size(dtype):
    """
    Return a copy of the input NumPy dtype, but with native byteorder replaced with explicit little-endian/big-endian
    byteorder that matches the native byteorder and uses standard sizes.
    Normally when a "<" or ">" dtype is created that matches native byteorder, NumPy will replace it with implicit
    native byteorder instead, but this function forces "<" or ">" to be used.

    Used to help test buffer formats with "<" or ">" prefixes that match native byteorder.
    """
    return dtype_byteorder_swap_standard_size(dtype_byteorder_swap_standard_size(dtype))


def prop_as_sequence(collection, prop_rna):
    """
    Helper to get a flat sequence of a specific property through iteration instead of foreach_get.
    """
    prop_name = prop_rna.identifier
    if getattr(prop_rna, "is_array", False):
        gen = (getattr(array_prop, prop_name) for array_prop in collection)
        for dim_size in prop_rna.array_dimensions:
            if dim_size == 0:
                break
            gen = chain.from_iterable(gen)
        return list(gen)
    elif prop_rna.type == 'ENUM':
        # Enum properties are a special case where foreach_get/set access the enum as an index, but accessing the
        # enum property directly is done using the string identifiers of the enum's items.
        index_lookup = {item.identifier: item.value for item in prop_rna.enum_items}
        return [index_lookup.get(getattr(group, prop_name), MISSING_ENUM_VALUE) for group in collection]
    else:
        return [getattr(group, prop_name) for group in collection]


# -----------------------------------------------------------------------------
# Tests

class BaseTestForeachGetSet(unittest.TestCase):
    SEQUENCE_TYPES = ('LIST', 'IMPLICIT_NATIVE_ARRAY', 'EXPLICIT_NATIVE_ARRAY', 'NON_NATIVE_ARRAY')

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

    @staticmethod
    def sequence_generator(collection, prop_rna, is_set, dtype_modifier=None):
        item_length = prop_rna.array_length if getattr(prop_rna, "is_array", False) else 1
        sequence_length = len(collection) * item_length

        def next_sequence(next_fill_value, dtype):
            if dtype == list:
                return [next_fill_value] * sequence_length
            else:
                return np.full(sequence_length, next_fill_value,
                               dtype=dtype_modifier(dtype) if dtype_modifier else dtype)

        if collection:
            initial_value = getattr(collection[0], prop_rna.identifier)
            if getattr(prop_rna, "is_array", False):
                for dim_size in prop_rna.array_dimensions:
                    if dim_size == 0:
                        break
                    initial_value = initial_value[0]
            elif prop_rna.type == 'ENUM':
                # Get the integer value of the string identifier
                if initial_value in prop_rna.enum_items:
                    initial_value = prop_rna.enum_items[initial_value].value
                else:
                    initial_value = MISSING_ENUM_VALUE
        else:
            # The collection is empty, so the initial value is irrelevant.
            initial_value = 0

        # To correctly handle all properties with the same test setup ('BOOLEAN' and 'ENUM' being the most restrictive),
        # we can only use `0` and `1` as fill values.
        if is_set:
            last_fill_value = initial_value
            # When setting values, each time we set, we need to set to different values from before so that we can check
            # that setting the values worked.

            def generate(dtype):
                nonlocal last_fill_value
                if last_fill_value == 0:
                    last_fill_value = 1
                else:
                    last_fill_value = 0
                return next_sequence(last_fill_value, dtype)

            return generate
        else:
            # When getting values, the initial values in the sequence must differ from the default values of the
            # properties, so that we can check that getting the values worked.
            fill_value = 1 if initial_value == 0 else 0
            return lambda dtype: next_sequence(fill_value, dtype)

    @staticmethod
    def make_sequences_buffer_float(next_sequence):
        return [
            next_sequence(np.half),        # "e", 16-bit-precision floating-point (no corresponding C type)
            next_sequence(np.single),      # "f", float
            next_sequence(np.double),      # "d", double
            # Note: NumPy can only expose `long double` as a buffer when in native byteorder, size and alignment.
            next_sequence(np.longdouble),  # "g", long double
        ]

    @staticmethod
    def make_sequences_buffer_int_extra_native_mv(next_sequence):
        """
        Extra integer type buffers that cannot be represented by NumPy, but can be constructed with ctypes and cast to
        a memoryview with the expected native buffer format.
        """
        sequences = []

        # ssize_t ("n") format buffers are not supported by NumPy or ctypes.
        ssize_t_initializer = next_sequence(list)
        ssize_t_array = (ctypes.c_ssize_t * len(ssize_t_initializer))(*ssize_t_initializer)
        # ctypes exposed its ssize_t arrays as buffers with format set to one of the other integer types with
        # the same itemsize instead of the "n" format. A memoryview using the "n" format can be created by
        # casting.
        ssize_t_array_as_n = memoryview(ssize_t_array).cast("b").cast("n")
        assert len(ssize_t_array_as_n) == len(ssize_t_array)
        sequences.append(ssize_t_array_as_n)

        # size_t ("N") format buffers are not supported by NumPy or ctypes.
        size_t_initializer = next_sequence(list)
        size_t_array = (ctypes.c_size_t * len(size_t_initializer))(*size_t_initializer)
        # ctypes exposed its size_t arrays as buffers with format set to one of the other integer types with the
        # same itemsize instead of the "N" format. A memoryview using the "N" format can be created by casting.
        size_t_array_as_n = memoryview(size_t_array).cast("b").cast("N")
        assert len(size_t_array_as_n) == len(size_t_array)
        sequences.append(size_t_array_as_n)

        # void* ("P") format buffers are not supported by NumPy but are supported by ctypes.
        void_p_initializer = next_sequence(list)
        void_p_array = (ctypes.c_void_p * len(void_p_initializer))(*void_p_initializer)
        # ctypes adds a native byteorder, standard size and no alignment prefix to the buffer format, e.g. "<P", so
        # create a memoryview and cast away the prefix.
        # ctypes.c_void_p arrays also return `None` when accessed as a sequence and the value at the accessed index is
        # zero, which would break the tests whenever the array contains a zero. A "P" memoryview just returns zero.
        size_t_array_as_p = memoryview(void_p_array).cast("b").cast("P")
        assert len(size_t_array_as_p) == len(void_p_array)
        sequences.append(size_t_array_as_p)

        return sequences

    @staticmethod
    def make_sequences_buffer_int(next_sequence, ndarray_only):
        sequences = []

        if not ndarray_only:
            sequences.extend(BaseTestForeachGetSet.make_sequences_buffer_int_extra_native_mv(next_sequence))

        return sequences + [
            next_sequence(np.byte),       # "b", signed char
            next_sequence(np.short),      # "h", short
            next_sequence(np.intc),       # "i", int
            next_sequence(np.int_),       # "l", long
            next_sequence(np.longlong),   # "q", long long
            next_sequence(np.ubyte),      # "B", unsigned char
            next_sequence(np.ushort),     # "H", unsigned short
            next_sequence(np.uintc),      # "I", unsigned int
            next_sequence(np.uint),       # "L", unsigned long
            next_sequence(np.ulonglong),  # "Q", unsigned long long
        ]

    @staticmethod
    def make_sequences_buffer_bool(next_sequence, ndarray_only):
        # BOOLEAN properties that have no raw array access (e.g. they use a bitmask), are considered compatible with
        # bool buffers.
        sequences = [next_sequence(np.bool_)]  # "?", bool
        # However, BOOLEAN properties that do have raw array access are only considered compatible with buffers that
        # match the property's raw type, which is likely to be a single byte numeric/char type.
        sequences.extend(BaseTestForeachGetSet.make_sequences_buffer_int(next_sequence, ndarray_only))
        return sequences

    def make_subtest_sequences(self, collection, prop_rna, is_set, is_arrays, dtype_modifier):
        generate_sequence = self.sequence_generator(collection, prop_rna, is_set, dtype_modifier)

        ndarray_only = dtype_modifier is not None

        if is_arrays:
            match prop_rna.type:
                case 'BOOLEAN':
                    return self.make_sequences_buffer_bool(generate_sequence, ndarray_only)
                case 'INT' | 'ENUM':
                    return self.make_sequences_buffer_int(generate_sequence, ndarray_only)
                case 'FLOAT':
                    return self.make_sequences_buffer_float(generate_sequence)
                case _:
                    raise TypeError("Unsupported property type '%s'" % prop_rna.type)
        else:
            match prop_rna.type:
                case 'BOOLEAN':
                    py_type = bool
                case 'INT' | 'ENUM':
                    py_type = int
                case 'FLOAT':
                    py_type = float
                case _:
                    raise TypeError("Unsupported property type '%s'" % prop_rna.type)
            sequences = [list(map(py_type, generate_sequence(list)))]
            if is_set:
                sequences.append(tuple(map(py_type, generate_sequence(list))))
            return sequences

    def do_get_subtest(self, collection, prop_rna, seq):
        prop_name = prop_rna.identifier
        expected_sequence = prop_as_sequence(collection, prop_rna)

        accessed_as_buffer = False
        if isinstance(seq, np.ndarray):
            # View the sequence as a subtype that allows us to check whether the sequence was accessed as a sequence or
            # as a buffer.
            ndarray_with_sequence_check = seq.view(SequenceCheckNdarray)
            collection.foreach_get(prop_name, ndarray_with_sequence_check)
            # If the sequence wasn't used at all, this can result in a false positive, but other assertions should fail
            # in that case.
            accessed_as_buffer = not ndarray_with_sequence_check.used_as_sequence
        elif isinstance(seq, memoryview):
            # memoryview objects are not writable as a sequence, only readable, so if they are not accessed as a buffer,
            # writing to them as a sequence will fail (the memoryview type only implements PyObject_SetItem and not
            # PySequence_SetItem). `memoryview` cannot be subclassed, but this may be fixable once Python 3.12 is in use
            # because it makes the buffer protocol accessible to Python.
            try:
                collection.foreach_get(prop_name, seq)
            except TypeError:
                # Most likely the memoryview was accessed as a sequence.
                return False
            else:
                accessed_as_buffer = True
        else:
            collection.foreach_get(prop_name, seq)

        if prop_rna.type == 'FLOAT':
            self.assertSequenceAlmostEqual(seq, expected_sequence)
        else:
            self.assertSequenceEqual(seq, expected_sequence)

        return accessed_as_buffer

    def do_set_subtest(self, collection, prop_rna, seq):
        prop_name = prop_rna.identifier
        accessed_as_buffer = False
        if isinstance(seq, np.ndarray):
            # View the sequence as a subtype that allows us to check whether the sequence was accessed as a sequence or
            # as a buffer.
            ndarray_with_sequence_check = seq.view(SequenceCheckNdarray)
            collection.foreach_set(prop_name, ndarray_with_sequence_check)
            # If the sequence wasn't used at all, this can result in a false positive, but assertions should fail in
            # that case.
            accessed_as_buffer = not ndarray_with_sequence_check.used_as_sequence
        elif 0:  # isinstance(seq, memoryview):
            # There is currently no way to tell if a memoryview was accessed as a sequence or as a buffer. Once Python
            # 3.12 is in use, this may become possible because Python 3.12 makes the buffer protocol accessible to
            # Python.
            collection.foreach_set(prop_name, seq)
        else:
            collection.foreach_set(prop_name, seq)

        result_sequence = prop_as_sequence(collection, prop_rna)
        expected_sequence = seq

        if prop_rna.type == 'FLOAT':
            self.assertSequenceAlmostEqual(result_sequence, expected_sequence)
        else:
            self.assertSequenceEqual(result_sequence, expected_sequence)

        return accessed_as_buffer

    @staticmethod
    def get_sequence_description(seq):
        """
        Helper to get a description of an input sequence, to be printed when a subtest fails.
        """
        if isinstance(seq, (list, tuple)):
            return "%s ('%s')" % (type(seq).__name__, type(seq[0]).__name__ if seq else "None")
        elif isinstance(seq, np.ndarray):
            element_type = seq.dtype
            # Numpy cannot represent non-native long double ("g") arrays as a buffer. Getting `.data` will fail in that
            # case.
            buffer_format = seq.data.format if element_type.char != "g" else None
            return "%s ('%s')" % (element_type, buffer_format)
        elif isinstance(seq, ctypes.Array):
            element_type_name = seq._type_.__name__
            buffer_format = memoryview(seq).format
            return "%s ('%s')" % (element_type_name, buffer_format)
        elif isinstance(seq, memoryview):
            return "memoryview ('%s')" % seq.format
        else:
            raise TypeError("Unsupported type '%s'" % type(seq))

    def do_getset_subtest(self, collection, prop_rna, sequence_type, is_set):
        match sequence_type:
            case 'LIST':
                assert_buffer_usage = False
                is_arrays = False
                dtype_modifier = None
            case 'IMPLICIT_NATIVE_ARRAY':
                assert_buffer_usage = True
                is_arrays = True
                dtype_modifier = None
            case 'EXPLICIT_NATIVE_ARRAY':
                assert_buffer_usage = True
                is_arrays = True
                dtype_modifier = dtype_explicit_endian_standard_size
            case 'NON_NATIVE_ARRAY':
                assert_buffer_usage = False
                is_arrays = True
                dtype_modifier = dtype_byteorder_swap_standard_size
            case _:
                raise RuntimeError("Unrecognised sequence type '%s'" % sequence_type)

        sequences = self.make_subtest_sequences(collection, prop_rna, is_set, is_arrays, dtype_modifier)

        subtest_func = self.do_set_subtest if is_set else self.do_get_subtest
        with self.subTest(prop_name=prop_rna.identifier, sequence_type=sequence_type):
            # Due to varying itemsizes of C types depending on the current system, and the itemsize of a
            # property not being exposed to Blender's Python API, we only check that at least one sequence
            # was considered a compatible buffer.
            at_least_one_accessed_as_buffer = False
            for seq in sequences:
                with self.subTest(subsequence=self.get_sequence_description(seq)):
                    accessed_as_buffer = subtest_func(collection, prop_rna, seq)
                    at_least_one_accessed_as_buffer |= accessed_as_buffer
            if assert_buffer_usage:
                self.assertTrue(at_least_one_accessed_as_buffer, "at least one sequence should be accessed as a buffer")

    def check_foreach_getset(self, collection, prop_rna, is_set):
        for sequence_type in self.SEQUENCE_TYPES:
            self.do_getset_subtest(collection, prop_rna, sequence_type, is_set=is_set)

    def check_foreach_get(self, collection, prop_rna):
        self.check_foreach_getset(collection, prop_rna, is_set=False)

    def check_foreach_set(self, collection, prop_rna):
        self.check_foreach_getset(collection, prop_rna, is_set=True)


class TestPropCollectionIDPropForeachGetSet(BaseTestForeachGetSet):
    """
    Test different runtime properties on an ID property collection (`bpy_prop_collection_idprop`).

    These properties are expected to not have raw array access because they are runtime properties.
    """
    def setUp(self):
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

        self.property_group_class = TestPropertyGroup

        bpy.utils.register_class(TestPropertyGroup)
        id_type.test_collection = CollectionProperty(type=TestPropertyGroup)
        for _ in range(5):
            id_inst.test_collection.add()

    def tearDown(self):
        # Custom Properties save their data in ID properties by default. Deleting the Custom Property registration will
        # leave the ID properties behind, so ensure the ID properties are removed by clearing the collection before
        # deleting its registration.
        id_inst.test_collection.clear()
        del id_type.test_collection
        bpy.utils.unregister_class(self.property_group_class)

    def check_getset(self, prop_names, is_set):
        for prop_name in prop_names:
            prop_rna = self.property_group_class.bl_rna.properties[prop_name]
            self.check_foreach_getset(id_inst.test_collection, prop_rna, is_set)

    def check_get(self, *prop_names):
        self.check_getset(prop_names, is_set=False)

    def check_set(self, *prop_names):
        self.check_getset(prop_names, is_set=True)

    # Test methods

    def test_foreach_get_bool(self):
        self.check_get("test_bool", "test_bool_vector")

    def test_foreach_set_bool(self):
        self.check_set("test_bool", "test_bool_vector")

    def test_foreach_get_float(self):
        self.check_get("test_float", "test_float_vector")

    def test_foreach_set_float(self):
        self.check_set("test_float", "test_float_vector")

    def test_foreach_get_int(self):
        self.check_get("test_int", "test_int_vector", "test_unsigned_int")

    def test_foreach_set_int(self):
        self.check_set("test_int", "test_int_vector", "test_unsigned_int")

    def test_foreach_get_enum(self):
        self.check_get("test_enum")

    def test_foreach_set_enum(self):
        self.check_set("test_enum")


class TestPropCollectionForeachGetSetMeshAttributes(BaseTestForeachGetSet):
    """
    Test different mesh attribute collections.

    The properties of these collections' elements commonly have raw array access.
    """
    def setUp(self):
        self.mesh = bpy.data.meshes.new("")
        self.mesh.vertices.add(5)
        mesh_attributes = self.mesh.attributes
        # Attribute data types and the name of the property to use with foreach_get/foreach_set.
        self.attribute_type_to_prop_name = {
            'FLOAT': "value",
            'INT': "value",
            'FLOAT_VECTOR': "vector",
            'FLOAT_COLOR': "color",
            'BYTE_COLOR': "color",
            # 'STRING': "value",  # Does not support foreach_get/foreach_set
            'BOOLEAN': "value",
            'FLOAT2': "vector",
            'INT8': "value",
            'INT32_2D': "value",
            'QUATERNION': "value",
        }
        # Attribute pointers are not stable when adding additional attributes, so get the names first. See #107500.
        int_attribute_names = []
        float_attribute_names = []
        bool_attribute_names = []
        for data_type, prop_name in self.attribute_type_to_prop_name.items():
            attr = mesh_attributes.new("test_%s" % data_type.lower(), data_type, 'POINT')
            collection_rna = attr.bl_rna.properties["data"]
            collection_element_rna = collection_rna.fixed_type
            prop_rna = collection_element_rna.properties[prop_name]
            match prop_rna.type:
                case 'INT':
                    int_attribute_names.append(attr.name)
                case 'FLOAT':
                    float_attribute_names.append(attr.name)
                case 'BOOLEAN':
                    bool_attribute_names.append(attr.name)
        # Now that all the attributes have been added, the pointers should be stable.
        self.int_attributes = [mesh_attributes[name] for name in int_attribute_names]
        self.float_attributes = [mesh_attributes[name] for name in float_attribute_names]
        self.bool_attributes = [mesh_attributes[name] for name in bool_attribute_names]
        # There should always be at least one of each property type.
        assert self.int_attributes
        assert self.float_attributes
        assert self.bool_attributes

    def tearDown(self):
        bpy.data.meshes.remove(self.mesh)

    def check_getset(self, attributes, is_set):
        for attribute in attributes:
            prop_name = self.attribute_type_to_prop_name[attribute.data_type]
            collection = attribute.data
            collection_rna = attribute.bl_rna.properties["data"]
            collection_element_rna = collection_rna.fixed_type
            prop_rna = collection_element_rna.properties[prop_name]
            self.check_foreach_getset(collection, prop_rna, is_set)

    def check_get(self, attributes):
        self.check_getset(attributes, is_set=False)

    def check_set(self, attributes):
        self.check_getset(attributes, is_set=True)

    # Test methods

    def test_foreach_get_bool(self):
        self.check_get(self.bool_attributes)

    def test_foreach_set_bool(self):
        self.check_set(self.bool_attributes)

    def test_foreach_get_float(self):
        self.check_get(self.float_attributes)

    def test_foreach_set_float(self):
        self.check_set(self.float_attributes)

    def test_foreach_get_int(self):
        self.check_get(self.int_attributes)

    def test_foreach_set_int(self):
        self.check_set(self.int_attributes)


class TestPropCollectionForeachGetSetNoRawAccess(BaseTestForeachGetSet):
    """
    Test the bpy.types.Collection.objects collection.

    The properties of this collection's elements are expected to never have raw array access because the Objects in the
    collection are not stored in contiguous memory.
    """
    def setUp(self):
        VECTOR_SIZE = 3

        DEFAULT_INT = 0  # Must be either 0 or 1 so that bool and enum tests can work properly.
        DEFAULT_BOOL = bool(DEFAULT_INT)
        DEFAULT_FLOAT = float(DEFAULT_INT)

        cls = bpy.types.Object

        cls.runtime_bool = BoolProperty(default=DEFAULT_BOOL)
        cls.runtime_bool_vector = BoolVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_BOOL] * VECTOR_SIZE)
        cls.runtime_float = FloatProperty(default=DEFAULT_FLOAT)
        cls.runtime_float_vector = FloatVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_FLOAT] * VECTOR_SIZE)
        cls.runtime_int = IntProperty(default=DEFAULT_INT)
        cls.runtime_int_vector = IntVectorProperty(size=VECTOR_SIZE, default=[DEFAULT_INT] * VECTOR_SIZE)
        cls.runtime_unsigned_int = IntProperty(subtype='UNSIGNED', default=DEFAULT_INT)
        _ENUM_ITEMS = (("0", "Item 0", ""), ("1", "Item 1", ""))
        cls.runtime_enum = EnumProperty(items=_ENUM_ITEMS, default=_ENUM_ITEMS[DEFAULT_INT][0])

        self.property_group_class = bpy.types.Object

        self.collection = bpy.data.collections.new("")
        self.objects = []
        for _ in range(5):
            obj = bpy.data.objects.new("", None)
            self.objects.append(obj)
            self.collection.objects.link(obj)

    def tearDown(self):
        bpy.data.collections.remove(self.collection)
        for obj in self.objects:
            bpy.data.objects.remove(obj)

        cls = bpy.types.Object

        del cls.runtime_bool
        del cls.runtime_bool_vector
        del cls.runtime_float
        del cls.runtime_float_vector
        del cls.runtime_int
        del cls.runtime_int_vector
        del cls.runtime_unsigned_int
        del cls.runtime_enum

    def check_getset(self, prop_names, is_set):
        for prop_name in prop_names:
            prop_rna = bpy.types.Object.bl_rna.properties[prop_name]
            self.check_foreach_getset(self.collection.objects, prop_rna, is_set)

    def check_get(self, *prop_names):
        self.check_getset(prop_names, is_set=False)

    def check_set(self, *prop_names):
        self.check_getset(prop_names, is_set=True)

    # Test methods

    def test_foreach_get_bool(self):
        self.check_get(
            "runtime_bool",
            "runtime_bool_vector",

            "show_bounds",
            "is_missing",  # read-only
        )

    def test_foreach_set_bool(self):
        self.check_set(
            "runtime_bool",
            "runtime_bool_vector",

            "show_bounds",
        )

    def test_foreach_get_float(self):
        self.check_get(
            "runtime_float",
            "runtime_float_vector",  # 1D array

            "color",  # 1D array
            "location",  # 1D array
            "matrix_world",  # 2D array
            "bound_box",  # read-only, 2D array
        )

    def test_foreach_set_float(self):
        self.check_set(
            "runtime_float",
            "runtime_float_vector",  # 1D array

            "color",  # 1D array
            "location",  # 1D array
            "matrix_world",  # 2D array
        )

    def test_foreach_get_int(self):
        self.check_get(
            "runtime_int",
            "runtime_int_vector",
            "runtime_unsigned_int",

            "active_material_index",
            "users",  # read-only
        )

    def test_foreach_set_int(self):
        self.check_set(
            "runtime_int",
            "runtime_int_vector",
            "runtime_unsigned_int",

            "active_material_index",
        )

    def test_foreach_get_enum(self):
        self.check_get(
            "runtime_enum",

            "display_bounds_type",
            "type",  # read-only
        )

    def test_foreach_set_enum(self):
        self.check_set(
            "runtime_enum",

            "display_bounds_type",
        )


class TestPropCollectionForeachGetSetNoItemPropertyPointer(BaseTestForeachGetSet):
    """
    Test the case where no item property pointer can be retrieved from the collection. This usually occurs when the
    collection's item type is defined dynamically.

    `ShapeKey.data` can be a collection of `ShapeKeyPoint`, `ShapeKeyCurvePoint` or `ShapeKeyBezierPoint`, with Curve
    instances allowing a mix of both `ShapeKeyCurvePoint` and `ShapeKeyBezierPoint` in the same collection.
    """
    def setUp(self):
        self.mixed_curve = bpy.data.curves.new("", 'CURVE')
        bezier_spline = self.mixed_curve.splines.new('BEZIER')
        bezier_spline.bezier_points.add(5)
        poly_spline = self.mixed_curve.splines.new('POLY')
        poly_spline.points.add(5)
        self.mixed_curve_object = bpy.data.objects.new("", self.mixed_curve)
        self.mixed_curve_shape_key = self.mixed_curve_object.shape_key_add()

        self.bezier_curve = bpy.data.curves.new("", 'CURVE')
        bezier_spline2 = self.bezier_curve.splines.new('BEZIER')
        bezier_spline2.bezier_points.add(5)
        bezier_spline3 = self.bezier_curve.splines.new('BEZIER')
        bezier_spline3.bezier_points.add(5)
        self.bezier_curve_object = bpy.data.objects.new("", self.bezier_curve)
        self.bezier_curve_shape_key = self.bezier_curve_object.shape_key_add()

    def tearDown(self):
        self.mixed_curve_object.shape_key_clear()
        bpy.data.objects.remove(self.mixed_curve_object)
        bpy.data.curves.remove(self.mixed_curve)

        self.bezier_curve_object.shape_key_clear()
        bpy.data.objects.remove(self.bezier_curve_object)
        bpy.data.curves.remove(self.bezier_curve)

    # Test methods

    def test_foreach_get_no_item_property_pointer_non_array(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["radius"]
        assert not prop_rna.is_array
        self.check_foreach_get(self.bezier_curve_shape_key.data, prop_rna)

    def test_foreach_set_no_item_property_pointer_non_array(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["radius"]
        assert not prop_rna.is_array
        self.check_foreach_set(self.bezier_curve_shape_key.data, prop_rna)

    def test_foreach_get_no_item_property_pointer_array(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["co"]
        assert prop_rna.is_array
        self.check_foreach_get(self.bezier_curve_shape_key.data, prop_rna)

    def test_foreach_set_no_item_property_pointer_array(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["co"]
        assert prop_rna.is_array
        self.check_foreach_set(self.bezier_curve_shape_key.data, prop_rna)

    def test_foreach_get_no_item_property_pointer_mixed_property_not_found_for_some_items(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["handle_left"]
        assert "handle_left" not in bpy.types.ShapeKeyCurvePoint.bl_rna.properties

        collection_bezier_only = self.bezier_curve_shape_key.data
        self.check_foreach_get(collection_bezier_only, prop_rna)

        collection_mixed = self.mixed_curve_shape_key.data
        sequence_get = self.sequence_generator(collection_mixed, prop_rna, is_set=False)(list)
        # Error: Property named 'handle_left' not found
        # RuntimeError: internal error setting the array
        with self.assertRaises(RuntimeError):
            collection_mixed.foreach_get(prop_rna.identifier, sequence_get)

    def test_foreach_set_no_item_property_pointer_mixed_property_not_found_for_some_items(self):
        prop_rna = bpy.types.ShapeKeyBezierPoint.bl_rna.properties["handle_left"]
        assert "handle_left" not in bpy.types.ShapeKeyCurvePoint.bl_rna.properties

        collection_bezier_only = self.bezier_curve_shape_key.data
        self.check_foreach_set(collection_bezier_only, prop_rna)

        collection_mixed = self.mixed_curve_shape_key.data
        sequence_set = self.sequence_generator(collection_mixed, prop_rna, is_set=True)(list)
        # Error: Property named 'handle_left' not found
        # RuntimeError: internal error setting the array
        with self.assertRaises(RuntimeError):
            collection_mixed.foreach_set(prop_rna.identifier, sequence_set)


if __name__ == '__main__':
    import sys
    sys.argv = [__file__] + (sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else [])
    unittest.main()
