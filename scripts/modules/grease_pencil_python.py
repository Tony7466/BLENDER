# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

# Helper class to get and set attributes at an index for a domain
class AttributeGetterSetter:
    def __init__(self, attributes, index, domain):
        self._attributes = attributes
        self._index = index
        self._domain = domain

    def _get_attribute(self, name, type):
        if attribute := self._attributes.get(name):
            if type in {'FLOAT', 'INT', 'STRING', 'BOOLEAN', 'INT8', 'INT32_2D', 'QUATERNION', 'FLOAT4X4'}:
                return attribute.data[self._index].value
            elif type == 'FLOAT_VECTOR':
                return attribute.data[self._index].vector
            elif type in {'FLOAT_COLOR', 'BYTE_COLOR'}:
                return attribute.data[self._index].color
            else:
                raise Exception(f"Unkown type {type}")
        else:
            raise Exception(f"Attribute {name} not found")

    def _set_attribute(self, name, type, value):
        if attribute := self._attributes.get(name, self._attributes.new(name, type, self._domain)):
            if type in {'FLOAT', 'INT', 'STRING', 'BOOLEAN', 'INT8', 'INT32_2D', 'QUATERNION', 'FLOAT4X4'}:
                attribute.data[self._index].value = value
            elif type == 'FLOAT_VECTOR':
                attribute.data[self._index].vector = value
            elif type in {'FLOAT_COLOR', 'BYTE_COLOR'}:
                attribute.data[self._index].color = value
        else:
            raise Exception(f"Could not create attribute {name} of type {type}")


def def_prop_for_attribute(attr_name, type, doc):
    """
    Creates a property that can read and write an attribute.
    """

    def fget(self):
        # Define getter callback for property
        return self._get_attribute(attr_name, type)

    def fset(self, value):
        # Define setter callback for property
        self._set_attribute(attr_name, type, value)
    prop = property(fget=fget, fset=fset, doc=doc)
    return prop


def DefAttributeGetterSetters(attributes_list):
    """
    A class decorator that reads a list of attribute infos and creates properties on the class with getters and setters.
    """
    def wrapper(cls):
        for prop_name, attr_name, type, doc in attributes_list:
            prop = def_prop_for_attribute(attr_name, type, doc)
            setattr(cls, prop_name, prop)
        return cls
    return wrapper


# Define the list of attributes that should be exposed as read/write properties on the class.
@DefAttributeGetterSetters([
    # Property Name, Attribute Name, Type, Docstring
    ('position', 'position', 'FLOAT_VECTOR', "The position of the point (in local space)."),
    ('radius', 'radius', 'FLOAT', "The radius of the point."),
    ('opacity', 'opacity', 'FLOAT', "The opacity of the point."),
    ('select', '.selection', 'BOOLEAN', "The selection state for this point."),
    ('vertex_color', 'vertex_color', 'FLOAT_COLOR',
     "The color for this point. The alpha value is used as a mix factor with the base color of the stroke."),
    ('rotation', 'rotation', 'FLOAT', "The rotation for this point. Used to rotate textures."),
    ('delta_time', 'delta_time', 'FLOAT', "The time delta in seconds since the start of the stroke."),
])
class GreasePencilStrokePoint(AttributeGetterSetter):
    """
    A helper class to get access to stroke point data.
    """

    def __init__(self, drawing, point_index):
        super().__init__(drawing.attributes, point_index, 'POINT')


# Define the list of attributes that should be exposed as read/write properties on the class.
@DefAttributeGetterSetters([
    # Property Name, Attribute Name, Type, Docstring
    ('cyclic', 'cyclic', 'BOOLEAN', "The closed state for this stroke."),
    ('material_index', 'material_index', 'INT', "The index of the material for this stroke."),
    ('select', '.selection', 'BOOLEAN', "The selection state for this stroke."),
    ('softness', 'softness', 'FLOAT', "Used by the renderer to generate a soft gradient from the stroke center line to the edges."),
    ('start_cap', 'start_cap', 'INT8', "The type of start cap of this stroke."),
    ('end_cap', 'end_cap', 'INT8', "The type of end cap of this stroke."),
    ('curve_type', 'curve_type', 'INT8', "The type of curve."),
    ('aspect_ratio', 'aspect_ratio', 'FLOAT', "The aspect ratio (x/y) used for textures. "),
    ('fill_opacity', 'fill_opacity', 'FLOAT', "The opacity of the fill."),
    ('fill_color', 'fill_color', 'FLOAT_COLOR', "The color of the fill."),
    ('time_start', 'init_time', 'FLOAT', "A time value for when the stroke was created."),
])
class GreasePencilStroke(AttributeGetterSetter):
    """
    A helper class to get access to stroke data.
    """

    def __init__(self, drawing, curve_index, points_start_index, points_end_index):
        super().__init__(drawing.attributes, curve_index, 'CURVE')
        self._drawing = drawing
        self._curve_index = curve_index
        self._points_start_index = points_start_index
        self._points_end_index = points_end_index

    @property
    def points(self):
        """
        Return a list of points in the stroke.
        """
        return [
            GreasePencilStrokePoint(
                self._drawing,
                point) for point in range(
                self._points_start_index,
                self._points_end_index)]

    def add_points(self, count):
        """
        Add new points at the end of the stroke and returns the new points as a list.
        """
        previous_end = self._points_end_index
        new_size = self._points_end_index - self._points_start_index + count
        self._drawing.resize_curves(sizes=[new_size], indices=[self._curve_index])
        self._points_end_index = self._points_start_index + new_size
        return [
            GreasePencilStrokePoint(
                self._drawing,
                point) for point in range(
                previous_end,
                self._points_end_index)]

    def remove_points(self, count):
        """
        Remove points at the end of the stroke.
        """
        new_size = self._points_end_index - self._points_start_index - count
        # A stroke need to have at least one point.
        if new_size < 1:
            new_size = 1
        self._drawing.resize_curves(sizes=[new_size], indices=[self._curve_index])
        self._points_end_index = self._points_start_index + new_size
