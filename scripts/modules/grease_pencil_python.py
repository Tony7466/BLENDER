# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

def get_attribute_value_at_index(attribute, type, index):
    if type in {'FLOAT', 'INT', 'STRING', 'BOOLEAN', 'INT8', 'INT32_2D', 'QUATERNION', 'FLOAT4X4'}:
        return attribute.data[index].value
    elif type == 'FLOAT_VECTOR':
        return attribute.data[index].vector
    elif type in {'FLOAT_COLOR', 'BYTE_COLOR'}:
        return attribute.data[index].color


def set_attribute_value_at_index(attribute, type, index, value):
    if type in {'FLOAT', 'INT', 'STRING', 'BOOLEAN', 'INT8', 'INT32_2D', 'QUATERNION', 'FLOAT4X4'}:
        attribute.data[index].value = value
    elif type == 'FLOAT_VECTOR':
        attribute.data[index].vector = value
    elif type in {'FLOAT_COLOR', 'BYTE_COLOR'}:
        attribute.data[index].color = value


class GreasePencilStrokePoint:
    """
    A helper class to get access to stroke point data.
    """

    def __init__(self, drawing, point_index):
        self.drawing = drawing
        self.point_index = point_index

    def get_attribute(self, name, type):
        if attribute := self.drawing.attributes.get(name):
            return get_attribute_value_at_index(attribute, type, self.point_index)

    def set_attribute(self, name, type, value):
        if attribute := self.drawing.attributes.get(name, self.drawing.attributes.new(name, type, 'POINT')):
            set_attribute_value_at_index(attribute, type, self.point_index, value)

    @property
    def position(self):
        return self.get_attribute('position', 'FLOAT_VECTOR')

    @position.setter
    def position(self, value):
        self.set_attribute('position', 'FLOAT_VECTOR', value)

    @property
    def radius(self):
        return self.get_attribute('radius', 'FLOAT')

    @radius.setter
    def radius(self, value):
        self.set_attribute('radius', 'FLOAT', value)

    @property
    def opacity(self):
        return self.get_attribute('opacity', 'FLOAT')

    @opacity.setter
    def opacity(self, value):
        self.set_attribute('opacity', 'FLOAT', value)

    @property
    def selection(self):
        return self.get_attribute('.selection', 'BOOLEAN')

    @selection.setter
    def selection(self, value):
        self.set_attribute('.selection', 'BOOLEAN', value)

    @property
    def vertex_color(self):
        return self.get_attribute('vertex_color', 'FLOAT_COLOR')

    @vertex_color.setter
    def vertex_color(self, value):
        self.set_attribute('vertex_color', 'FLOAT_COLOR', value)

    @property
    def rotation(self):
        return self.get_attribute('rotation', 'FLOAT')

    @rotation.setter
    def rotation(self, value):
        self.set_attribute('rotation', 'FLOAT', value)

    @property
    def delta_time(self):
        return self.get_attribute('delta_time', 'FLOAT')

    @delta_time.setter
    def delta_time(self, value):
        self.set_attribute('delta_time', 'FLOAT', value)


class GreasePencilStroke:
    """
    A helper class to get access to stroke data.
    """

    def __init__(self, drawing, curve_index, points_start_index, points_end_index):
        self.drawing = drawing
        self.curve_index = curve_index
        self.points_start_index = points_start_index
        self.points_end_index = points_end_index

    def get_attribute(self, name, type):
        if attribute := self.drawing.attributes.get(name):
            return get_attribute_value_at_index(attribute, type, self.curve_index)

    def set_attribute(self, name, type, value):
        if attribute := self.drawing.attributes.get(name, self.drawing.attributes.new(name, type, 'CURVE')):
            set_attribute_value_at_index(attribute, type, self.curve_index, value)

    @property
    def points(self):
        return [
            GreasePencilStrokePoint(
                self.drawing,
                point_i) for point_i in range(
                self.points_start_index,
                self.points_end_index)]

    @property
    def cyclic(self):
        return self.get_attribute('cyclic', 'BOOLEAN')

    @cyclic.setter
    def cyclic(self, value):
        self.set_attribute('cyclic', 'BOOLEAN', value)

    @property
    def material_index(self):
        return self.get_attribute('material_index', 'INT')

    @material_index.setter
    def material_index(self, value):
        self.set_attribute('material_index', 'INT', value)

    @property
    def selection(self):
        return self.get_attribute('.selection', 'BOOLEAN')

    @selection.setter
    def selection(self, value):
        self.set_attribute('.selection', 'BOOLEAN', value)

    @property
    def softness(self):
        return self.get_attribute('softness', 'FLOAT')

    @softness.setter
    def softness(self, value):
        self.set_attribute('softness', 'FLOAT', value)

    @property
    def start_cap(self):
        return self.get_attribute('start_cap', 'INT8')

    @start_cap.setter
    def start_cap(self, value):
        self.set_attribute('start_cap', 'INT8', value)

    @property
    def end_cap(self):
        return self.get_attribute('end_cap', 'INT8')

    @end_cap.setter
    def end_cap(self, value):
        self.set_attribute('end_cap', 'INT8', value)

    @property
    def curve_type(self):
        return self.get_attribute('curve_type', 'INT8')

    @curve_type.setter
    def curve_type(self, value):
        self.set_attribute('curve_type', 'INT8', value)

    @property
    def aspect_ratio(self):
        return self.get_attribute('aspect_ratio', 'FLOAT')

    @aspect_ratio.setter
    def aspect_ratio(self, value):
        self.set_attribute('aspect_ratio', 'FLOAT', value)

    @property
    def fill_opacity(self):
        return self.get_attribute('fill_opacity', 'FLOAT')

    @fill_opacity.setter
    def fill_opacity(self, value):
        self.set_attribute('fill_opacity', 'FLOAT', value)

    @property
    def fill_color(self):
        return self.get_attribute('fill_color', 'FLOAT_COLOR')

    @fill_color.setter
    def fill_color(self, value):
        self.set_attribute('fill_color', 'FLOAT_COLOR', value)

    @property
    def time_start(self):
        return self.get_attribute('init_time', 'FLOAT')

    @time_start.setter
    def time_start(self, value):
        self.set_attribute('init_time', 'FLOAT', value)
