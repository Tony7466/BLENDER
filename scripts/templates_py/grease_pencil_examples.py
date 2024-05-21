import bpy
import mathutils
import random
import numpy as np


# Test for legacy or present Grease Pencil object
obj = bpy.context.active_object
if obj.type == 'GPENCIL':
    print('Legacy GPv2 object')
elif obj.type == 'GREASEPENCIL':
    print('Present GPv3 object')


# For the active layer, get the frame at current scene time
grease_pencil = bpy.context.active_object.data
layer = grease_pencil.layers.active_layer
if layer is None:
    print('No active layer')
else:
    current_frame = layer.get_frame_at(bpy.context.scene.frame_current)
    if current_frame is None:
        print('No keyframe found at frame', bpy.context.scene.frame_current)


# Iterate over all frames in a layer
for frame in layer.frames:
    print('Frame:', frame.frame_number, frame.select)


# Get the drawing at a frame
drawing = current_frame.drawing


# Get the number of strokes and stroke points in a drawing
print('Number of strokes:', drawing.num_strokes, 'and points:', drawing.num_points)


# Get attribute values
stroke_material = drawing.material_indices[0]  # Get the material of first stroke in drawing
materials = drawing.material_indices[1:5]  # Using slice: get the materials of the second to fifth stroke

# Set attribute values
drawing.material_indices[0] = 1  # Set the material index of the first stroke to 1
drawing.material_indices[1:5] = [1, 1, 1, 1]
drawing.tag_redraw()  # Update the Grease Pencil object in the viewport

# Make all selected strokes cyclic
for i, stroke_selected in enumerate(drawing.stroke_selections):
    if stroke_selected:
        drawing.cyclic[i] = True
drawing.tag_redraw()  # Update the Grease Pencil object in the viewport


# Add random noise to the position of all points of the first stroke
point_range = drawing.strokes[0]
for point_i in range(point_range.start, point_range.stop):
    drawing.positions[point_i].x += (random.random() - 0.5) * 0.2
    drawing.positions[point_i].z += (random.random() - 0.5) * 0.2


# Fill an attribute with a value
drawing.radii.fill(0.1)  # For all points in the drawing, set the radius to 0.1
drawing.radii.fill(0.1, [0, 1, 2, 3])  # Fill with indices: set the radius of the first four points to 0.1
drawing.tag_redraw()


# Assign colors and positions
drawing.positions[0] = mathutils.Vector((1.0, 0.0, 2.0))  # Assign a Vector or...
drawing.positions[0] = [1.0, 0.0, 2.0]  # Assign a list or...
drawing.positions[0] = (1.0, 0.0, 2.0)  # Assign a tuple
drawing.vertex_colors[0] = [0.8, 0.2, 0.0, 1.0]  # Assign a list or...
drawing.vertex_colors[0] = (0.8, 0.2, 0.0, 1.0)  # Assign a tuple


# Change the radius of all selected points using numpy
selected_points = np.ndarray(drawing.num_points, dtype=bool)
drawing.point_selections.foreach_get(selected_points)  # Get the selected points in the drawing
selected_point_indices = np.nonzero(selected_points)[0]  # Get the indices of the selected points
drawing.radii.fill(0.1, selected_point_indices)  # Change the radius of the selected points
drawing.tag_redraw()


# Get the positions of all selected points using numpy
selected_points = np.ndarray(drawing.num_points, dtype=bool)
drawing.point_selections.foreach_get(selected_points)  # Get the selected points in the drawing
point_positions = np.ndarray((drawing.num_points, 3), dtype=np.float32)
drawing.positions.foreach_get(point_positions.ravel())  # Get the position of all points in the drawing
print('Positions of selected points:', point_positions[selected_points])
print('Point indices of selected points:', np.nonzero(selected_points)[0])


# Change the softness of strokes with a material index between 1 and 3
stroke_materials = np.ndarray(drawing.num_strokes, dtype=np.int32)
drawing.material_indices.foreach_get(stroke_materials)
# Get the stroke indices of strokes with a material index between 1 and 3
indices = np.nonzero((stroke_materials >= 1) & (stroke_materials <= 3))[0]
drawing.softnesses.fill(0.05, indices)  # Change the softness of these strokes
drawing.tag_redraw()


#
# Add strokes to drawing
#
new_strokes_pos = [
    # Stroke 1
    [(-3.0, 0, -0.44), (-2.98, 0, -0.41), (-2.75, 0, -0.37), (-2.49, 0, -0.38), (-2.32, 0, -0.38), (-1.96, 0, -0.37),
     (-1.81, 0, -0.38), (-1.65, 0, -0.34), (-1.23, 0, -0.34), (-0.88, 0, -0.34), (-0.85, 0, -0.38)],
    # Stroke 2
    [(-3.0, 0, -1.58), (-2.94, 0, -1.61), (-2.48, 0, -1.59), (-2.30, 0, -1.54), (-2.11, 0, -1.55), (-1.86, 0, -1.55), (-1.60, 0, -1.59), (-1.38, 0, -1.63),
     (-1.16, 0, -1.61), (-0.74, 0, -1.59), (-0.25, 0, -1.58), (-0.02, 0, -1.62), (0.13, 0, -1.58), (0.31, 0, -1.61), (0.36, 0, -1.59), (0.69, 0, -1.56)],
    # Stroke 3
    [(-3.0, 0, -0.97), (-2.76, 0, -0.94), (-2.5, 0, -0.90), (-2.06, 0, -0.89), (-1.79, 0, -0.88), (-1.45, 0, -0.89), (-0.97, 0, -0.93),
     (-0.72, 0, -0.90), (-0.48, 0, -0.89), (-0.43, 0, -0.86), (-0.29, 0, -0.88), (-0.11, 0, -0.87), (0.08, 0, -0.91), (0.26, 0, -0.93)]
]
new_strokes_radius = [
    # Stroke 1
    [0.036, 0.045, 0.023, 0.028, 0.046, 0.031, 0.018, 0.034, 0.022, 0.022, 0.053],
    # Stroke 2
    [0.021, 0.033, 0.037, 0.050, 0.046, 0.059, 0.028, 0.05, 0.047, 0.05, 0.057, 0.034, 0.015, 0.047, 0.019, 0.031],
    # Stroke 3
    [0.057, 0.058, 0.041, 0.039, 0.032, 0.027, 0.055, 0.029, 0.028, 0.033, 0.018, 0.025, 0.046, 0.023]
]
num_points_per_stroke = [11, 16, 14]
new_strokes_num = len(new_strokes_radius)

# Add new (empty) strokes to drawing
new_stroke_start_index = drawing.num_strokes
drawing.add_strokes(num_points_per_stroke)  # Argument is of the form: [num_points_stroke_1, num_points_stroke_2, ...]

# For the new strokes, set the point position and radius
for i in range(new_strokes_num):
    # Get the start and stop point index of the new stroke in the drawing
    stroke_index = new_stroke_start_index + i
    point_range = drawing.strokes[stroke_index]

    # Assign the point positions and radii to the drawing
    drawing.positions[point_range.start:point_range.stop] = new_strokes_pos[i]
    drawing.radii[point_range.start:point_range.stop] = new_strokes_radius[i]

    # Set the opacity of the points
    drawing.opacities.fill(1.0, range(point_range.start, point_range.stop))

    # Ensure that the new stroke is deselected
    drawing.stroke_selections[stroke_index] = False

    # As an example: set material and softness of the stroke
    drawing.material_indices[stroke_index] = 0
    drawing.softnesses[stroke_index] = 0.05

drawing.tag_redraw()  # Update the drawing in the viewport


#
# All builtin attributes
#

# Stroke attributes
stroke_index = 0
drawing.cyclic[stroke_index]  # True/False
drawing.start_caps[stroke_index]  # 0 (round) / 1 (flat)
drawing.end_caps[stroke_index]  # 0 (round) / 1 (flat)
drawing.hardnesses[stroke_index]  # Float in range 0.0-1.0
drawing.material_indices[stroke_index]  # Int
drawing.stroke_selections[stroke_index]  # True/False
drawing.fill_colors[stroke_index]  # [r, g, b, a], can also be assigned with (r, g, b, a)
drawing.aspect_ratios[stroke_index]  # Float in range 0.0-1.0
drawing.init_times[stroke_index]  # Float

# Point attributes
point_index = 0
drawing.positions[point_index]  # Vector(x, y, z), can also be assigned with (x, y, z) or [x, y, z]
drawing.radii[point_index]  # Float
drawing.opacities[point_index]  # Float in range 0.0-1.0
drawing.vertex_colors[point_index]  # [r, g, b, a], can also be assigned with (r, g, b, a)
drawing.point_selections[point_index]  # True/False
drawing.rotations[point_index]  # Float
drawing.delta_times[point_index]  # Float
