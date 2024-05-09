import bpy
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
print('Number of strokes and points:', drawing.num_strokes, drawing.num_points)


# Make all selected strokes cyclic
for i, stroke_selected in enumerate(drawing.stroke_selections):
    if stroke_selected.value:
        drawing.cyclic[i].value = True
drawing.tag_redraw()  # Update the Grease Pencil object in the viewport


# Add random noise to the position of all points of the first stroke
point_range = drawing.strokes[0]
for point_i in range(point_range.start, point_range.end + 1):
    drawing.positions[point_i].vector.x += (random.random() - 0.5) * 0.2
    drawing.positions[point_i].vector.z += (random.random() - 0.5) * 0.2


# Change the radius of all selected points using numpy
selected_points = np.ndarray(drawing.num_points, dtype=bool)
drawing.point_selections.foreach_get('value', selected_points)
point_radii = np.ndarray(drawing.num_points, dtype=float)
drawing.radii.foreach_get('value', point_radii)
point_radii[selected_points] = 0.1
drawing.radii.foreach_set('value', point_radii)
drawing.tag_redraw()


# Get the positions of all selected points
selected_points = np.ndarray(drawing.num_points, dtype=bool)
drawing.point_selections.foreach_get('value', selected_points)
point_positions = np.ndarray((drawing.num_points, 3), dtype=float)
drawing.positions.foreach_get('vector', point_positions.ravel())
print('Positions of selected points:', point_positions[selected_points])
print('Point indices of selected points:', np.arange(drawing.num_points)[selected_points])


#
# Add strokes to drawing
new_strokes_num = 3

# For the example: create random strokes
num_points_per_stroke = []
new_strokes_pos = []
new_strokes_radius = []
for _ in range(new_strokes_num):
    new_points_pos = []
    new_points_radius = []
    x = -3.0
    z = -1.0 + random.randint(-100, 100) / 150
    radius = 0.015
    for i in range(random.randint(10, 20)):
        new_points_pos.append((x, 0, z))
        new_points_radius.append(radius + random.randint(0, 100) / 2000)
        x += random.randint(0, 100) / 200
        z += random.randint(-100, 100) / 2000
    new_strokes_pos.append(new_points_pos)
    new_strokes_radius.append(new_points_radius)
    num_points_per_stroke.append(len(new_points_radius))

# Add new (empty) strokes to drawing
new_stroke_start_index = drawing.num_strokes
drawing.add_strokes(num_points_per_stroke)  # Argument is of the form: [num_points_stroke_1, num_points_stroke_2, ...]

# For the new strokes, set the point position and radius
point_positions = np.ndarray((drawing.num_points, 3), dtype=float)
drawing.positions.foreach_get('vector', point_positions.ravel())
point_radii = np.ndarray(drawing.num_points, dtype=float)
drawing.radii.foreach_get('value', point_radii)
for i in range(new_strokes_num):
    # Get the start and end point index of the new stroke in the drawing
    stroke_index = new_stroke_start_index + i
    point_range = drawing.strokes[stroke_index]

    # Copy the point positions and radii to the numpy arrays
    point_positions[point_range.start:point_range.end + 1] = new_strokes_pos[i]
    point_radii[point_range.start:point_range.end + 1] = new_strokes_radius[i]

    # Ensure that the new stroke is deselected
    drawing.stroke_selections[stroke_index].value = False

    # As an example: set material and hardness of the stroke
    drawing.material_indices[stroke_index].value = 0
    drawing.hardnesses[stroke_index].value = 0.95

# Update the point positions and radii in the drawing
drawing.positions.foreach_set('vector', point_positions.ravel())
drawing.radii.foreach_set('value', point_radii)
drawing.tag_redraw()


#
# All builtin attributes

# Stroke attributes
stroke_index = 0
drawing.cyclic[stroke_index].value
drawing.start_caps[stroke_index].value
drawing.end_caps[stroke_index].value
drawing.hardnesses[stroke_index].value
drawing.material_indices[stroke_index].value
drawing.stroke_selections[stroke_index].value
drawing.fill_colors[stroke_index].color  # .color[channel 0-3]
drawing.aspect_ratios[stroke_index].value
drawing.init_times[stroke_index].value

# Point attributes
point_index = 0
drawing.positions[point_index].vector
drawing.radii[point_index].value
drawing.opacities[point_index].value
drawing.vertex_colors[point_index].color  # .color[channel 0-3]
drawing.point_selections[point_index].value
drawing.rotations[point_index].value
drawing.delta_times[point_index].value
