import bpy


# Test for legacy or present Grease Pencil object
obj = bpy.context.active_object
assert obj.type == 'GREASEPENCIL'

# For the active layer, get the frame at current scene time
grease_pencil = bpy.context.active_object.data
layer = grease_pencil.layers.active_layer
if layer is None:
    print("No active layer")
else:
    current_frame = layer.get_frame_at(bpy.context.scene.frame_current)
    if current_frame is None:
        print(f"No keyframe found at frame {bpy.context.scene.frame_current}")


# Iterate over all frames in a layer
for frame in layer.frames:
    print(f"Frame: {frame.frame_number} {'is selected' if frame.select else 'is not selected'}")


# Get the drawing at a frame
drawing = current_frame.drawing

# Check the type of drawing
# -> DRAWING: This drawing is owned by the Grease Pencil data
# -> REFERENCE: This drawing is referenced from another Grease Pencil data
print(f"Type: {drawing.type}")

# Get the number of strokes and stroke points in a drawing
print(f"Number of strokes: {drawing.num_strokes}, and points: {drawing.num_points}")