# SPDX-FileCopyrightText: 2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later

import bpy
import gpu
from gpu_extras.batch import batch_for_shader
from math import cos, sin, pi
from .node_functions import (
    abs_node_location,
    dpi_fac,
)


def draw_line(x1, y1, x2, y2, size, color=(1.0, 1.0, 1.0, 0.7)):
    shader = gpu.shader.from_builtin('POLYLINE_SMOOTH_COLOR')
    shader.uniform_float("viewportSize", gpu.state.viewport_get()[2:])
    shader.uniform_float("lineWidth", size * bpy.context.preferences.system.pixel_size)

    vertices = ((x1, y1), (x2, y2))
    vertex_colors = ((color[0] + (1.0 - color[0]) / 4,
                      color[1] + (1.0 - color[1]) / 4,
                      color[2] + (1.0 - color[2]) / 4,
                      color[3] + (1.0 - color[3]) / 4),
                     color)

    batch = batch_for_shader(shader, 'LINE_STRIP', {"pos": vertices, "color": vertex_colors})
    batch.draw(shader)


def draw_circle_2d_filled(mx, my, radius, color=(1.0, 1.0, 1.0, 0.7)):
    radius = radius * bpy.context.preferences.system.pixel_size
    sides = 12
    vertices = [(radius * cos(i * 2 * pi / sides) + mx,
                 radius * sin(i * 2 * pi / sides) + my)
                for i in range(sides + 1)]

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    shader.uniform_float("color", color)
    batch = batch_for_shader(shader, 'TRI_FAN', {"pos": vertices})
    batch.draw(shader)


def draw_rounded_node_border(node, radius=8, color=(1.0, 1.0, 1.0, 0.7)):
    area_width = bpy.context.area.width
    sides = 16
    radius *= bpy.context.preferences.system.pixel_size

    nlocx, nlocy = abs_node_location(node)

    nlocx = (nlocx + 1) * dpi_fac()
    nlocy = (nlocy + 1) * dpi_fac()
    ndimx = node.dimensions.x
    ndimy = node.dimensions.y

    if node.hide:
        nlocx += -1
        nlocy += 5
    if node.type == 'REROUTE':
        # nlocx += 1
        nlocy -= 1
        ndimx = 0
        ndimy = 0
        radius += 6

    shader = gpu.shader.from_builtin('UNIFORM_COLOR')
    shader.uniform_float("color", color)

    # Top left corner.
    mx, my = bpy.context.region.view2d.view_to_region(nlocx, nlocy, clip=False)
    vertices = [(mx, my)]
    for i in range(sides + 1):
        if (4 <= i <= 8):
            if mx < area_width:
                cosine = radius * cos(i * 2 * pi / sides) + mx
                sine = radius * sin(i * 2 * pi / sides) + my
                vertices.append((cosine, sine))

    batch = batch_for_shader(shader, 'TRI_FAN', {"pos": vertices})
    batch.draw(shader)

    # Top right corner.
    mx, my = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy, clip=False)
    vertices = [(mx, my)]
    for i in range(sides + 1):
        if (0 <= i <= 4):
            if mx < area_width:
                cosine = radius * cos(i * 2 * pi / sides) + mx
                sine = radius * sin(i * 2 * pi / sides) + my
                vertices.append((cosine, sine))

    batch = batch_for_shader(shader, 'TRI_FAN', {"pos": vertices})
    batch.draw(shader)

    # Bottom left corner.
    mx, my = bpy.context.region.view2d.view_to_region(nlocx, nlocy - ndimy, clip=False)
    vertices = [(mx, my)]
    for i in range(sides + 1):
        if (8 <= i <= 12):
            if mx < area_width:
                cosine = radius * cos(i * 2 * pi / sides) + mx
                sine = radius * sin(i * 2 * pi / sides) + my
                vertices.append((cosine, sine))

    batch = batch_for_shader(shader, 'TRI_FAN', {"pos": vertices})
    batch.draw(shader)

    # Bottom right corner.
    mx, my = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy - ndimy, clip=False)
    vertices = [(mx, my)]
    for i in range(sides + 1):
        if (12 <= i <= 16):
            if mx < area_width:
                cosine = radius * cos(i * 2 * pi / sides) + mx
                sine = radius * sin(i * 2 * pi / sides) + my
                vertices.append((cosine, sine))

    batch = batch_for_shader(shader, 'TRI_FAN', {"pos": vertices})
    batch.draw(shader)

    # Prepare drawing all edges in one batch.
    vertices = []
    indices = []
    id_last = 0

    # Left edge.
    m1x, m1y = bpy.context.region.view2d.view_to_region(nlocx, nlocy, clip=False)
    m2x, m2y = bpy.context.region.view2d.view_to_region(nlocx, nlocy - ndimy, clip=False)
    if m1x < area_width and m2x < area_width:
        vertices.extend([(m2x - radius, m2y), (m2x, m2y),
                         (m1x, m1y), (m1x - radius, m1y)])
        indices.extend([(id_last, id_last + 1, id_last + 3),
                        (id_last + 3, id_last + 1, id_last + 2)])
        id_last += 4

    # Top edge.
    m1x, m1y = bpy.context.region.view2d.view_to_region(nlocx, nlocy, clip=False)
    m2x, m2y = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy, clip=False)
    m1x = min(m1x, area_width)
    m2x = min(m2x, area_width)
    vertices.extend([(m1x, m1y), (m2x, m1y),
                     (m2x, m1y + radius), (m1x, m1y + radius)])
    indices.extend([(id_last, id_last + 1, id_last + 3),
                    (id_last + 3, id_last + 1, id_last + 2)])
    id_last += 4

    # Right edge.
    m1x, m1y = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy, clip=False)
    m2x, m2y = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy - ndimy, clip=False)
    if m1x < area_width and m2x < area_width:
        vertices.extend([(m1x, m2y), (m1x + radius, m2y),
                         (m1x + radius, m1y), (m1x, m1y)])
        indices.extend([(id_last, id_last + 1, id_last + 3),
                        (id_last + 3, id_last + 1, id_last + 2)])
        id_last += 4

    # Bottom edge.
    m1x, m1y = bpy.context.region.view2d.view_to_region(nlocx, nlocy - ndimy, clip=False)
    m2x, m2y = bpy.context.region.view2d.view_to_region(nlocx + ndimx, nlocy - ndimy, clip=False)
    m1x = min(m1x, area_width)
    m2x = min(m2x, area_width)
    vertices.extend([(m1x, m2y), (m2x, m2y),
                     (m2x, m1y - radius), (m1x, m1y - radius)])
    indices.extend([(id_last, id_last + 1, id_last + 3),
                    (id_last + 3, id_last + 1, id_last + 2)])

    # Draw all edges in one batch.
    if len(vertices) != 0:
        batch = batch_for_shader(shader, 'TRIS', {"pos": vertices}, indices=indices)
        batch.draw(shader)


def draw_callback_node_outline(context, mouse_path, mode, lazy_connect_props):
    if not mouse_path:
        return
    else:
        gpu.state.blend_set('ALPHA')

        nodes = context.space_data.edit_tree.nodes

        if mode == "LINK":
            col_outer = (1.0, 0.2, 0.2, 0.4)
            col_inner = (0.0, 0.0, 0.0, 0.5)
            col_circle_inner = (0.3, 0.05, 0.05, 1.0)
        elif mode == "LINKMENU":
            col_outer = (0.4, 0.6, 1.0, 0.4)
            col_inner = (0.0, 0.0, 0.0, 0.5)
            col_circle_inner = (0.08, 0.15, .3, 1.0)
        elif mode == "MIX":
            col_outer = (0.2, 1.0, 0.2, 0.4)
            col_inner = (0.0, 0.0, 0.0, 0.5)
            col_circle_inner = (0.05, 0.3, 0.05, 1.0)

        m1x = mouse_path[0][0]
        m1y = mouse_path[0][1]
        m2x = mouse_path[-1][0]
        m2y = mouse_path[-1][1]

        node1 = nodes[lazy_connect_props.from_node_name]
        node2 = nodes[lazy_connect_props.to_node_name]

        if node1 == node2:
            col_outer = (0.4, 0.4, 0.4, 0.4)
            col_inner = (0.0, 0.0, 0.0, 0.5)
            col_circle_inner = (0.2, 0.2, 0.2, 1.0)

        draw_rounded_node_border(node1, radius=6, color=col_outer)  # outline
        draw_rounded_node_border(node1, radius=5, color=col_inner)  # inner
        draw_rounded_node_border(node2, radius=6, color=col_outer)  # outline
        draw_rounded_node_border(node2, radius=5, color=col_inner)  # inner

        draw_line(m1x, m1y, m2x, m2y, 5, col_outer)  # line outline
        draw_line(m1x, m1y, m2x, m2y, 2, col_inner)  # line inner

        # Circle outline.
        draw_circle_2d_filled(m1x, m1y, 7, col_outer)
        draw_circle_2d_filled(m2x, m2y, 7, col_outer)

        # Circle inner.
        draw_circle_2d_filled(m1x, m1y, 5, col_circle_inner)
        draw_circle_2d_filled(m2x, m2y, 5, col_circle_inner)

        gpu.state.blend_set('NONE')
