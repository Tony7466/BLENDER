
/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

#pragma once

#include "overlay_next_private.hh"

namespace blender::draw::overlay {

/* Matches Vertex Format. */
struct Vertex {
  float3 pos;
  int vclass;
};

enum VertexClass {
  VCLASS_NONE = 0,
  VCLASS_LIGHT_AREA_SHAPE = 1 << 0,
  VCLASS_LIGHT_SPOT_SHAPE = 1 << 1,
  VCLASS_LIGHT_SPOT_BLEND = 1 << 2,
  VCLASS_LIGHT_SPOT_CONE = 1 << 3,
  VCLASS_LIGHT_DIST = 1 << 4,

  VCLASS_CAMERA_FRAME = 1 << 5,
  VCLASS_CAMERA_DIST = 1 << 6,
  VCLASS_CAMERA_VOLUME = 1 << 7,

  VCLASS_SCREENSPACE = 1 << 8,
  VCLASS_SCREENALIGNED = 1 << 9,

  VCLASS_EMPTY_SCALED = 1 << 10,
  VCLASS_EMPTY_AXES = 1 << 11,
  VCLASS_EMPTY_AXES_NAME = 1 << 12,
  VCLASS_EMPTY_AXES_SHADOW = 1 << 13,
  VCLASS_EMPTY_SIZE = 1 << 14,
};

static const float bone_box_verts[8][3] = {
    {1.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, -1.0f},
    {-1.0f, 0.0f, -1.0f},
    {-1.0f, 0.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, -1.0f},
    {-1.0f, 1.0f, -1.0f},
    {-1.0f, 1.0f, 1.0f},
};

static const std::array<uint, 24> bone_box_wire = {
    0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
};

/* A single ring of vertices. */
static Vector<float2> ring_vertices(const float radius, const int segments)
{
  Vector<float2> verts;
  for (int i : IndexRange(segments)) {
    float angle = (2 * M_PI * i) / segments;
    verts.append(radius * float2(math::cos(angle), math::sin(angle)));
  }
  return verts;
}

/* Returns lines segment geometry forming 3 circles, one on each axis. */
static Vector<Vertex> sphere_axes_circles(const float radius,
                                          const VertexClass vclass,
                                          const int segments)
{
  Vector<float2> ring = ring_vertices(radius, segments);

  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  for (int axis : IndexRange(3)) {
    for (int i : IndexRange(segments)) {
      for (int j : IndexRange(2)) {
        float2 cv = ring[(i + j) % segments];
        if (axis == 0) {
          verts.append({{cv[0], cv[1], 0.0f}, vclass});
        }
        else if (axis == 1) {
          verts.append({{cv[0], 0.0f, cv[1]}, vclass});
        }
        else {
          verts.append({{0.0f, cv[0], cv[1]}, vclass});
        }
      }
    }
  }
  return verts;
}

static const Vector<Vertex> &quad_wire_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  verts = {{{-1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{-1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{-1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{+1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{+1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{+1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{+1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{-1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED}};
  return verts;
}

static const Vector<Vertex> &plain_axes_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  verts = {{{0.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{0.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{-1.0f, 0.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{+1.0f, 0.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{0.0f, 0.0f, -1.0f}, VCLASS_EMPTY_SCALED},
           {{0.0f, 0.0f, +1.0f}, VCLASS_EMPTY_SCALED}};
  return verts;
}

static const Vector<Vertex> &single_arrow_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  float3 p[3] = {{0.0f, 0.0f, 1.0f}, {0.035f, 0.035f, 0.75f}, {-0.035f, 0.035f, 0.75f}};
  for (int sides : IndexRange(4)) {
    if (sides % 2 == 1) {
      p[1].x = -p[1].x;
      p[2].y = -p[2].y;
    }
    else {
      p[1].y = -p[1].y;
      p[2].x = -p[2].x;
    }
    for (int i = 0, a = 1; i < 2; i++, a++) {
      verts.append({p[i], VCLASS_EMPTY_SCALED});
      verts.append({p[a], VCLASS_EMPTY_SCALED});
    }
  }
  verts.append({{0.0f, 0.0f, 0.0f}, VCLASS_EMPTY_SCALED});
  verts.append({{0.0f, 0.0f, 0.75f}, VCLASS_EMPTY_SCALED});
  return verts;
}

static const Vector<Vertex> &cube_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  for (auto index : bone_box_wire) {
    float x = bone_box_verts[index][0];
    float y = bone_box_verts[index][1] * 2.0f - 1.0f;
    float z = bone_box_verts[index][2];
    verts.append({{x, y, z}, VCLASS_EMPTY_SCALED});
  }
  return verts;
}

static const Vector<Vertex> &circle_verts()
{
  const int resolution = 64;
  Vector<float2> ring = ring_vertices(1.0f, resolution);

  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  for (int a : IndexRange(resolution + 1)) {
    float2 cv = ring[a % resolution];
    verts.append({{cv.x, 0.0f, cv.y}, VCLASS_EMPTY_SCALED});
  }
  return verts;
}

static const Vector<Vertex> &empty_sphere_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  verts = sphere_axes_circles(1.0f, VCLASS_EMPTY_SCALED, 32);
  return verts;
}

static const Vector<Vertex> &empty_cone_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const int resolution = 8;
  Vector<float2> ring = ring_vertices(1.0f, resolution);

  for (int i : IndexRange(resolution)) {
    float2 cv = ring[i % resolution];
    /* Cone sides. */
    verts.append({{cv[0], 0.0f, cv[1]}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, 2.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    /* Base ring. */
    for (int j : IndexRange(2)) {
      float2 cv = ring[(i + j) % resolution];
      verts.append({{cv[0], 0.0f, cv[1]}, VCLASS_EMPTY_SCALED});
    }
  }
  return verts;
}

static const Vector<Vertex> &arrows_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  float2 x_axis_name_scale = {0.0215f, 0.025f};
  Vector<float2> x_axis_name = {
      float2(0.9f, 1.0f) * x_axis_name_scale,
      float2(-1.0f, -1.0f) * x_axis_name_scale,
      float2(-0.9f, 1.0f) * x_axis_name_scale,
      float2(1.0f, -1.0f) * x_axis_name_scale,
  };

  float2 y_axis_name_scale = {0.0175f, 0.025f};
  Vector<float2> y_axis_name = {
      float2(-1.0f, 1.0f) * y_axis_name_scale,
      float2(0.0f, -0.1f) * y_axis_name_scale,
      float2(1.0f, 1.0f) * y_axis_name_scale,
      float2(0.0f, -0.1f) * y_axis_name_scale,
      float2(0.0f, -0.1f) * y_axis_name_scale,
      float2(0.0f, -1.0f) * y_axis_name_scale,
  };

  float2 z_axis_name_scale = {0.02f, 0.025f};
  Vector<float2> z_axis_name = {
      float2(-0.95f, 1.0f) * z_axis_name_scale,
      float2(0.95f, 1.0f) * z_axis_name_scale,
      float2(0.95f, 1.0f) * z_axis_name_scale,
      float2(0.95f, 0.90f) * z_axis_name_scale,
      float2(0.95f, 0.90f) * z_axis_name_scale,
      float2(-1.0f, -0.90f) * z_axis_name_scale,
      float2(-1.0f, -0.90f) * z_axis_name_scale,
      float2(-1.0f, -1.0f) * z_axis_name_scale,
      float2(-1.0f, -1.0f) * z_axis_name_scale,
      float2(1.0f, -1.0f) * z_axis_name_scale,
  };

  float2 axis_marker_scale = {0.007f, 0.007f};
  Vector<float2> axis_marker = {
#if 0 /* square */
        float2(-1.0f, 1.0f) * axis_marker_scale,
        float2(1.0f, 1.0f) * axis_marker_scale,
        float2(1.0f, 1.0f) * axis_marker_scale,
        float2(1.0f, -1.0f) * axis_marker_scale,
        float2(1.0f, -1.0f) * axis_marker_scale,
        float2(-1.0f, -1.0f) * axis_marker_scale,
        float2(-1.0f, -1.0f) * axis_marker_scale,
        float2(-1.0f, 1.0f) * axis_marker_scale,
#else /* diamond */
    float2(-1.0f, 0.0f) * axis_marker_scale,
    float2(0.0f, 1.0f) * axis_marker_scale,
    float2(0.0f, 1.0f) * axis_marker_scale,
    float2(1.0f, 0.0f) * axis_marker_scale,
    float2(1.0f, 0.0f) * axis_marker_scale,
    float2(0.0f, -1.0f) * axis_marker_scale,
    float2(0.0f, -1.0f) * axis_marker_scale,
    float2(-1.0f, 0.0f) * axis_marker_scale,
#endif
  };

  for (int axis : IndexRange(3)) {
    /* Vertex layout is XY screen position and axis in Z.
     * Fractional part of Z is a positive offset at axis unit position. */
    int flag = VCLASS_EMPTY_AXES | VCLASS_SCREENALIGNED;
    /* Center to axis line. */
    verts.append({{0.0f, 0.0f, 0.0f}, 0});
    verts.append({{0.0f, 0.0f, float(axis)}, flag});
    /* Axis end marker. */
    const int marker_fill_layer = 6;
    for (int j = 1; j < marker_fill_layer + 1; j++) {
      for (float2 axis_marker_vert : axis_marker) {
        verts.append({{axis_marker_vert * ((4.0f * j) / marker_fill_layer), float(axis)}, flag});
      }
    }
    /* Axis name. */
    Vector<float2> *axis_names[3] = {&x_axis_name, &y_axis_name, &z_axis_name};
    for (float2 axis_name_vert : *(axis_names[axis])) {
      int flag = VCLASS_EMPTY_AXES | VCLASS_EMPTY_AXES_NAME | VCLASS_SCREENALIGNED;
      verts.append({{axis_name_vert * 4.0f, axis + 0.25f}, flag});
    }
  }
  return verts;
}

static const Vector<Vertex> &metaball_wire_circle_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const int resolution = 64;
  const float radius = 1.0f;
  Vector<float2> ring = ring_vertices(radius, resolution);

  for (int i : IndexRange(resolution + 1)) {
    float2 cv = ring[i % resolution];
    verts.append({{cv[0], cv[1], 0.0f}, VCLASS_SCREENALIGNED});
  }
  return verts;
}

static const Vector<Vertex> &speaker_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const int segments = 16;

  for (int i : IndexRange(3)) {
    float r = (i == 0 ? 0.5f : 0.25f);
    float z = 0.25f * i - 0.125f;

    verts.append({float3(r, 0.0f, z), VCLASS_NONE});

    for (float2 v : ring_vertices(r, segments)) {
      verts.append({float3(v, z), VCLASS_NONE});
      verts.append({float3(v, z), VCLASS_NONE});
    }

    verts.append({float3(r, 0.0f, z), VCLASS_NONE});
  }

  for (int i : IndexRange(4)) {
    float x = (((i + 1) % 2) * (i - 1)) * 0.5f;
    float y = ((i % 2) * (i - 2)) * 0.5f;

    for (int i2 : IndexRange(3)) {
      if (i2 == 1) {
        x *= 0.5f;
        y *= 0.5f;
      }
      float z = 0.25f * i2 - 0.125f;

      verts.append({float3(x, y, z), VCLASS_NONE});
      if (i2 == 1) {
        verts.append({float3(x, y, z), VCLASS_NONE});
      }
    }
  }
  return verts;
}

static float light_distance_z_get(char axis, const bool start)
{
  switch (axis) {
    case 'x': /* - X */
      return start ? 0.4f : 0.3f;
    case 'X': /* + X */
      return start ? 0.6f : 0.7f;
    case 'y': /* - Y */
      return start ? 1.4f : 1.3f;
    case 'Y': /* + Y */
      return start ? 1.6f : 1.7f;
    case 'z': /* - Z */
      return start ? 2.4f : 2.3f;
    case 'Z': /* + Z */
      return start ? 2.6f : 2.7f;
  }
  return 0.0;
}

const float sin_pi_3 = 0.86602540378f;
const float cos_pi_3 = 0.5f;

const int diamond_segments = 4;

static const Vector<Vertex> &probe_cube_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float r = 14.0f;
  int flag = VCLASS_SCREENSPACE;
  /* Icon */
  const float2 p[7] = {
      {0.0f, 1.0f},
      {sin_pi_3, cos_pi_3},
      {sin_pi_3, -cos_pi_3},
      {0.0f, -1.0f},
      {-sin_pi_3, -cos_pi_3},
      {-sin_pi_3, cos_pi_3},
      {0.0f, 0.0f},
  };
  for (int i : IndexRange(6)) {
    float2 t1 = p[i];
    float2 t2 = p[(i + 1) % 6];
    verts.append({{t1 * r, 0.0f}, flag});
    verts.append({{t2 * r, 0.0f}, flag});
  }
  for (int i : {1, 6, 5, 6, 3, 6}) {
    verts.append({{p[i].x * r, p[i].y * r, 0.0f}, flag});
  }
  /* Direction Lines */
  flag = VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE;
  for (char axis : "zZyYxX") {
    float zsta = light_distance_z_get(axis, true);
    float zend = light_distance_z_get(axis, false);
    verts.append({{0.0f, 0.0f, zsta}, flag});
    verts.append({{0.0f, 0.0f, zend}, flag});

    const float r = 1.2f;

    verts.append({{r, 0.0f, zsta}, flag});
    for (float2 v : ring_vertices(r, diamond_segments)) {
      verts.append({{v, zsta}, flag});
      verts.append({{v, zsta}, flag});
    }
    verts.append({{r, 0.0f, zsta}, flag});

    verts.append({{r, 0.0f, zend}, flag});
    for (float2 v : ring_vertices(r, diamond_segments)) {
      verts.append({{v, zend}, flag});
      verts.append({{v, zend}, flag});
    }
    verts.append({{r, 0.0f, zend}, flag});
  }
  return verts;
}

static const Vector<Vertex> &probe_grid_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float r = 14.0f;
  int flag = VCLASS_SCREENSPACE;
  /* Icon */
  const float2 p[7] = {
      {0.0f, 1.0f},
      {sin_pi_3, cos_pi_3},
      {sin_pi_3, -cos_pi_3},
      {0.0f, -1.0f},
      {-sin_pi_3, -cos_pi_3},
      {-sin_pi_3, cos_pi_3},
      {0.0f, 0.0f},
  };
  for (int i : IndexRange(6)) {
    float2 t1 = p[i];
    float2 t2 = p[(i + 1) % 6];
    float2 tr;
    verts.append({{t1 * r, 0.0f}, flag});
    verts.append({{t2 * r, 0.0f}, flag});
    /* Internal wires. */
    for (int j = 1; j < 2; j++) {
      tr = p[(i / 2) * 2 + 1] * (-0.5f * j);
      t1 = p[i] + tr;
      t2 = p[(i + 1) % 6] + tr;
      verts.append({{t1 * r, 0.0f}, flag});
      verts.append({{t2 * r, 0.0f}, flag});
    }
  }
  for (int i : {1, 6, 5, 6, 3, 6}) {
    verts.append({{p[i].x * r, p[i].y * r, 0.0f}, flag});
  }
  /* Direction Lines */
  flag = VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE;
  for (char axis : "zZyYxX") {
    float zsta = light_distance_z_get(axis, true);
    float zend = light_distance_z_get(axis, false);
    verts.append({{0.0f, 0.0f, zsta}, flag});
    verts.append({{0.0f, 0.0f, zend}, flag});

    const float r = 1.2f;

    verts.append({{r, 0.0f, zsta}, flag});
    for (float2 v : ring_vertices(r, diamond_segments)) {
      verts.append({{v, zsta}, flag});
      verts.append({{v, zsta}, flag});
    }
    verts.append({{r, 0.0f, zsta}, flag});

    verts.append({{r, 0.0f, zend}, flag});
    for (float2 v : ring_vertices(r, diamond_segments)) {
      verts.append({{v, zend}, flag});
      verts.append({{v, zend}, flag});
    }
    verts.append({{r, 0.0f, zend}, flag});
  }
  return verts;
}

static const Vector<Vertex> &probe_planar_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float r = 20.0f;
  /* Icon */
  const float2 p[4] = {
      {0.0f, 0.5f},
      {sin_pi_3, 0.0f},
      {0.0f, -0.5f},
      {-sin_pi_3, 0.0f},
  };
  for (int i = 0; i < 4; i++) {
    for (int a = 0; a < 2; a++) {
      verts.append({{p[(i + a) % 4] * r, 0.0}, VCLASS_SCREENSPACE});
    }
  }
  return verts;
}

ShapeCache::ShapeCache()
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&format, "vclass", GPU_COMP_I32, 1, GPU_FETCH_INT);
  }

  auto batch_ptr = [&](const Vector<Vertex> &verts,
                       GPUPrimType prim = GPU_PRIM_LINES) -> BatchPtr {
    GPUVertBuf *vbo = GPU_vertbuf_create_with_format(&format);
    GPU_vertbuf_data_alloc(vbo, verts.size());
    memcpy(GPU_vertbuf_get_data(vbo), verts.data(), verts.size() * sizeof(Vertex));

    return BatchPtr(GPU_batch_create_ex(prim, vbo, nullptr, GPU_BATCH_OWNS_VBO));
  };

  quad_wire = batch_ptr(quad_wire_verts());
  plain_axes = batch_ptr(plain_axes_verts());
  single_arrow = batch_ptr(single_arrow_verts());
  cube = batch_ptr(cube_verts());
  circle = batch_ptr(circle_verts(), GPU_PRIM_LINE_STRIP);
  empty_sphere = batch_ptr(empty_sphere_verts());
  empty_cone = batch_ptr(empty_cone_verts());
  arrows = batch_ptr(arrows_verts());
  metaball_wire_circle = batch_ptr(metaball_wire_circle_verts(), GPU_PRIM_LINE_STRIP);
  speaker = batch_ptr(speaker_verts());
  probe_cube = batch_ptr(probe_cube_verts());
  probe_grid = batch_ptr(probe_grid_verts());
  probe_planar = batch_ptr(probe_planar_verts());
}

}  // namespace blender::draw::overlay
