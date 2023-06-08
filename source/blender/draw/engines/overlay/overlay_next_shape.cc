/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup overlay
 */

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

#define DIAMOND_NSEGMENTS 4
#define INNER_NSEGMENTS 8
#define OUTER_NSEGMENTS 10
#define CIRCLE_NSEGMENTS 32

static const float3 bone_box_verts[8] = {
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

static const uint3 bone_box_solid_tris[12] = {
    {0, 2, 1}, /* bottom */
    {0, 3, 2},

    {0, 1, 5}, /* sides */
    {0, 5, 4},

    {1, 2, 6},
    {1, 6, 5},

    {2, 3, 7},
    {2, 7, 6},

    {3, 0, 4},
    {3, 4, 7},

    {4, 5, 6}, /* top */
    {4, 6, 7},
};

/**
 * Store indices of generated verts from bone_box_solid_tris to define adjacency infos.
 * See bone_octahedral_solid_tris for more infos.
 */
static const uint4 bone_box_wire_lines_adjacency[12] = {
    {4, 2, 0, 11},
    {0, 1, 2, 8},
    {2, 4, 1, 14},
    {1, 0, 4, 20}, /* bottom */
    {0, 8, 11, 14},
    {2, 14, 8, 20},
    {1, 20, 14, 11},
    {4, 11, 20, 8}, /* top */
    {20, 0, 11, 2},
    {11, 2, 8, 1},
    {8, 1, 14, 4},
    {14, 4, 20, 0}, /* sides */
};

/* aligned with bone_box_solid_tris */
static const float3 bone_box_solid_normals[12] = {
    {0.0f, -1.0f, 0.0f},
    {0.0f, -1.0f, 0.0f},

    {1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},

    {0.0f, 0.0f, -1.0f},
    {0.0f, 0.0f, -1.0f},

    {-1.0f, 0.0f, 0.0f},
    {-1.0f, 0.0f, 0.0f},

    {0.0f, 0.0f, 1.0f},
    {0.0f, 0.0f, 1.0f},

    {0.0f, 1.0f, 0.0f},
    {0.0f, 1.0f, 0.0f},
};

static void append_circle_verts(
    Vector<Vertex> &verts, int segments, float radius, float z, int flag)
{
  for (int a : IndexRange(segments)) {
    for (int b : IndexRange(2)) {
      float angle = (2.0f * M_PI * (a + b)) / segments;
      float s = sinf(angle) * radius;
      float c = cosf(angle) * radius;
      verts.append({{s, c, z}, flag});
    }
  }
}

static void append_circle_dashed_verts(
    Vector<Vertex> &verts, int segments, float radius, float z, int flag)
{
  for (int a : IndexRange(segments)) {
    for (int b : IndexRange(2)) {
      float angle = (2.0f * M_PI * (a * 2 + b)) / (segments * 2);
      float s = sinf(angle) * radius;
      float c = cosf(angle) * radius;
      verts.append({{s, c, z}, flag});
    }
  }
}

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

static const Vector<Vertex> &empty_cube_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  for (auto index : bone_box_wire) {
    float3 vert = bone_box_verts[index];
    vert.y = vert.y * 2.0f - 1.0f;
    verts.append({vert, VCLASS_EMPTY_SCALED});
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
    verts.append({{cv.x, 0.0f, cv.y}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, 2.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    /* Base ring. */
    for (int j : IndexRange(2)) {
      float2 cv = ring[(i + j) % resolution];
      verts.append({{cv.x, 0.0f, cv.y}, VCLASS_EMPTY_SCALED});
    }
  }
  return verts;
}

static const Vector<Vertex> &empty_cylinder_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }
  const int segments = 12;

  int flag = VCLASS_EMPTY_SCALED;
  float2 p[segments];
  for (int i : IndexRange(segments)) {
    float angle = 2 * M_PI * ((float)i / (float)segments);
    p[i] = {cosf(angle), sinf(angle)};
  }
  for (int i : IndexRange(segments)) {
    float2 cv = p[(i) % segments];
    float2 pv = p[(i + 1) % segments];

    /* cylinder sides */
    verts.append({{cv, -1.0f}, flag});
    verts.append({{cv, 1.0f}, flag});
    /* top ring */
    verts.append({{cv, 1.0f}, flag});
    verts.append({{pv, 1.0f}, flag});
    /* bottom ring */
    verts.append({{cv, -1.0f}, flag});
    verts.append({{pv, -1.0f}, flag});
  }

  return verts;
}

static const Vector<Vertex> &empty_capsule_body_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  verts = {{{1.0f, 0.0f, 1.0f}, VCLASS_NONE},
           {{1.0f, 0.0f, 0.0f}, VCLASS_NONE},
           {{0.0f, 1.0f, 1.0f}, VCLASS_NONE},
           {{0.0f, 1.0f, 0.0f}, VCLASS_NONE},
           {{-1.0f, 0.0f, 1.0f}, VCLASS_NONE},
           {{-1.0f, 0.0f, 0.0f}, VCLASS_NONE},
           {{0.0f, -1.0f, 1.0f}, VCLASS_NONE},
           {{0.0f, -1.0f, 0.0f}, VCLASS_NONE}};

  return verts;
}

static const Vector<Vertex> &empty_capsule_cap_verts()
{
  const int segments = 24 /* Must be multiple of 2. */;
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* a single ring of vertices */
  float2 p[segments];
  for (int i : IndexRange(segments)) {
    float angle = 2 * M_PI * ((float)i / (float)segments);
    p[i] = {cosf(angle), sinf(angle)};
  }

  /* Base circle */
  for (int i : IndexRange(segments)) {
    verts.append({{p[(i) % segments], 0.0f}, VCLASS_NONE});
    verts.append({{p[(i + 1) % segments], 0.0f}, VCLASS_NONE});
  }

  for (int i : IndexRange(segments / 2)) {
    int ci = i % segments;
    int pi = (i + 1) % segments;
    /* Y half circle */
    verts.append({{p[ci].x, 0.0f, p[ci].y}, VCLASS_NONE});
    verts.append({{p[pi].x, 0.0f, p[pi].y}, VCLASS_NONE});
    /* X half circle */
    verts.append({{0.0f, p[ci].x, p[ci].y}, VCLASS_NONE});
    verts.append({{0.0f, p[pi].x, p[pi].y}, VCLASS_NONE});
  }

  return verts;
}

static const Vector<Vertex> &quad_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  verts = {{{-1.0f, 1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{1.0f, 1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{-1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED},
           {{1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED}};

  return verts;
}

static const Vector<Vertex> &grid_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  for (int i : IndexRange(8)) {
    for (int j : IndexRange(8)) {
      float2 pos0 = {(float)i / 8.0f, (float)j / 8.0f};
      float2 pos1 = {(float)(i + 1) / 8.0f, (float)j / 8.0f};
      float2 pos2 = {(float)i / 8.0f, (float)(j + 1) / 8.0f};
      float2 pos3 = {(float)(i + 1) / 8.0f, (float)(j + 1) / 8.0f};

      pos0 = float2{-1.0f, -1.0f} + pos0 * 2.0f;
      pos1 = float2{-1.0f, -1.0f} + pos1 * 2.0f;
      pos2 = float2{-1.0f, -1.0f} + pos2 * 2.0f;
      pos3 = float2{-1.0f, -1.0f} + pos3 * 2.0f;

      verts.append({{pos0, 0.0f}, VCLASS_NONE});
      verts.append({{pos1, 0.0f}, VCLASS_NONE});
      verts.append({{pos2, 0.0f}, VCLASS_NONE});

      verts.append({{pos2, 0.0f}, VCLASS_NONE});
      verts.append({{pos1, 0.0f}, VCLASS_NONE});
      verts.append({{pos3, 0.0f}, VCLASS_NONE});
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

    append_circle_verts(verts, segments, r, z, VCLASS_NONE);
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

/* -------------------------------------------------------------------- */
/** \name Lights
 * \{ */

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

static const Vector<Vertex> &groundline_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Ground Point */
  append_circle_verts(verts, DIAMOND_NSEGMENTS, 1.35f, 0.0f, 0);
  /* Ground Line */
  verts.append({{0.0, 0.0, 1.0}, 0});
  verts.append({{0.0, 0.0, 0.0}, 0});
  return verts;
}

static const Vector<Vertex> &light_icon_inner_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float r = 9.0f;

  append_circle_verts(verts, DIAMOND_NSEGMENTS, r * 0.3f, 0.0f, VCLASS_SCREENSPACE);
  append_circle_dashed_verts(verts, INNER_NSEGMENTS, r * 1.0f, 0.0f, VCLASS_SCREENSPACE);
  return verts;
}

static const Vector<Vertex> &light_icon_outer_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float r = 9.0f;

  append_circle_dashed_verts(verts, OUTER_NSEGMENTS, r * 1.33f, 0.0f, VCLASS_SCREENSPACE);
  return verts;
}

static const Vector<Vertex> &light_icon_sun_rays_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const int num_rays = 8;
  const float r = 9.0f;

  /* Sun Rays */
  for (int a : IndexRange(num_rays)) {
    float angle = (2.0f * M_PI * a) / (float)num_rays;
    float s = sinf(angle) * r;
    float c = cosf(angle) * r;
    verts.append({{s * 1.6f, c * 1.6f, 0.0f}, VCLASS_SCREENSPACE});
    verts.append({{s * 1.9f, c * 1.9f, 0.0f}, VCLASS_SCREENSPACE});
    verts.append({{s * 2.2f, c * 2.2f, 0.0f}, VCLASS_SCREENSPACE});
    verts.append({{s * 2.5f, c * 2.5f, 0.0f}, VCLASS_SCREENSPACE});
  }
  return verts;
}

static const Vector<Vertex> &light_point_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Light area */
  int flag = VCLASS_SCREENALIGNED | VCLASS_LIGHT_AREA_SHAPE;
  append_circle_verts(verts, CIRCLE_NSEGMENTS, 1.0f, 0.0f, flag);
  return verts;
}

static const Vector<Vertex> &light_sun_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Direction Line */
  verts.append({{0.0, 0.0, 0.0}, 0});
  verts.append({{0.0, 0.0, -20.0}, 0}); /* Good default. */
  return verts;
}

static const Vector<Vertex> &light_spot_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Light area */
  int flag = VCLASS_SCREENALIGNED | VCLASS_LIGHT_AREA_SHAPE;
  append_circle_verts(verts, CIRCLE_NSEGMENTS, 1.0f, 0.0f, flag);
  /* Cone cap */
  flag = VCLASS_LIGHT_SPOT_SHAPE;
  append_circle_verts(verts, CIRCLE_NSEGMENTS, 1.0f, 0.0f, flag);
  flag = VCLASS_LIGHT_SPOT_SHAPE | VCLASS_LIGHT_SPOT_BLEND;
  append_circle_verts(verts, CIRCLE_NSEGMENTS, 1.0f, 0.0f, flag);
  /* Cone silhouette */
  flag = VCLASS_LIGHT_SPOT_SHAPE | VCLASS_LIGHT_SPOT_CONE;
  for (int a : IndexRange(CIRCLE_NSEGMENTS)) {
    float angle = (2.0f * M_PI * a) / CIRCLE_NSEGMENTS;
    float s = sinf(angle);
    float c = cosf(angle);
    verts.append({{0.0f, 0.0f, 0.0f}, 0});
    verts.append({{s, c, -1.0f}, flag});
  }
  /* Direction Line */
  float zsta = light_distance_z_get('z', true);
  float zend = light_distance_z_get('z', false);
  verts.append({{0.0, 0.0, zsta}, VCLASS_LIGHT_DIST});
  verts.append({{0.0, 0.0, zend}, VCLASS_LIGHT_DIST});
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zsta, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zend, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  return verts;
}

static const Vector<Vertex> &light_spot_cone_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Cone apex */
  verts.append({{0.0f, 0.0f, 0.0f}, 0});
  /* Cone silhouette */
  int flag = VCLASS_LIGHT_SPOT_SHAPE;
  for (int a : IndexRange(CIRCLE_NSEGMENTS + 1)) {
    float angle = (2.0f * M_PI * a) / CIRCLE_NSEGMENTS;
    float s = sinf(-angle);
    float c = cosf(-angle);
    verts.append({{s, c, -1.0f}, flag});
  }
  return verts;
}

static const Vector<Vertex> &light_area_disk_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Light area */
  append_circle_verts(verts, CIRCLE_NSEGMENTS, 0.5f, 0.0f, VCLASS_LIGHT_AREA_SHAPE);
  /* Direction Line */
  float zsta = light_distance_z_get('z', true);
  float zend = light_distance_z_get('z', false);
  verts.append({{0.0, 0.0, zsta}, VCLASS_LIGHT_DIST});
  verts.append({{0.0, 0.0, zend}, VCLASS_LIGHT_DIST});
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zsta, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zend, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  return verts;
}

static const Vector<Vertex> &light_area_square_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float2 p[4] = {{-1.0f, -1.0f}, {-1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, -1.0f}};

  /* Light area */
  int flag = VCLASS_LIGHT_AREA_SHAPE;
  for (int a : IndexRange(4)) {
    for (int b : IndexRange(2)) {
      verts.append({{p[(a + b) % 4] * 0.5f, 0.0f}, flag});
    }
  }
  /* Direction Line */
  float zsta = light_distance_z_get('z', true);
  float zend = light_distance_z_get('z', false);
  verts.append({{0.0, 0.0, zsta}, VCLASS_LIGHT_DIST});
  verts.append({{0.0, 0.0, zend}, VCLASS_LIGHT_DIST});
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zsta, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.2f, zend, VCLASS_LIGHT_DIST | VCLASS_SCREENSPACE);
  return verts;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Probe
 * \{ */

const float sin_pi_3 = 0.86602540378f;
const float cos_pi_3 = 0.5f;

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
    append_circle_verts(verts, DIAMOND_NSEGMENTS, r, zsta, flag);
    append_circle_verts(verts, DIAMOND_NSEGMENTS, r, zend, flag);
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
    append_circle_verts(verts, DIAMOND_NSEGMENTS, r, zsta, flag);
    append_circle_verts(verts, DIAMOND_NSEGMENTS, r, zend, flag);
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
  for (int i : IndexRange(4)) {
    for (int a : IndexRange(2)) {
      verts.append({{p[(i + a) % 4] * r, 0.0}, VCLASS_SCREENSPACE});
    }
  }
  return verts;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Camera
 * \{ */

static const Vector<Vertex> &camera_frame_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float2 p[4] = {{-1.0f, -1.0f}, {-1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, -1.0f}};
  /* Frame */
  for (int a : IndexRange(4)) {
    for (int b : IndexRange(2)) {
      verts.append({{p[(a + b) % 4], 1.0f}, VCLASS_CAMERA_FRAME});
    }
  }
  /* Wires to origin. */
  for (int a : IndexRange(4)) {
    verts.append({{p[a], 1.0f}, VCLASS_CAMERA_FRAME});
    verts.append({{p[a], 0.0f}, VCLASS_CAMERA_FRAME});
  }

  return verts;
}

static const Vector<Vertex> &camera_volume_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_CAMERA_FRAME | VCLASS_CAMERA_VOLUME;
  for (uint3 i : bone_box_solid_tris) {
    for (int a : IndexRange(3)) {
      float3 vert = bone_box_verts[i[a]];
      verts.append({{vert.z, vert.x, vert.y}, flag});
    }
  }

  return verts;
}

static const Vector<Vertex> &camera_volume_wire_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_CAMERA_FRAME | VCLASS_CAMERA_VOLUME;
  for (uint i : bone_box_wire) {
    float3 vert = bone_box_verts[i];
    verts.append({{vert.z, vert.x, vert.y}, flag});
  }

  return verts;
}

static const Vector<Vertex> &camera_tria_wire_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  const float2 p[3] = {{-1.0f, 1.0f}, {1.0f, 1.0f}, {0.0f, 0.0f}};
  for (int a : IndexRange(3)) {
    for (int b : IndexRange(2)) {
      verts.append({{p[(a + b) % 3], 1.0f}, VCLASS_CAMERA_FRAME});
    }
  }

  return verts;
}

static const Vector<Vertex> &camera_tria_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Use camera frame position */
  verts = {{{-1.0f, 1.0f, 1.0f}, VCLASS_CAMERA_FRAME},
           {{1.0f, 1.0f, 1.0f}, VCLASS_CAMERA_FRAME},
           {{0.0f, 0.0f, 1.0f}, VCLASS_CAMERA_FRAME}};
  return verts;
}

static const Vector<Vertex> &camera_distances_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  /* Direction Line */
  verts.extend({{{0.0, 0.0, 0.0}, VCLASS_CAMERA_DIST}, {{0.0, 0.0, 1.0}, VCLASS_CAMERA_DIST}});
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.5f, 0.0f, VCLASS_CAMERA_DIST | VCLASS_SCREENSPACE);
  append_circle_verts(
      verts, DIAMOND_NSEGMENTS, 1.5f, 1.0f, VCLASS_CAMERA_DIST | VCLASS_SCREENSPACE);
  /* Focus cross */
  verts.extend({{{1.0, 0.0, 2.0}, VCLASS_CAMERA_DIST},
                {{-1.0, 0.0, 2.0}, VCLASS_CAMERA_DIST},
                {{0.0, 1.0, 2.0}, VCLASS_CAMERA_DIST},
                {{0.0, -1.0, 2.0}, VCLASS_CAMERA_DIST}});
  return verts;
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Force Fields
 * \{ */

#define CIRCLE_RESOL 32
#define SPIRAL_RESOL 32
#define SIDE_STIPPLE 32

static const Vector<Vertex> &field_wind_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE;
  for (int i : IndexRange(4)) {
    float z = 0.05f * (float)i;
    append_circle_verts(verts, CIRCLE_RESOL, 1.0f, z, flag);
  }

  return verts;
}

static const Vector<Vertex> &field_force_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE | VCLASS_SCREENALIGNED;
  for (int i : IndexRange(3)) {
    float radius = 1.0f + 0.5f * i;
    append_circle_verts(verts, CIRCLE_RESOL, radius, 0.0f, flag);
  }

  return verts;
}

static const Vector<Vertex> &field_vortex_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE;
  for (int a = SPIRAL_RESOL; a > -1; a--) {
    float r = a / (float)SPIRAL_RESOL;
    float angle = (2.0f * M_PI * a) / SPIRAL_RESOL;
    verts.append({{sinf(angle) * r, cosf(angle) * r, 0.0f}, flag});
  }
  for (int a = 1; a <= SPIRAL_RESOL; a++) {
    float r = a / (float)SPIRAL_RESOL;
    float angle = (2.0f * M_PI * a) / SPIRAL_RESOL;
    verts.append({{sinf(angle) * -r, cosf(angle) * -r, 0.0f}, flag});
  }

  return verts;
}

static const Vector<Vertex> &field_curve_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE | VCLASS_SCREENALIGNED;
  append_circle_verts(verts, CIRCLE_RESOL, 1.0f, 0.0f, flag);

  return verts;
}

static const Vector<Vertex> &field_tube_limit_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE;
  /* Caps */
  for (int i : IndexRange(2)) {
    float z = i * 2.0f - 1.0f;
    append_circle_dashed_verts(verts, CIRCLE_RESOL, 1.0f, z, flag);
  }
  /* Side Edges */
  for (int a : IndexRange(4)) {
    float angle = (2.0f * M_PI * a) / 4.0f;
    for (int i : IndexRange(SIDE_STIPPLE)) {
      float z = (i / (float)SIDE_STIPPLE) * 2.0f - 1.0f;
      verts.append({{sinf(angle), cosf(angle), z}, flag});
    }
  }

  return verts;
}

static const Vector<Vertex> &field_cone_limit_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE;
  /* Caps */
  for (int i : IndexRange(2)) {
    float z = i * 2.0f - 1.0f;
    append_circle_dashed_verts(verts, CIRCLE_RESOL, 1.0f, z, flag);
  }
  /* Side Edges */
  for (int a : IndexRange(4)) {
    float angle = (2.0f * M_PI * a) / 4.0f;
    for (int i : IndexRange(SIDE_STIPPLE)) {
      float z = (i / (float)SIDE_STIPPLE) * 2.0f - 1.0f;
      verts.append({{sinf(angle) * z, cosf(angle) * z, z}, flag});
    }
  }

  return verts;
}

static const Vector<Vertex> &field_sphere_limit_verts()
{
  static Vector<Vertex> verts;
  if (!verts.is_empty()) {
    return verts;
  }

  int flag = VCLASS_EMPTY_SIZE | VCLASS_SCREENALIGNED;
  append_circle_dashed_verts(verts, CIRCLE_RESOL, 1.0f, 0.0f, flag);

  return verts;
}

/** \} */

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

  plain_axes = batch_ptr(plain_axes_verts());
  single_arrow = batch_ptr(single_arrow_verts());
  arrows = batch_ptr(arrows_verts());
  quad_wire = batch_ptr(quad_wire_verts());
  circle = batch_ptr(circle_verts(), GPU_PRIM_LINE_STRIP);
  empty_cube = batch_ptr(empty_cube_verts());
  empty_sphere = batch_ptr(empty_sphere_verts());
  empty_cone = batch_ptr(empty_cone_verts());
  empty_cylinder = batch_ptr(empty_cylinder_verts());
  empty_capsule_body = batch_ptr(empty_capsule_body_verts());
  empty_capsule_cap = batch_ptr(empty_capsule_cap_verts());

  /** TODO(Miguel Pozo): Fix. This copies the batch, since the unique ptr releases it. */
  GPUBatch *sphere_solid_batch = GPU_batch_calloc();
  GPU_batch_copy(sphere_solid_batch, DRW_cache_sphere_get(DRW_LOD_LOW));
  sphere_solid = BatchPtr(sphere_solid_batch);

  quad = batch_ptr(quad_verts(), GPU_PRIM_TRI_STRIP);
  grid = batch_ptr(grid_verts(), GPU_PRIM_TRIS);

  metaball_wire_circle = batch_ptr(metaball_wire_circle_verts(), GPU_PRIM_LINE_STRIP);

  speaker = batch_ptr(speaker_verts());

  groundline = batch_ptr(groundline_verts());
  light_icon_inner = batch_ptr(light_icon_inner_verts());
  light_icon_outer = batch_ptr(light_icon_outer_verts());
  light_icon_sun_rays = batch_ptr(light_icon_sun_rays_verts());
  light_point = batch_ptr(light_point_verts());
  light_sun = batch_ptr(light_sun_verts());
  light_spot = batch_ptr(light_spot_verts());
  light_spot_cone = batch_ptr(light_spot_cone_verts());
  light_area_disk = batch_ptr(light_area_disk_verts());
  light_area_square = batch_ptr(light_area_square_verts());

  probe_cube = batch_ptr(probe_cube_verts());
  probe_grid = batch_ptr(probe_grid_verts());
  probe_planar = batch_ptr(probe_planar_verts());

  camera_frame = batch_ptr(camera_frame_verts());
  camera_volume = batch_ptr(camera_volume_verts());
  camera_volume_wire = batch_ptr(camera_volume_wire_verts());
  camera_tria_wire = batch_ptr(camera_tria_wire_verts());
  camera_tria = batch_ptr(camera_tria_verts(), GPU_PRIM_TRIS);
  camera_distances = batch_ptr(camera_distances_verts());

  field_wind = batch_ptr(field_wind_verts());
  field_force = batch_ptr(field_force_verts());
  field_vortex = batch_ptr(field_vortex_verts(), GPU_PRIM_LINE_STRIP);
  field_curve = batch_ptr(field_curve_verts());
  field_tube_limit = batch_ptr(field_tube_limit_verts());
  field_cone_limit = batch_ptr(field_cone_limit_verts());
  field_sphere_limit = batch_ptr(field_sphere_limit_verts());
}

}  // namespace blender::draw::overlay
