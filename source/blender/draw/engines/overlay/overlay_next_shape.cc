/* SPDX-FileCopyrightText: 2023 Blender Authors
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

struct VertShaded {
  float3 pos;
  int v_class;
  float3 nor;
};

/* Caller gets ownership of the #gpu::VertBuf. */
static gpu::VertBuf *vbo_from_vector(Vector<Vertex> &vector)
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&format, "vclass", GPU_COMP_I32, 1, GPU_FETCH_INT);
  }

  gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
  GPU_vertbuf_data_alloc(*vbo, vector.size());
  vbo->data<Vertex>().copy_from(vector);
  return vbo;
}

static gpu::VertBuf *vbo_from_vector(Vector<VertShaded> &vector)
{
  static GPUVertFormat format = {0};
  if (format.attr_len == 0) {
    GPU_vertformat_attr_add(&format, "pos", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
    GPU_vertformat_attr_add(&format, "vclass", GPU_COMP_I32, 1, GPU_FETCH_INT);
    GPU_vertformat_attr_add(&format, "nor", GPU_COMP_F32, 3, GPU_FETCH_FLOAT);
  }

  gpu::VertBuf *vbo = GPU_vertbuf_create_with_format(format);
  GPU_vertbuf_data_alloc(*vbo, vector.size());
  vbo->data<VertShaded>().copy_from(vector);
  return vbo;
}

enum VertexClass {
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

/* Sphere shape resolution */
/* Low */
#define DRW_SPHERE_SHAPE_LATITUDE_LOW 32
#define DRW_SPHERE_SHAPE_LONGITUDE_LOW 24
/* Medium */
#define DRW_SPHERE_SHAPE_LATITUDE_MEDIUM 64
#define DRW_SPHERE_SHAPE_LONGITUDE_MEDIUM 48
/* High */
#define DRW_SPHERE_SHAPE_LATITUDE_HIGH 80
#define DRW_SPHERE_SHAPE_LONGITUDE_HIGH 60

#define DIAMOND_NSEGMENTS 4

static constexpr float bone_box_verts[8][3] = {
    {1.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, -1.0f},
    {-1.0f, 0.0f, -1.0f},
    {-1.0f, 0.0f, 1.0f},
    {1.0f, 1.0f, 1.0f},
    {1.0f, 1.0f, -1.0f},
    {-1.0f, 1.0f, -1.0f},
    {-1.0f, 1.0f, 1.0f},
};

static constexpr std::array<uint, 24> bone_box_wire = {
    0, 1, 1, 2, 2, 3, 3, 0, 4, 5, 5, 6, 6, 7, 7, 4, 0, 4, 1, 5, 2, 6, 3, 7,
};

static const std::array<uint3, 12> bone_box_solid_tris{
    uint3{0, 2, 1}, /* bottom */
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

/* A single ring of vertices. */
static Vector<float2> ring_vertices(const float radius,
                                    const int segments,
                                    const bool half = false)
{
  Vector<float2> verts;
  const float full = (half ? 1.0f : 2.0f) * M_PI;
  for (const int i : IndexRange(segments + (half ? 1 : 0))) {
    const float angle = (full * i) / segments;
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

  Vector<Vertex> verts;
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

static void append_as_lines_cyclic(
    Vector<Vertex> &dest, Vector<float2> verts, float z, int flag, bool dashed = false)
{
  const int step = dashed ? 2 : 1;
  for (int i : IndexRange(verts.size() / step)) {
    for (int j : IndexRange(2)) {
      float2 cv = verts[(i * step + j) % (verts.size())];
      dest.append({{cv[0], cv[1], z}, flag});
    }
  }
}

static VertShaded sphere_lat_lon_vert(const float2 &lat_pt, const float2 &lon_pt)
{
  const float x = lon_pt.y * lat_pt.x;
  const float y = lon_pt.x;
  const float z = lon_pt.y * lat_pt.y;
  return VertShaded{{x, y, z}, VCLASS_EMPTY_SCALED, {x, y, z}};
}

static void append_sphere(Vector<VertShaded> &dest, const eDRWLevelOfDetail level_of_detail)
{
  BLI_assert(level_of_detail >= DRW_LOD_LOW && level_of_detail < DRW_LOD_MAX);
  static const std::array<Vector<float2>, DRW_LOD_MAX> latitude_rings = {
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LATITUDE_LOW),
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LATITUDE_MEDIUM),
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LATITUDE_HIGH)};
  static const std::array<Vector<float2>, DRW_LOD_MAX> longitude_half_rings = {
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LONGITUDE_LOW, true),
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LONGITUDE_MEDIUM, true),
      ring_vertices(1.0f, DRW_SPHERE_SHAPE_LONGITUDE_HIGH, true)};

  const Vector<float2> &latitude_ring = latitude_rings[level_of_detail];
  const Vector<float2> &longitude_half_ring = longitude_half_rings[level_of_detail];

  for (const int i : latitude_ring.index_range()) {
    const float2 lat_pt = latitude_ring[i];
    const float2 next_lat_pt = latitude_ring[(i + 1) % latitude_ring.size()];
    for (const int j : IndexRange(longitude_half_ring.size() - 1)) {
      const float2 lon_pt = longitude_half_ring[j];
      const float2 next_lon_pt = longitude_half_ring[j + 1];
      if (j != 0) { /* Pole */
        dest.append(sphere_lat_lon_vert(next_lat_pt, next_lon_pt));
        dest.append(sphere_lat_lon_vert(next_lat_pt, lon_pt));
        dest.append(sphere_lat_lon_vert(lat_pt, lon_pt));
      }
      if (j != longitude_half_ring.index_range().last(1)) { /* Pole */
        dest.append(sphere_lat_lon_vert(lat_pt, next_lon_pt));
        dest.append(sphere_lat_lon_vert(next_lat_pt, next_lon_pt));
        dest.append(sphere_lat_lon_vert(lat_pt, lon_pt));
      }
    }
  }
}

ShapeCache::ShapeCache()
{
  /* quad_wire */
  {
    Vector<Vertex> verts;
    verts.append({{-1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{-1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{-1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{+1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{+1.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{+1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{+1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{-1.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED});

    quad_wire = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* plain_axes */
  {
    Vector<Vertex> verts;
    verts.append({{0.0f, -1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, +1.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{-1.0f, 0.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{+1.0f, 0.0f, 0.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, 0.0f, -1.0f}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, 0.0f, +1.0f}, VCLASS_EMPTY_SCALED});

    plain_axes = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* single_arrow */
  {
    Vector<Vertex> verts;
    float p[3][3] = {{0}};
    p[0][2] = 1.0f;
    p[1][0] = 0.035f;
    p[1][1] = 0.035f;
    p[2][0] = -0.035f;
    p[2][1] = 0.035f;
    p[1][2] = p[2][2] = 0.75f;
    for (int sides : IndexRange(4)) {
      if (sides % 2 == 1) {
        p[1][0] = -p[1][0];
        p[2][1] = -p[2][1];
      }
      else {
        p[1][1] = -p[1][1];
        p[2][0] = -p[2][0];
      }
      for (int i = 0, a = 1; i < 2; i++, a++) {
        verts.append({{p[i][0], p[i][1], p[i][2]}, VCLASS_EMPTY_SCALED});
        verts.append({{p[a][0], p[a][1], p[a][2]}, VCLASS_EMPTY_SCALED});
      }
    }
    verts.append({{0.0f, 0.0f, 0.0}, VCLASS_EMPTY_SCALED});
    verts.append({{0.0f, 0.0f, 0.75f}, VCLASS_EMPTY_SCALED});

    single_arrow = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* cube */
  {
    Vector<Vertex> verts;
    for (auto index : bone_box_wire) {
      float x = bone_box_verts[index][0];
      float y = bone_box_verts[index][1] * 2.0 - 1.0f;
      float z = bone_box_verts[index][2];
      verts.append({{x, y, z}, VCLASS_EMPTY_SCALED});
    }

    cube = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* circle */
  {
    constexpr int resolution = 64;
    Vector<float2> ring = ring_vertices(1.0f, resolution);

    Vector<Vertex> verts;
    for (int a : IndexRange(resolution + 1)) {
      float2 cv = ring[a % resolution];
      verts.append({{cv.x, 0.0f, cv.y}, VCLASS_EMPTY_SCALED});
    }

    circle = BatchPtr(GPU_batch_create_ex(
        GPU_PRIM_LINE_STRIP, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* empty_spehere */
  {
    Vector<Vertex> verts = sphere_axes_circles(1.0f, VCLASS_EMPTY_SCALED, 32);

    empty_sphere = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* empty_cone */
  {
    constexpr int resolution = 8;
    Vector<float2> ring = ring_vertices(1.0f, resolution);

    Vector<Vertex> verts;
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

    empty_cone = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* arrows */
  {
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
        float2(-0.95f, 1.00f) * z_axis_name_scale,
        float2(0.95f, 1.00f) * z_axis_name_scale,
        float2(0.95f, 1.00f) * z_axis_name_scale,
        float2(0.95f, 0.90f) * z_axis_name_scale,
        float2(0.95f, 0.90f) * z_axis_name_scale,
        float2(-1.00f, -0.90f) * z_axis_name_scale,
        float2(-1.00f, -0.90f) * z_axis_name_scale,
        float2(-1.00f, -1.00f) * z_axis_name_scale,
        float2(-1.00f, -1.00f) * z_axis_name_scale,
        float2(1.00f, -1.00f) * z_axis_name_scale,
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

    Vector<Vertex> verts;
    for (int axis : IndexRange(3)) {
      /* Vertex layout is XY screen position and axis in Z.
       * Fractional part of Z is a positive offset at axis unit position. */
      int flag = VCLASS_EMPTY_AXES | VCLASS_SCREENALIGNED;
      /* Center to axis line. */
      verts.append({{0.0f, 0.0f, 0.0f}, 0});
      verts.append({{0.0f, 0.0f, float(axis)}, flag});
      /* Axis end marker. */
      constexpr int marker_fill_layer = 6;
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
    arrows = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* metaball_wire_circle */
  {
    constexpr int resolution = 64;
    constexpr float radius = 1.0f;
    Vector<float2> ring = ring_vertices(radius, resolution);

    Vector<Vertex> verts;
    for (int i : IndexRange(resolution + 1)) {
      float2 cv = ring[i % resolution];
      verts.append({{cv[0], cv[1], 0.0f}, VCLASS_SCREENALIGNED});
    }
    metaball_wire_circle = BatchPtr(GPU_batch_create_ex(
        GPU_PRIM_LINE_STRIP, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  };
  /* speaker */
  {
    const int segments = 16;
    Vector<Vertex> verts;

    for (int j = 0; j < 3; j++) {
      float z = 0.25f * j - 0.125f;
      float r = (j == 0 ? 0.5f : 0.25f);

      verts.append({{r, 0.0f, z}});

      for (int i = 1; i < segments; i++) {
        float x = cosf(2.0f * float(M_PI) * i / segments) * r;
        float y = sinf(2.0f * float(M_PI) * i / segments) * r;
        Vertex v{{x, y, z}};
        verts.append(v);
        verts.append(v);
      }
      Vertex v{{r, 0.0f, z}};
      verts.append(v);
    }

    for (int j = 0; j < 4; j++) {
      float x = (((j + 1) % 2) * (j - 1)) * 0.5f;
      float y = ((j % 2) * (j - 2)) * 0.5f;
      for (int i = 0; i < 3; i++) {
        if (i == 1) {
          x *= 0.5f;
          y *= 0.5f;
        }

        float z = 0.25f * i - 0.125f;
        Vertex v{{x, y, z}};
        verts.append(v);
        if (i == 1) {
          verts.append(v);
        }
      }
    }
    speaker = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* camera distances */
  {
    static const Vector<float2> diamond = ring_vertices(1.5f, DIAMOND_NSEGMENTS);
    static const Vector<float2> cross = {{1.0f, 0.0f}, {-1.0f, 0.0f}, {0.0f, 1.0f}, {0.0f, -1.0f}};

    Vector<Vertex> verts;
    verts.append({{0.0f, 0.0f, 0.0f}, VCLASS_CAMERA_DIST});
    verts.append({{0.0f, 0.0f, 1.0f}, VCLASS_CAMERA_DIST});

    append_as_lines_cyclic(verts, diamond, 0.0f, VCLASS_CAMERA_DIST | VCLASS_SCREENSPACE);
    append_as_lines_cyclic(verts, diamond, 1.0f, VCLASS_CAMERA_DIST | VCLASS_SCREENSPACE);

    /* Focus cross */
    for (const float2 &point : cross) {
      verts.append({{point.x, point.y, 2.0f}, VCLASS_CAMERA_DIST});
    }
    camera_distances = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* camera frame */
  {
    static const Vector<float2> rect{{-1.0f, -1.0f}, {-1.0f, 1.0f}, {1.0f, 1.0f}, {1.0f, -1.0f}};
    Vector<Vertex> verts;
    /* Frame */
    append_as_lines_cyclic(verts, rect, 1.0f, VCLASS_CAMERA_FRAME);
    /* Wires to origin. */
    for (const float2 &point : rect) {
      verts.append({{point.x, point.y, 1.0f}, VCLASS_CAMERA_FRAME});
      verts.append({{point.x, point.y, 0.0f}, VCLASS_CAMERA_FRAME});
    }
    camera_frame = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* camera tria */
  {
    static const Vector<float2> triangle = {{-1.0f, 1.0f}, {1.0f, 1.0f}, {0.0f, 0.0f}};
    Vector<Vertex> verts(2 * 3);
    /* Wire */
    append_as_lines_cyclic(verts, triangle, 1.0f, VCLASS_CAMERA_FRAME);
    camera_tria_wire = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));

    verts.clear();
    /* Triangle */
    for (const float2 &point : triangle) {
      verts.append({{point.x, point.y, 1.0f}, VCLASS_CAMERA_FRAME});
    }
    camera_tria = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_TRIS, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* camera volume */
  {
    Vector<Vertex> verts;
    for (const uint3 &tri : bone_box_solid_tris) {
      for (const int i : IndexRange(tri.type_length)) {
        const int v = tri[i];
        const float x = bone_box_verts[v][2];
        const float y = bone_box_verts[v][0];
        const float z = bone_box_verts[v][1];
        verts.append({{x, y, z}, VCLASS_CAMERA_FRAME | VCLASS_CAMERA_VOLUME});
      }
    }
    camera_volume = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_TRIS, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* camera volume wire */
  {
    Vector<Vertex> verts(bone_box_wire.size());
    for (int i : bone_box_wire) {
      const float x = bone_box_verts[i][2];
      const float y = bone_box_verts[i][0];
      const float z = bone_box_verts[i][1];
      verts.append({{x, y, z}, VCLASS_CAMERA_FRAME | VCLASS_CAMERA_VOLUME});
    }
    camera_volume_wire = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_LINES, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
  /* spheres */
  {
    Vector<VertShaded> verts;
    append_sphere(verts, DRW_LOD_LOW);
    sphere_low_detail = BatchPtr(
        GPU_batch_create_ex(GPU_PRIM_TRIS, vbo_from_vector(verts), nullptr, GPU_BATCH_OWNS_VBO));
  }
}

}  // namespace blender::draw::overlay
