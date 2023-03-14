/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#include "ply_import_ascii.hh"
#include "ply_functions.hh"

#include <algorithm>
#include <fstream>

#include "fast_float.h"
#include <charconv>

static bool is_whitespace(char c)
{
  return c <= ' ';
}

static const char *drop_whitespace(const char *p, const char *end)
{
  while (p < end && is_whitespace(*p)) {
    ++p;
  }
  return p;
}

static const char *drop_non_whitespace(const char *p, const char *end)
{
  while (p < end && !is_whitespace(*p)) {
    ++p;
  }
  return p;
}

static const char *drop_plus(const char *p, const char *end)
{
  if (p < end && *p == '+') {
    ++p;
  }
  return p;
}

static const char *parse_float(const char *p,
                               const char *end,
                               float fallback,
                               float &dst,
                               bool skip_space = true,
                               bool require_trailing_space = false)
{
  if (skip_space) {
    p = drop_whitespace(p, end);
  }
  p = drop_plus(p, end);
  fast_float::from_chars_result res = fast_float::from_chars(p, end, dst);
  if (ELEM(res.ec, std::errc::invalid_argument, std::errc::result_out_of_range)) {
    dst = fallback;
  }
  else if (require_trailing_space && res.ptr < end && !is_whitespace(*res.ptr)) {
    /* If there are trailing non-space characters, do not eat up the number. */
    dst = fallback;
    return p;
  }
  return res.ptr;
}

static const char *parse_int(
    const char *p, const char *end, int fallback, int &dst, bool skip_space = true)
{
  if (skip_space) {
    p = drop_whitespace(p, end);
  }
  p = drop_plus(p, end);
  std::from_chars_result res = std::from_chars(p, end, dst);
  if (ELEM(res.ec, std::errc::invalid_argument, std::errc::result_out_of_range)) {
    dst = fallback;
  }
  return res.ptr;
}

namespace blender::io::ply {

static const float data_type_normalizer[] = {
    127.0f, 255.0f, 32767.0f, 65535.0f, INT_MAX, UINT_MAX, 1.0f, 1.0f};
static_assert(std::size(data_type_normalizer) == PLY_TYPE_COUNT,
              "PLY data type normalization factor table mismatch");

static int get_index(const PlyElement &element, StringRef property)
{
  for (int i = 0, n = int(element.properties.size()); i != n; i++) {
    const auto &prop = element.properties[i];
    if (prop.first == property) {
      return i;
    }
  }
  return -1;
}

static int3 get_vertex_index(const PlyElement &element)
{
  return {get_index(element, "x"), get_index(element, "y"), get_index(element, "z")};
}

static int3 get_color_index(const PlyElement &element)
{
  return {get_index(element, "red"), get_index(element, "green"), get_index(element, "blue")};
}

static int3 get_normal_index(const PlyElement &element)
{
  return {get_index(element, "nx"), get_index(element, "ny"), get_index(element, "nz")};
}

static int2 get_uv_index(const PlyElement &element)
{
  return {get_index(element, "s"), get_index(element, "t")};
}

static void load_vertex_element(fstream &file,
                                const PlyHeader &header,
                                const PlyElement &element,
                                PlyData *data)
{
  /* Figure out vertex component indices. */
  int3 vertex_index = get_vertex_index(element);
  int3 color_index = get_color_index(element);
  int3 normal_index = get_normal_index(element);
  int2 uv_index = get_uv_index(element);
  int alpha_index = get_index(element, "alpha");

  bool has_vertex = vertex_index.x >= 0 && vertex_index.y >= 0 && vertex_index.z >= 0;
  bool has_color = color_index.x >= 0 && color_index.y >= 0 && color_index.z >= 0;
  bool has_normal = normal_index.x >= 0 && normal_index.y >= 0 && normal_index.z >= 0;
  bool has_uv = uv_index.x >= 0 && uv_index.y >= 0;
  bool has_alpha = alpha_index >= 0;

  if (!has_vertex) {
    throw std::runtime_error("Vertex positions are not present in the file");
  }

  float4 color_norm = {1, 1, 1, 1};
  if (has_color) {
    color_norm.x = data_type_normalizer[element.properties[color_index.x].second];
    color_norm.y = data_type_normalizer[element.properties[color_index.y].second];
    color_norm.z = data_type_normalizer[element.properties[color_index.z].second];
  }
  if (has_alpha) {
    color_norm.w = data_type_normalizer[element.properties[alpha_index].second];
  }

  Vector<float> value_vec(element.properties.size());

  for (int i = 0; i < header.vertex_count; i++) {
    std::string line;
    safe_getline(file, line);

    /* Parse whole line as floats. */
    const char *p = line.data();
    const char *end = p + line.size();
    int value_idx = 0;
    while (p < end && value_idx < value_vec.size()) {
      float val;
      p = parse_float(p, end, 0.0f, val);
      value_vec[value_idx++] = val;
    }

    /* Vertex coord */
    float3 vertex3;
    vertex3.x = value_vec[vertex_index.x];
    vertex3.y = value_vec[vertex_index.y];
    vertex3.z = value_vec[vertex_index.z];
    data->vertices.append(vertex3);

    /* Vertex color */
    if (has_color) {
      float4 colors4;
      colors4.x = value_vec[color_index.x] / color_norm.x;
      colors4.y = value_vec[color_index.y] / color_norm.y;
      colors4.z = value_vec[color_index.z] / color_norm.z;
      if (has_alpha) {
        colors4.w = value_vec[alpha_index] / color_norm.w;
      }
      else {
        colors4.w = 1.0f;
      }
      data->vertex_colors.append(colors4);
    }

    /* If normals */
    if (has_normal) {
      float3 normals3;
      normals3.x = value_vec[normal_index.x];
      normals3.y = value_vec[normal_index.y];
      normals3.z = value_vec[normal_index.z];
      data->vertex_normals.append(normals3);
    }

    /* If uv */
    if (has_uv) {
      float2 uvmap;
      uvmap.x = value_vec[uv_index.x];
      uvmap.y = value_vec[uv_index.y];
      data->uv_coordinates.append(uvmap);
    }
  }
}

static void load_face_element(fstream &file,
                              const PlyHeader &header,
                              const PlyElement &element,
                              PlyData *data)
{
  for (int i = 0; i < header.face_count; i++) {
    std::string line;
    getline(file, line);

    const char *p = line.data();
    const char *end = p + line.size();
    int count = 0;
    p = parse_int(p, end, 0, count);

    Array<uint> vertex_indices(count);
    for (int j = 0; j < count; j++) {
      int index;
      p = parse_int(p, end, 0, index);
      /* If the face has a vertex index that is outside the range. */
      if (index >= header.vertex_count) {
        throw std::runtime_error("Vertex index out of bounds");
      }
      vertex_indices[j] = index;
    }
    data->faces.append(vertex_indices);
  }
}

static void load_edge_element(fstream &file,
                              const PlyHeader &header,
                              const PlyElement &element,
                              PlyData *data)
{
  for (int i = 0; i < header.edge_count; i++) {
    std::string line;
    getline(file, line);
    const char *p = line.data();
    const char *end = p + line.size();
    int index0, index1;
    p = parse_int(p, end, 0, index0);
    p = parse_int(p, end, 0, index1);
    data->edges.append(std::make_pair(index0, index1));
  }
}

std::unique_ptr<PlyData> import_ply_ascii(fstream &file, const PlyHeader &header)
{
  std::unique_ptr<PlyData> data = std::make_unique<PlyData>();

  for (const PlyElement &element : header.elements) {
    if (element.name == "vertex") {
      load_vertex_element(file, header, element, data.get());
    }
    else if (element.name == "face") {
      load_face_element(file, header, element, data.get());
    }
    else if (element.name == "edge") {
      load_edge_element(file, header, element, data.get());
    }
  }

  return data;
}


}  // namespace blender::io::ply
