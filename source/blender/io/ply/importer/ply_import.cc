/* SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup ply
 */

#include "BKE_layer.h"
#include "BKE_lib_id.h"
#include "BKE_mesh.hh"
#include "BKE_object.h"
#include "BKE_report.h"

#include "DNA_collection_types.h"
#include "DNA_object_types.h"
#include "DNA_scene_types.h"

#include "BLI_fileops.hh"
#include "BLI_math_vector.h"
#include "BLI_memory_utils.hh"

#include "DEG_depsgraph.h"
#include "DEG_depsgraph_build.h"

#include "ply_data.hh"
#include "ply_functions.hh"
#include "ply_import.hh"
#include "ply_import_ascii.hh"
#include "ply_import_binary.hh"
#include "ply_import_mesh.hh"

namespace blender::io::ply {

void splitstr(std::string str, Vector<std::string> &words, const StringRef &deli)
{
  int pos;

  while ((pos = int(str.find(deli))) != std::string::npos) {
    words.append(str.substr(0, pos));
    str.erase(0, pos + deli.size());
  }
  /* We add the final word to the vector. */
  words.append(str.substr());
}

enum PlyDataTypes from_string(const StringRef &input)
{
  if (input == "uchar" || input == "uint8") {
    return PlyDataTypes::UCHAR;
  }
  if (input == "char" || input == "int8") {
    return PlyDataTypes::CHAR;
  }
  if (input == "ushort" || input == "uint16") {
    return PlyDataTypes::USHORT;
  }
  if (input == "short" || input == "int16") {
    return PlyDataTypes::SHORT;
  }
  if (input == "uint" || input == "uint32") {
    return PlyDataTypes::UINT;
  }
  if (input == "int" || input == "int32") {
    return PlyDataTypes::INT;
  }
  if (input == "float" || input == "float32") {
    return PlyDataTypes::FLOAT;
  }
  if (input == "double" || input == "float64") {
    return PlyDataTypes::DOUBLE;
  }
  return PlyDataTypes::FLOAT;
}

const char *read_header(fstream &file, PlyHeader &r_header)
{
  std::string line;
  while (true) { /* We break when end_header is encountered. */
    safe_getline(file, line);
    if (r_header.header_size == 0 && line != "ply") {
      return "Invalid PLY header.";
    }
    r_header.header_size++;
    Vector<std::string> words{};
    splitstr(line, words, " ");

    if (strcmp(words[0].c_str(), "format") == 0) {
      if (strcmp(words[1].c_str(), "ascii") == 0) {
        r_header.type = PlyFormatType::ASCII;
      }
      else if (strcmp(words[1].c_str(), "binary_big_endian") == 0) {
        r_header.type = PlyFormatType::BINARY_BE;
      }
      else if (strcmp(words[1].c_str(), "binary_little_endian") == 0) {
        r_header.type = PlyFormatType::BINARY_LE;
      }
    }
    else if (strcmp(words[0].c_str(), "element") == 0) {
      PlyElement element;
      element.name = words[1];
      element.count = std::stoi(words[2]);
      r_header.elements.append(element);
      if (strcmp(words[1].c_str(), "vertex") == 0) {
        r_header.vertex_count = std::stoi(words[2]);
      }
      else if (strcmp(words[1].c_str(), "face") == 0) {
        r_header.face_count = std::stoi(words[2]);
      }
      else if (strcmp(words[1].c_str(), "edge") == 0) {
        r_header.edge_count = std::stoi(words[2]);
      }
    }
    else if (strcmp(words[0].c_str(), "property") == 0) {
      std::pair<std::string, PlyDataTypes> property;
      property.first = words[2];
      property.second = from_string(words[1]);
      r_header.elements.last().properties.append(property);
    }
    else if (words[0] == "end_header") {
      break;
    }
    else if ((words[0][0] >= '0' && words[0][0] <= '9') || words[0][0] == '-' || line.empty() ||
             file.eof()) {
      /* A value was found before we broke out of the loop. No end_header. */
      return "No end_header.";
    }
  }
  return nullptr;
}

void importer_main(bContext *C, const PLYImportParams &import_params, wmOperator *op)
{
  Main *bmain = CTX_data_main(C);
  Scene *scene = CTX_data_scene(C);
  ViewLayer *view_layer = CTX_data_view_layer(C);
  importer_main(bmain, scene, view_layer, import_params, op);
}

void importer_main(Main *bmain,
                   Scene *scene,
                   ViewLayer *view_layer,
                   const PLYImportParams &import_params,
                   wmOperator *op)
{
  /* File base name used for both mesh and object. */
  char ob_name[FILE_MAX];
  BLI_strncpy(ob_name, BLI_path_basename(import_params.filepath), FILE_MAX);
  BLI_path_extension_replace(ob_name, FILE_MAX, "");

  /* Parse header. */
  fstream infile(import_params.filepath, std::ios::in | std::ios::binary);
  PlyHeader header;
  const char *err = read_header(infile, header);
  if (err != nullptr) {
    fprintf(stderr, "PLY Importer: %s: %s\n", ob_name, err);
    BKE_reportf(op->reports, RPT_ERROR, "PLY Importer: %s: %s", ob_name, err);
    return;
  }

  /* Create mesh and do all prep work. */
  Mesh *mesh = BKE_mesh_add(bmain, ob_name);

  BKE_view_layer_base_deselect_all(scene, view_layer);
  LayerCollection *lc = BKE_layer_collection_get_active(view_layer);
  Object *obj = BKE_object_add_only_object(bmain, OB_MESH, ob_name);
  BKE_mesh_assign_object(bmain, obj, mesh);
  BKE_collection_object_add(bmain, lc->collection, obj);
  BKE_view_layer_synced_ensure(scene, view_layer);
  Base *base = BKE_view_layer_base_find(view_layer, obj);
  BKE_view_layer_base_select_and_set_active(view_layer, base);

  /* Parse actual file data. */
  try {
    std::unique_ptr<PlyData> data;
    if (header.type == PlyFormatType::ASCII) {
      data = import_ply_ascii(infile, header);
    }
    else {
      data = import_ply_binary(infile, &header);
    }

    Mesh *temp_val = convert_ply_to_mesh(*data, mesh, import_params);
    if (import_params.merge_verts && temp_val != mesh) {
      BKE_mesh_nomain_to_mesh(temp_val, mesh, obj);
    }
  }
  catch (std::exception &e) {
    fprintf(stderr, "PLY Importer: failed to read file. %s.\n", e.what());
    BKE_report(op->reports, RPT_ERROR, "PLY Importer: failed to parse file.");
    return;
  }

  /* Object matrix and finishing up. */
  float global_scale = import_params.global_scale;
  if ((scene->unit.system != USER_UNIT_NONE) && import_params.use_scene_unit) {
    global_scale *= scene->unit.scale_length;
  }
  float scale_vec[3] = {global_scale, global_scale, global_scale};
  float obmat3x3[3][3];
  unit_m3(obmat3x3);
  float obmat4x4[4][4];
  unit_m4(obmat4x4);
  /* +Y-forward and +Z-up are the Blender's default axis settings. */
  mat3_from_axis_conversion(
      IO_AXIS_Y, IO_AXIS_Z, import_params.forward_axis, import_params.up_axis, obmat3x3);
  copy_m4_m3(obmat4x4, obmat3x3);
  rescale_m4(obmat4x4, scale_vec);
  BKE_object_apply_mat4(obj, obmat4x4, true, false);

  DEG_id_tag_update(&lc->collection->id, ID_RECALC_COPY_ON_WRITE);
  int flags = ID_RECALC_TRANSFORM | ID_RECALC_GEOMETRY | ID_RECALC_ANIMATION |
              ID_RECALC_BASE_FLAGS;
  DEG_id_tag_update_ex(bmain, &obj->id, flags);
  DEG_id_tag_update(&scene->id, ID_RECALC_BASE_FLAGS);
  DEG_relations_tag_update(bmain);

  infile.close();
}
}  // namespace blender::io::ply
