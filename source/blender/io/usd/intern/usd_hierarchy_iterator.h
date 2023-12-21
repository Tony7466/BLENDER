/* SPDX-FileCopyrightText: 2019 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */
#pragma once

#include "IO_abstract_hierarchy_iterator.h"
#include "usd.h"
#include "usd_exporter_context.h"
#include "usd_skel_convert.h"

#include "BKE_idtype.h"
#include "BLI_set.hh"

#include <map>
#include <string>

#include <pxr/usd/usd/common.h>
#include <pxr/usd/usd/timeCode.h>

struct Depsgraph;
struct Main;
struct Object;

namespace blender::io::usd {

using blender::io::AbstractHierarchyIterator;
using blender::io::AbstractHierarchyWriter;
using blender::io::HierarchyContext;

class USDHierarchyIterator : public AbstractHierarchyIterator {
 private:
  const pxr::UsdStageRefPtr stage_;
  pxr::UsdTimeCode export_time_;
  const USDExportParams &params_;

  ObjExportMap armature_export_map_;
  ObjExportMap skinned_mesh_export_map_;
  ObjExportMap shape_key_mesh_export_map_;

  Map<const void*, const std::string> prim_names_map_;
  Set<const std::string> prim_names_;

 public:
  USDHierarchyIterator(Main *bmain,
                       Depsgraph *depsgraph,
                       pxr::UsdStageRefPtr stage,
                       const USDExportParams &params);

  void set_export_frame(float frame_nr);

  virtual std::string make_valid_name(const std::string &name) const override;

  void process_usd_skel() const;

  bool id_needs_display_name(const ID* id) const;
  bool object_needs_display_name(const Object* object) const;
  bool object_data_needs_display_name(const Object* object) const;

  std::string find_name(const void* pointer) const;

 protected:
  virtual bool mark_as_weak_export(const Object *object) const override;

  virtual AbstractHierarchyWriter *create_transform_writer(
      const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_data_writer(const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_hair_writer(const HierarchyContext *context) override;
  virtual AbstractHierarchyWriter *create_particle_writer(
      const HierarchyContext *context) override;

  virtual void release_writer(AbstractHierarchyWriter *writer) override;

  virtual bool include_data_writers(const HierarchyContext *context) const override;
  virtual bool include_child_writers(const HierarchyContext *context) const override;

  virtual std::vector<std::string> get_computed_material_names(const Material **materials,
                                                               const size_t count) const override;

 private:
  USDExporterContext create_usd_export_context(const HierarchyContext *context);

  void add_usd_skel_export_mapping(const Object *obj, const pxr::SdfPath &usd_path);

  std::string generate_unique_name(const std::string token);
  void store_name(const void* pointer, const std::string name);
  void process_names_for_object(const Object* object);

  void process_materials(const Material** materials, const size_t count);

  virtual std::string get_object_name(const Object *object) const override;
  virtual std::string get_object_data_name(const Object *object) const override;
  virtual std::string get_object_computed_name(const Object* object) const override;
  virtual std::string get_object_data_computed_name(const Object* object) const override;
};

}  // namespace blender::io::usd
