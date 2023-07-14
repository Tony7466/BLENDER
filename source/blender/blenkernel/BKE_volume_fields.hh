/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_vector_types.hh"

#include "BKE_attribute.hh"
#include "BKE_geometry_fields.hh"
#include "BKE_volume.h"

#include "intern/volume_grids.hh"

#ifdef WITH_OPENVDB
#  include <openvdb/openvdb.h>
#endif

namespace blender::bke {

#ifdef WITH_OPENVDB

/* -------------------------------------------------------------------- */
/** \name Grid Value Field
 * \{ */

class AnonymousAttributeFieldInput : public GeometryFieldInput {
 private:
  AnonymousAttributeIDPtr anonymous_id_;
  std::string producer_name_;

 public:
  AnonymousAttributeFieldInput(AnonymousAttributeIDPtr anonymous_id,
                               const CPPType &type,
                               std::string producer_name)
      : GeometryFieldInput(type, anonymous_id->user_name()),
        anonymous_id_(std::move(anonymous_id)),
        producer_name_(producer_name)
  {
    category_ = Category::AnonymousAttribute;
  }

  template<typename T>
  static fn::Field<T> Create(AnonymousAttributeIDPtr anonymous_id, std::string producer_name)
  {
    const CPPType &type = CPPType::get<T>();
    auto field_input = std::make_shared<AnonymousAttributeFieldInput>(
        std::move(anonymous_id), type, std::move(producer_name));
    return fn::Field<T>{field_input};
  }

  const AnonymousAttributeIDPtr &anonymous_id() const
  {
    return anonymous_id_;
  }

  GVArray get_varray_for_context(const GeometryFieldContext &context,
                                 const IndexMask &mask) const override;

  std::string socket_inspection_name() const override;

  uint64_t hash() const override;
  bool is_equal_to(const fn::FieldNode &other) const override;
  std::optional<eAttrDomain> preferred_domain(const GeometryComponent &component) const override;
};

/** \} */

#endif

}  // namespace blender::bke
