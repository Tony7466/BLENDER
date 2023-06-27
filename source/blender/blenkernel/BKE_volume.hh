/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "DNA_volume_types.h"

#include "BLI_bounds_types.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_implicit_sharing.hh"
#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_math_matrix_types.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_offset_indices.hh"
#include "BLI_shared_cache.hh"
#include "BLI_span.hh"
#include "BLI_vector.hh"
#include "BLI_virtual_array.hh"

#include "BKE_attribute.hh"
#include "BKE_attribute_math.hh"

/** \file
 * \ingroup bke
 * \brief Volume data-block.
 */

namespace blender::bke {

/**
 * A C++ class that wraps the DNA struct for better encapsulation and ease of use. It inherits
 * directly from the struct rather than storing a pointer to avoid more complicated ownership
 * handling.
 */
// class VolumeGeometry : public ::VolumeGeometry {
//  public:
//   VolumeGeometry();
//   VolumeGeometry(const VolumeGeometry &other);
//   VolumeGeometry(VolumeGeometry &&other);
//   VolumeGeometry &operator=(const VolumeGeometry &other);
//   VolumeGeometry &operator=(VolumeGeometry &&other);
//   ~VolumeGeometry();
//
//   /* --------------------------------------------------------------------
//    * Accessors.
//    */
//
//   /**
//    * The total number of control points in all curves.
//    */
//   int cells_num() const;
//   IndexRange cells_range() const;
//
//   /**
//    * The largest and smallest position values of evaluated points.
//    */
//   std::optional<Bounds<float3>> bounds_min_max() const;
//
//   /* --------------------------------------------------------------------
//    * Operations.
//    */
//
//  public:
//   /** Call after operations changing the grid tree topology. */
//   void tag_tree_changed();
//
//   AttributeAccessor attributes() const;
//   MutableAttributeAccessor attributes_for_write();
//
//   /* --------------------------------------------------------------------
//    * Attributes.
//    */
//
//   GVArray adapt_domain(const GVArray &varray, eAttrDomain from, eAttrDomain to) const;
//   template<typename T>
//   VArray<T> adapt_domain(const VArray<T> &varray, eAttrDomain from, eAttrDomain to) const
//   {
//     return this->adapt_domain(GVArray(varray), from, to).typed<T>();
//   }
//
//   /* --------------------------------------------------------------------
//    * File Read/Write.
//    */
//
//   void blend_read(BlendDataReader &reader);
//   void blend_write(BlendWriter &writer, ID &id);
// };

}  // namespace blender::bke
