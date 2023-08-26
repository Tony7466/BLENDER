/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

//#include <iostream>

#include "BLI_linear_allocator.hh"
#include "BLI_map.hh"
#include "BLI_offset_indices.hh"
#include "BLI_string_ref.hh"
#include "BLI_virtual_array.hh"

#include "BKE_mesh.h"

#include "GEO_builder.hh"

namespace blender::geometry::builder {

Builder::Builder()
{
  mesh_primitives_.reserve(4);
  mesh_primitives_.add_new("Vertices", {});
  mesh_primitives_.add_new("Edges", {});
  mesh_primitives_.add_new("Corners", {});
  mesh_primitives_.add_new("Faces", {});
}

void Builder::push_element(const StringRef &primitive_type,
                           const StringRef &identifier,
                           const OffsetRange &element)
{
  Branch &branch = mesh_primitives_.lookup(primitive_type);
  if (branch.ordered_elements.is_empty()) {
    BLI_assert(branch.named_elements.is_empty());
    branch.ordered_elements.append(element);
    branch.named_elements.add_new(identifier, &branch.ordered_elements.first());
    return;
  }

  const OffsetRange &previous_element = branch.ordered_elements.last();
  const int64_t n_shift = previous_element.range.one_after_last();

  OffsetRange shifted_element = element;
  shifted_element.range = shifted_element.range.shift(n_shift);

  const int index = branch.ordered_elements.append_and_get_index(shifted_element);
  branch.named_elements.add_new(identifier, &branch.ordered_elements[index]);
}

void Builder::push_element_by_size(const StringRef &primitive_type,
                                   const StringRef identifier,
                                   const int total)
{
  this->push_element(primitive_type, identifier, OffsetRange{IndexRange(total), std::nullopt});
}

void Builder::push_virtual_element(const StringRef primitive_type,
                                   const StringRef identifier,
                                   const VArray<int> counts)
{
  if (counts.is_single()) {
    const int total_size = counts.get_internal_single() * counts.size();
    this->push_element_by_size(primitive_type, identifier, total_size);
    return;
  }

  MutableSpan<int> accumulations = allocator_.allocate_array<int>(counts.size() + 1);
  accumulations.last() = 0;
  counts.materialize(accumulations.drop_back(1));
  offset_indices::accumulate_counts_to_offsets(accumulations);
  const int total_size = accumulations.last();

  OffsetRange new_element{IndexRange(total_size), accumulations.as_span()};
  this->push_element(primitive_type, identifier, new_element);
}

IndexRange Builder::lookup_range(const StringRef primitive_type, const StringRef identifier) const
{
  const Branch &branch = mesh_primitives_.lookup(primitive_type);
  const OffsetRange &offset_range = *branch.named_elements.lookup(identifier);
  return offset_range.range;
}

OffsetIndices<int> Builder::lookup_offsets(const StringRef primitive_type,
                                           const StringRef identifier) const
{
  const Branch &branch = mesh_primitives_.lookup(primitive_type);
  const OffsetRange &offset_range = *branch.named_elements.lookup(identifier);
  BLI_assert(offset_range.offsets.has_value());
  return OffsetIndices<int>(*offset_range.offsets);
}

void Builder::finalize()
{
  BLI_assert(result == nullptr);

  const auto branch_total = [this](const StringRef primitive_type) {
    const Branch &branch = mesh_primitives_.lookup(primitive_type);
    if (branch.ordered_elements.is_empty()) {
      return 0;
    }
    const int size = branch.ordered_elements.last().range.one_after_last();
    return size;
  };

  const int tot_vert = branch_total("Vertices");
  const int tot_edge = branch_total("Edges");
  const int tot_loop = branch_total("Corners");
  const int tot_face = branch_total("Faces");

  result = BKE_mesh_new_nomain(tot_vert, tot_edge, tot_face, tot_loop);
}

Mesh &Builder::mesh() const
{
  BLI_assert(result != nullptr);
  return *result;
}

void Builder::status(std::stringstream &/*stream*/) const
{
  //stream << "Status:\n";
  //stream << "  Has mesh: " << ((result != nullptr) ? "True" : "False") << ";\n";
  //stream << "  Customs{\n";
  //mesh_primitives_.lookup("Vertices").status(stream, "    Vertices");
  //mesh_primitives_.lookup("Edges").status(stream, "    Edges");
  //mesh_primitives_.lookup("Corners").status(stream, "    Corners");
  //mesh_primitives_.lookup("Faces").status(stream, "    Faces");
  //stream << "};\n";
}

void Builder::Branch::status(std::stringstream &/*stream*/, const char */*pref*/) const
{
  //for (const int index : this->ordered_elements.index_range()) {
  //  const Builder::OffsetRange &offset_range = this->ordered_elements[index];
  //  std::stringstream prefix;
  //  prefix << pref << ": " << index << ": ";
  //  offset_range.status(stream, prefix.str().data());
  //}
}

void Builder::OffsetRange::status(std::stringstream &/*stream*/, const char */*pref*/) const
{
  //stream << pref << "{Start: " << range.start();
  //stream << " : Size: " << range.size();
  //stream << ", End: " << range.one_after_last() << "}\n";
}

}  // namespace blender::geometry::builder
