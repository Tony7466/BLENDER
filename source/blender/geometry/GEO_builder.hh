/* SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_linear_allocator.hh"
#include "BLI_map.hh"
#include "BLI_offset_indices.hh"
#include "BLI_string_ref.hh"
#include "BLI_virtual_array.hh"

namespace blender::geometry::builder {

class Builder {
 private:
  LinearAllocator<> allocator_;

  struct OffsetRange {
    IndexRange range;
    std::optional<Span<int>> offsets;

    void status(std::stringstream &stream, const char *pref = "") const;
  };

  struct Branch {
    Vector<OffsetRange> ordered_elements;
    Map<StringRef, OffsetRange *> named_elements;

    void status(std::stringstream &stream, const char *pref = "") const;
  };

  Map<StringRef, Branch> mesh_primitives_;

  mutable Mesh *result = nullptr;

 public:
  Builder();

  void push_element(const StringRef &primitive_type,
                    const StringRef &identifier,
                    const OffsetRange &element);

  void push_element_by_size(const StringRef &primitive_type,
                            const StringRef identifier,
                            const int total);

  void push_virtual_element(const StringRef primitive_type,
                            const StringRef identifier,
                            const VArray<int> counts);

  IndexRange lookup_range(const StringRef primitive_type, const StringRef identifier) const;

  OffsetIndices<int> lookup_offsets(const StringRef primitive_type,
                                    const StringRef identifier) const;

  void finalize();

  Mesh &mesh() const;

  void status(std::stringstream &stream) const;
};

using MeshBuilder = Builder;

}  // namespace blender::geometry::curve_constraints
