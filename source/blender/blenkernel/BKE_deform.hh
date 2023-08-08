/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2023 Blender Foundation. */

#pragma once

/** \file
 * \ingroup bke
 *
 * Low-level deform utilities.
 */

#include "BKE_deform.h"

#include "BLI_index_mask.hh"
#include "BLI_index_range.hh"
#include "BLI_span.hh"
#include "BLI_task.hh"
#include "BLI_virtual_array.hh"

#include "DNA_meshdata_types.h"

namespace blender::bke {

class VArrayImpl_For_VertexWeights final : public VMutableArrayImpl<float> {
 private:
  MDeformVert *dverts_;
  const int dvert_index_;

 public:
  VArrayImpl_For_VertexWeights(MutableSpan<MDeformVert> dverts, const int dvert_index)
      : VMutableArrayImpl<float>(dverts.size()), dverts_(dverts.data()), dvert_index_(dvert_index)
  {
  }

  VArrayImpl_For_VertexWeights(Span<MDeformVert> dverts, const int dvert_index)
      : VMutableArrayImpl<float>(dverts.size()),
        dverts_(const_cast<MDeformVert *>(dverts.data())),
        dvert_index_(dvert_index)
  {
  }

  float get(const int64_t index) const override
  {
    if (dverts_ == nullptr) {
      return 0.0f;
    }
    if (const MDeformWeight *weight = this->find_weight_at_index(index)) {
      return weight->weight;
    }
    return 0.0f;
  }

  void set(const int64_t index, const float value) override
  {
    MDeformVert &dvert = dverts_[index];
    if (value == 0.0f) {
      if (MDeformWeight *weight = this->find_weight_at_index(index)) {
        weight->weight = 0.0f;
      }
    }
    else {
      MDeformWeight *weight = BKE_defvert_ensure_index(&dvert, dvert_index_);
      weight->weight = value;
    }
  }

  void set_all(Span<float> src) override
  {
    threading::parallel_for(src.index_range(), 4096, [&](const IndexRange range) {
      for (const int64_t i : range) {
        this->set(i, src[i]);
      }
    });
  }

  void materialize(const IndexMask &mask, float *dst) const override
  {
    if (dverts_ == nullptr) {
      mask.foreach_index([&](const int i) { dst[i] = 0.0f; });
    }
    threading::parallel_for(mask.index_range(), 4096, [&](const IndexRange range) {
      mask.slice(range).foreach_index_optimized<int64_t>([&](const int64_t index) {
        if (const MDeformWeight *weight = this->find_weight_at_index(index)) {
          dst[index] = weight->weight;
        }
        else {
          dst[index] = 0.0f;
        }
      });
    });
  }

  void materialize_to_uninitialized(const IndexMask &mask, float *dst) const override
  {
    this->materialize(mask, dst);
  }

 private:
  MDeformWeight *find_weight_at_index(const int64_t index)
  {
    for (MDeformWeight &weight : MutableSpan(dverts_[index].dw, dverts_[index].totweight)) {
      if (weight.def_nr == dvert_index_) {
        return &weight;
      }
    }
    return nullptr;
  }
  const MDeformWeight *find_weight_at_index(const int64_t index) const
  {
    for (const MDeformWeight &weight : Span(dverts_[index].dw, dverts_[index].totweight)) {
      if (weight.def_nr == dvert_index_) {
        return &weight;
      }
    }
    return nullptr;
  }
};

}  // namespace blender::bke
