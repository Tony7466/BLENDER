/* SPDX-FileCopyrightText: 2023 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_cpp_type.hh"
#include "BLI_generic_virtual_array.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_volume.hh"

#include "BKE_attribute.h"
#include "BKE_volume.h"

namespace blender::bke {

/**
 * Result when looking up an attribute from some geometry with the intention of only reading from
 * it.
 */
template<typename T> struct AttributeGridReader {
  /**
   * Virtual array that provides access to the attribute data. This may be empty.
   */
  volume::Grid<T> grid;
  /**
   * Domain where the attribute is stored. This also determines the size of the virtual array.
   */
  eAttrDomain domain;

  /**
   * Information about shared ownership of the attribute array. This will only be provided
   * if the virtual array directly references the contiguous original attribute array.
   */
  const ImplicitSharingInfo *sharing_info;

  const volume::Grid<T> &operator*() const
  {
    return this->grid;
  }
  volume::Grid<T> &operator*()
  {
    return this->grid;
  }

  operator bool() const
  {
    return this->varray;
  }
};

/**
 * Result when looking up an attribute from some geometry with read and write access. After
 * writing to the attribute, the #finish method has to be called. This may invalidate caches based
 * on this attribute.
 */
template<typename T> struct AttributeGridWriter {
  /**
   * Grid pointer giving read and write access to the attribute. This may be empty.
   */
  volume::MutableGrid<T> grid;
  /**
   * Domain where the attribute is stored on the geometry. Also determines the size of the
   * virtual array.
   */
  eAttrDomain domain;
  /**
   * A function that has to be called after the attribute has been edited. This may be empty.
   */
  std::function<void()> tag_modified_fn;

  operator bool() const
  {
    return this->grid != nullptr;
  }

  /**
   * Has to be called after the attribute has been modified.
   */
  void finish()
  {
    if (this->tag_modified_fn) {
      this->tag_modified_fn();
    }
  }
};

/**
 * A generic version of #AttributeReader.
 */
struct GAttributeGridReader {
  volume::GGrid grid;
  eAttrDomain domain;
  const ImplicitSharingInfo *sharing_info;

  operator bool() const
  {
    return this->grid;
  }

  const volume::GGrid &operator*() const
  {
    return this->grid;
  }
  volume::GGrid &operator*()
  {
    return this->grid;
  }

  template<typename T> AttributeGridReader<T> typed() const
  {
    return {grid.typed<T>(), domain, sharing_info};
  }
};

/**
 * A generic version of #AttributeWriter.
 */
struct GAttributeGridWriter {
  volume::GMutableGrid grid;
  eAttrDomain domain;
  std::function<void()> tag_modified_fn;

  operator bool() const
  {
    return this->grid;
  }

  void finish()
  {
    if (this->tag_modified_fn) {
      this->tag_modified_fn();
    }
  }

  template<typename T> AttributeGridWriter<T> typed() const
  {
    return {grid.typed<T>(), domain};
  }
};

}  // namespace blender::bke
