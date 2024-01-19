/*
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * The Original Code is Copyright (C) 2021 NVIDIA Corporation.
 * All rights reserved.
 */
#pragma once

#include "usd_reader_xform.h"

struct Collection;

namespace blender::io::usd {

/* Wraps the UsdGeomPointInstancer schema. Creates a Blender point cloud object. */

class USDPointInstancerReader : public USDXformReader {

 public:
  USDPointInstancerReader(const pxr::UsdPrim &prim,
                          const USDImportParams &import_params,
                          const ImportSettings &settings);

  bool valid() const override;

  void create_object(Main *bmain, double motionSampleTime) override;

  void read_object_data(Main *bmain, double motionSampleTime) override;

  pxr::SdfPathVector proto_paths() const;

  /**
   * Set the given collection on the Collection Info
   * node referenced by the geometry nodes modifier
   * on the object created by the reader.  This assumes
   * create_object() and read_object_data() have already
   * been called.
   *
   * \param bmain: Pointer to Main
   * \param coll: The collection to set
   */
  void set_collection(Main *bmain, Collection *coll);
};

}  // namespace blender::io::usd
