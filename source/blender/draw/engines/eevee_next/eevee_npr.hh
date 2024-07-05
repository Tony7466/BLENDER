/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 */

#pragma once

namespace blender::eevee {

class Instance;

class NPRModule {
 private:
  Instance &inst_;

 public:
  NPRModule(Instance &inst) : inst_(inst){};

  void sync_material();
};

}  // namespace blender::eevee
