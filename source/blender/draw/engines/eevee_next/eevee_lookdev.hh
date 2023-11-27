/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup eevee
 *
 * World rendering with material handling. Also take care of lookdev
 * HDRI and default material.
 */

#pragma once

#include "DNA_world_types.h"

namespace blender::eevee {

class Instance;

/* -------------------------------------------------------------------- */
/** \name Parameters
 *
 * Parameters used to check changes and to configure the world shader node tree.
 *
 * \{ */
struct LookdevParameters {
  std::string hdri;
  float rot_z = 0.0f;
  float background_opacity = 0.0f;
  float intensity = 1.0f;
  float blur = 0.0f;
  bool show_scene_world = true;

  LookdevParameters();
  LookdevParameters(const ::View3D *v3d);
  bool operator==(const LookdevParameters &other) const;
  bool operator!=(const LookdevParameters &other) const;
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Viewport Override World
 *
 * In a viewport the world can be overridden by a custom HDRI and some settings.
 * \{ */

class LookdevWorld {
 private:
  bNode *environment_node_ = nullptr;
  bNodeSocketValueFloat *intensity_socket_ = nullptr;
  bNodeSocketValueFloat *angle_socket_ = nullptr;
  ::Image image = {};
  ::World world = {};

  LookdevParameters parameters_;

 public:
  LookdevWorld();
  ~LookdevWorld();

  /* Returns true if an update was detected. */
  bool sync(const LookdevParameters &new_parameters);

  ::World *world_get()
  {
    return &world;
  }

  float background_opacity_get()
  {
    return parameters_.background_opacity;
  }
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Lookdev
 *
 * \{ */

class LookdevModule {
 public:
 private:
  Instance &inst_;

  bool enabled_;

  PassSimple metallic_ps_ = {"Lookdev.Metallic"};
  PassSimple diffuse_ps_ = {"Lookdev.Diffuse"};

 public:
  LookdevModule(Instance &inst);
  ~LookdevModule();

  void init();
  void sync();
  void draw_metallic(View &view);
  void draw_diffuse(View &view);

  /**
   * Check if the lookdev spheres should be draw.
   */
  bool is_enabled() const
  {
    return enabled_;
  }
};

/** \} */

}  // namespace blender::eevee
