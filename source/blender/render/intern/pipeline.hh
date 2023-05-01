/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2006 Blender Foundation */

/** \file
 * \ingroup render
 */

#pragma once

struct Render;
struct RenderData;
struct RenderLayer;
struct RenderResult;

#ifdef __cplusplus
extern "C" {
#endif

struct RenderLayer *render_get_single_layer(Render *re, RenderResult *rr);
void render_copy_renderdata(RenderData *to, RenderData *from);

#ifdef __cplusplus
}
#endif
