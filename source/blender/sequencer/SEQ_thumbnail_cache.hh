/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#pragma once

struct ImBuf;
struct Sequence;
struct Scene;

#include <string>

namespace blender::seq {

ImBuf *thumbnail_cache_get(Scene *scene, const Sequence *seq, float timeline_frame);
void thumbnail_cache_invalidate_strip(Scene *scene, const Sequence *seq);
void thumbnail_cache_clear(Scene *scene);
void thumbnail_cache_destroy(Scene *scene);
std::string thumbnail_cache_get_stats(Scene *scene);

}  // namespace blender::seq
