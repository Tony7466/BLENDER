/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup sequencer
 */

#pragma once

struct bContext;
struct ImBuf;
struct rctf;
struct Sequence;
struct Scene;

#include "BLI_function_ref.hh"

#include <string>

namespace blender::seq {

static constexpr int SEQ_THUMB_SIZE = 256;

ImBuf *thumbnail_cache_get(
    const bContext *C, Scene *scene, const Sequence *seq, float timeline_frame, double cur_time);
void thumbnail_cache_maintain_capacity(Scene *scene, double cur_time);
void thumbnail_cache_invalidate_strip(Scene *scene, const Sequence *seq);
void thumbnail_cache_discard_requests_outside(Scene *scene, const rctf &rect);
void thumbnail_cache_clear(Scene *scene);
void thumbnail_cache_destroy(Scene *scene);

std::string thumbnail_cache_get_stats(Scene *scene);
void thumbnail_cache_for_each_request(
    Scene *scene,
    FunctionRef<void(int index, float timeline_frame, int channel, int frame_index)> callback);

}  // namespace blender::seq
