/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "grease_pencil_trace_util.hh"

namespace blender::ed::image_trace {

static int to_potrace(const TurnPolicy turn_policy)
{
  switch (turn_policy) {
    case TurnPolicy ::Foreground:
      return POTRACE_TURNPOLICY_BLACK;
    case TurnPolicy ::Background:
      return POTRACE_TURNPOLICY_WHITE;
    case TurnPolicy ::Left:
      return POTRACE_TURNPOLICY_LEFT;
    case TurnPolicy ::Right:
      return POTRACE_TURNPOLICY_RIGHT;
    case TurnPolicy ::Minority:
      return POTRACE_TURNPOLICY_MINORITY;
    case TurnPolicy ::Majority:
      return POTRACE_TURNPOLICY_MAJORITY;
    case TurnPolicy ::Random:
      return POTRACE_TURNPOLICY_RANDOM;
  }
  BLI_assert_unreachable();
  return POTRACE_TURNPOLICY_MINORITY;
}

Bitmap *create_bitmap(const int2 &size)
{
#ifdef WITH_POTRACE
  constexpr int BM_WORDSIZE = int(sizeof(potrace_word));
  constexpr int BM_WORDBITS = 8 * BM_WORDSIZE;

  /* Number of words per scanline. */
  const int32_t dy = (size.x + BM_WORDBITS - 1) / BM_WORDBITS;

  potrace_bitmap_t *bm = (potrace_bitmap_t *)MEM_mallocN(sizeof(potrace_bitmap_t), __func__);
  if (!bm) {
    return nullptr;
  }
  bm->w = size.x;
  bm->h = size.y;
  bm->dy = dy;
  bm->map = (potrace_word *)calloc(size.y, dy * BM_WORDSIZE);
  if (!bm->map) {
    free(bm);
    return nullptr;
  }

  return bm;
#else
  UNUSED_VARS(size);
  return nullptr;
#endif
}

void free_bitmap(Bitmap *bm)
{
#ifdef WITH_POTRACE
  if (bm != nullptr) {
    free(bm->map);
  }
  MEM_SAFE_FREE(bm);
#else
  UNUSED_VARS(bm);
#endif
}

Trace *trace_bitmap(const TraceParams &params, Bitmap &bm)
{
  potrace_param_t *po_params = potrace_param_default();
  if (!po_params) {
    return nullptr;
  }
  po_params->turdsize = params.size_threshold;
  po_params->turnpolicy = to_potrace(params.turn_policy);
  po_params->alphamax = params.alpha_max;
  po_params->opticurve = params.optimize_curves;
  po_params->opttolerance = params.optimize_tolerance;

  potrace_state_t *st = potrace_trace(po_params, &bm);
  potrace_param_free(po_params);

  if (!st || st->status != POTRACE_STATUS_OK) {
    if (st) {
      potrace_state_free(st);
    }
    return nullptr;
  }
  return st;
}

}  // namespace blender::ed::image_trace
