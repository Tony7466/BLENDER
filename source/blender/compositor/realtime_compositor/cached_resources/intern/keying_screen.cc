/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdint>
#include <memory>
#include <string>

#include "BLI_array.hh"
#include "BLI_hash.hh"
#include "BLI_index_range.hh"
#include "BLI_math_vector_types.hh"
#include "BLI_task.hh"

#include "DNA_defaults.h"
#include "DNA_movieclip_types.h"
#include "DNA_tracking_types.h"

#include "GPU_texture.h"

#include "BKE_movieclip.h"
#include "BKE_tracking.h"

#include "COM_context.hh"
#include "COM_keying_screen.hh"

namespace blender::realtime_compositor {

/* --------------------------------------------------------------------
 * Keying Screen Key.
 */

KeyingScreenKey::KeyingScreenKey(int frame) : frame(frame) {}

uint64_t KeyingScreenKey::hash() const
{
  return get_default_hash(frame);
}

bool operator==(const KeyingScreenKey &a, const KeyingScreenKey &b)
{
  return a.frame == b.frame;
}

/* --------------------------------------------------------------------
 * Keying Screen.
 */

static int2 get_movie_clip_size(MovieClip *movie_clip, int frame_number)
{
  MovieClipUser user = *DNA_struct_default_get(MovieClipUser);
  BKE_movieclip_user_set_frame(&user, frame_number);

  int2 size;
  BKE_movieclip_get_size(movie_clip, &user, &size.x, &size.y);

  return size;
}

KeyingScreen::KeyingScreen(Context &context,
                           MovieClip *movie_clip,
                           MovieTrackingObject *movie_tracking_object)
{
  const int frame = context.get_frame_number();
  const int2 size = get_movie_clip_size(movie_clip, frame);

  texture_ = GPU_texture_create_2d(
      "Keying Screen", size.x, size.y, 1, GPU_RGBA16F, GPU_TEXTURE_USAGE_SHADER_READ, nullptr);
}

KeyingScreen::~KeyingScreen()
{
  GPU_texture_free(texture_);
}

void KeyingScreen::bind_as_texture(GPUShader *shader, const char *texture_name) const
{
  const int texture_image_unit = GPU_shader_get_sampler_binding(shader, texture_name);
  GPU_texture_bind(texture_, texture_image_unit);
}

void KeyingScreen::unbind_as_texture() const
{
  GPU_texture_unbind(texture_);
}

/* --------------------------------------------------------------------
 * Keying Screen Container.
 */

void KeyingScreenContainer::reset()
{
  /* First, delete all cached keying screens that are no longer needed. */
  for (auto &cached_keying_screens_for_id : map_.values()) {
    cached_keying_screens_for_id.remove_if([](auto item) { return !item.value->needed; });
  }
  map_.remove_if([](auto item) { return item.value.is_empty(); });

  /* Second, reset the needed status of the remaining cached keying screens to false to ready them
   * to track their needed status for the next evaluation. */
  for (auto &cached_keying_screens_for_id : map_.values()) {
    for (auto &value : cached_keying_screens_for_id.values()) {
      value->needed = false;
    }
  }
}

KeyingScreen &KeyingScreenContainer::get(Context &context,
                                         MovieClip *movie_clip,
                                         MovieTrackingObject *movie_tracking_object)
{
  const KeyingScreenKey key(context.get_frame_number());

  const std::string id_name = std::string(movie_clip->id.name) +
                              std::string(movie_tracking_object->name);
  auto &cached_keying_screens_for_id = map_.lookup_or_add_default(id_name);

  auto &keying_screen = *cached_keying_screens_for_id.lookup_or_add_cb(key, [&]() {
    return std::make_unique<KeyingScreen>(context, movie_clip, movie_tracking_object);
  });

  keying_screen.needed = true;
  return keying_screen;
}

}  // namespace blender::realtime_compositor
