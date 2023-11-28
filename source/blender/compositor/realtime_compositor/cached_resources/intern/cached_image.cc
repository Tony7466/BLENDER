/* SPDX-FileCopyrightText: 2023 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include <cstdint>
#include <memory>

#include "BLI_array.hh"
#include "BLI_assert.h"
#include "BLI_hash.hh"
#include "BLI_listbase.h"

#include "RE_pipeline.h"

#include "GPU_shader.h"
#include "GPU_texture.h"

#include "IMB_imbuf.h"
#include "IMB_imbuf_types.h"

#include "BKE_image.h"
#include "BKE_lib_id.h"

#include "DNA_ID.h"
#include "DNA_image_types.h"

#include "COM_cached_image.hh"
#include "COM_context.hh"
#include "COM_result.hh"
#include "COM_utilities.hh"

namespace blender::realtime_compositor {

/* --------------------------------------------------------------------
 * Cached Image Key.
 */

CachedImageKey::CachedImageKey(ImageUser image_user) : image_user(image_user) {}

uint64_t CachedImageKey::hash() const
{
  return get_default_hash_4(
      image_user.framenr, image_user.layer, image_user.pass, image_user.multi_index);
}

bool operator==(const CachedImageKey &a, const CachedImageKey &b)
{
  return a.image_user.framenr == b.image_user.framenr &&
         a.image_user.layer == b.image_user.layer && a.image_user.pass == b.image_user.pass &&
         a.image_user.multi_index == b.image_user.multi_index;
}

/* --------------------------------------------------------------------
 * Cached Image.
 */

/* Get a suitable texture format for the output of the premultiplication shader, for 8-bit
 * formats, we use half float textures to avoid data loss. This only needs to handles color
 * formats. */
static eGPUTextureFormat get_compatible_texture_format(GPUTexture *texture)
{
  const eGPUTextureFormat original_format = GPU_texture_format(texture);
  switch (original_format) {
    case GPU_RGBA16F:
    case GPU_RGBA32F:
      return original_format;
    case GPU_RGBA8:
    case GPU_SRGB8_A8:
      return GPU_RGBA16F;
    default:
      break;
  }

  BLI_assert_unreachable();
  return original_format;
}

/* Returns a new texture with the alpha of the input premultiplied. The original input is freed. */
static GPUTexture *premultiply_alpha(Context &context, GPUTexture *input)
{
  const eGPUTextureFormat texture_format = get_compatible_texture_format(input);
  const int2 size = int2(GPU_texture_width(input), GPU_texture_height(input));

  GPUTexture *premultiplied_texture = GPU_texture_create_2d(
      "Cached Image", size.x, size.y, 1, texture_format, GPU_TEXTURE_USAGE_GENERAL, nullptr);

  GPUShader *shader = context.get_shader("compositor_premultiply_alpha",
                                         texture_format == GPU_RGBA16F ? ResultPrecision::Half :
                                                                         ResultPrecision::Full);
  GPU_shader_bind(shader);

  const int input_unit = GPU_shader_get_sampler_binding(shader, "input_tx");
  GPU_texture_bind(input, input_unit);

  const int image_unit = GPU_shader_get_sampler_binding(shader, "output_img");
  GPU_texture_image_bind(premultiplied_texture, image_unit);

  compute_dispatch_threads_at_least(shader, size);

  GPU_shader_unbind();
  GPU_texture_unbind(input);
  GPU_texture_image_unbind(premultiplied_texture);
  GPU_texture_free(input);

  return premultiplied_texture;
}

/* Returns true if the image buffer is a color buffer, that is, has more than 8 planes. */
static bool is_color(ImBuf *image_buffer)
{
  return image_buffer->planes > 8;
}

CachedImage::CachedImage(Context &context, Image *image, ImageUser &image_user)
{
  /* Image buffer loaders do redundant alpha premultiplication/unpremultiplication, causing data
   * loss for zero alpha colored regions, so we employ a trick where we free any cached buffers,
   * change the alpha mode to packed to avoid any alpha handling, acquire the buffer, then finally
   * restore the original alpha mode and free the cached buffers once again. */
  BKE_image_free_buffers_ex(image, true);
  const char original_alpha_mode = image->alpha_mode;
  image->alpha_mode = IMA_ALPHA_CHANNEL_PACKED;
  ImBuf *image_buffer = BKE_image_acquire_ibuf(image, &image_user, nullptr);

  texture_ = IMB_create_gpu_texture("Image Texture", image_buffer, true, false);

  /* Compositor images should always be premultiplied, so premultiply the alpha of the texture if
   * it was straight, and set it to 1 using swizzling if it is ignored. */
  if (is_color(image_buffer)) {
    switch (original_alpha_mode) {
      case IMA_ALPHA_STRAIGHT:
        texture_ = premultiply_alpha(context, texture_);
        break;
      case IMA_ALPHA_IGNORE:
        GPU_texture_swizzle_set(texture_, "rgb1");
        break;
    }
  }

  /* Release image buffer and restore the original alpha mode of the image. */
  BKE_image_release_ibuf(image, image_buffer, nullptr);
  image->alpha_mode = original_alpha_mode;
  BKE_image_free_buffers(image);
}

CachedImage::~CachedImage()
{
  GPU_texture_free(texture_);
}

GPUTexture *CachedImage::texture()
{
  return texture_;
}

/* --------------------------------------------------------------------
 * Cached Image Container.
 */

void CachedImageContainer::reset()
{
  /* First, delete all cached images that are no longer needed. */
  for (auto &cached_images_for_id : map_.values()) {
    cached_images_for_id.remove_if([](auto item) { return !item.value->needed; });
  }
  map_.remove_if([](auto item) { return item.value.is_empty(); });

  /* Second, reset the needed status of the remaining cached images to false to ready them to
   * track their needed status for the next evaluation. */
  for (auto &cached_images_for_id : map_.values()) {
    for (auto &value : cached_images_for_id.values()) {
      value->needed = false;
    }
  }
}

/* Get the selected render layer selected assuming the image is a multilayer image. */
static RenderLayer *get_render_layer(Image *image, ImageUser &image_user)
{
  const ListBase *layers = &image->rr->layers;
  return static_cast<RenderLayer *>(BLI_findlink(layers, image_user.layer));
}

/* Get the index of the pass with the given name in the selected render layer's passes list
 * assuming the image is a multilayer image. */
static int get_pass_index(Image *image, ImageUser &image_user, const char *name)
{
  const RenderLayer *render_layer = get_render_layer(image, image_user);
  return BLI_findstringindex(&render_layer->passes, name, offsetof(RenderPass, name));
}

/* Get the index of the view selected in the image user. If the image is not a multi-view image
 * or only has a single view, then zero is returned. Otherwise, if the image is a multi-view
 * image, the index of the selected view is returned. However, note that the value of the view
 * member of the image user is not the actual index of the view. More specifically, the index 0
 * is reserved to denote the special mode of operation "All", which dynamically selects the view
 * whose name matches the view currently being rendered. It follows that the views are then
 * indexed starting from 1. So for non zero view values, the actual index of the view is the
 * value of the view member of the image user minus 1. */
static int get_view_index(Context &context, Image *image, ImageUser &image_user)
{
  /* The image is not a multi-view image, so just return zero. */
  if (!BKE_image_is_multiview(image)) {
    return 0;
  }

  const ListBase *views = &image->rr->views;
  /* There is only one view and its index is 0. */
  if (BLI_listbase_count_at_most(views, 2) < 2) {
    return 0;
  }

  const int view = image_user.view;
  /* The view is not zero, which means it is manually specified and the actual index is then the
   * view value minus 1. */
  if (view != 0) {
    return view - 1;
  }

  /* Otherwise, the view value is zero, denoting the special mode of operation "All", which finds
   * the index of the view whose name matches the view currently being rendered. */
  const char *view_name = context.get_view_name().data();
  const int matched_view = BLI_findstringindex(views, view_name, offsetof(RenderView, name));

  /* No view matches the view currently being rendered, so fallback to the first view. */
  if (matched_view == -1) {
    return 0;
  }

  return matched_view;
}

/* Get a copy of the image user that is appropriate to retrieve the needed image buffer from the
 * image. This essentially sets the appropriate frame, pass, and view that corresponds to the
 * given context and pass name. */
static ImageUser compute_image_user_for_pass(Context &context,
                                             Image *image,
                                             const ImageUser *image_user,
                                             const char *pass_name)
{
  ImageUser image_user_for_pass = *image_user;

  /* Compute the effective frame number of the image if it was animated. */
  BKE_image_user_frame_calc(image, &image_user_for_pass, context.get_frame_number());

  /* Set the needed view. */
  image_user_for_pass.view = get_view_index(context, image, image_user_for_pass);

  /* Set the needed pass. */
  if (BKE_image_is_multilayer(image)) {
    image_user_for_pass.pass = get_pass_index(image, image_user_for_pass, pass_name);
    BKE_image_multilayer_index(image->rr, &image_user_for_pass);
  }
  else {
    BKE_image_multiview_index(image, &image_user_for_pass);
  }

  return image_user_for_pass;
}

GPUTexture *CachedImageContainer::get(Context &context,
                                      Image *image,
                                      const ImageUser *image_user,
                                      const char *pass_name)
{
  ImageUser image_user_for_pass = compute_image_user_for_pass(
      context, image, image_user, pass_name);

  const CachedImageKey key(image_user_for_pass);

  auto &cached_images_for_id = map_.lookup_or_add_default(image->id.name);

  /* Invalidate the cache for that image ID if it was changed and reset the recalculate flag. */
  if (context.query_id_recalc_flag(reinterpret_cast<ID *>(image)) & ID_RECALC_ALL) {
    cached_images_for_id.clear();
  }

  auto &cached_image = *cached_images_for_id.lookup_or_add_cb(
      key, [&]() { return std::make_unique<CachedImage>(context, image, image_user_for_pass); });

  cached_image.needed = true;
  return cached_image.texture();
}

}  // namespace blender::realtime_compositor
