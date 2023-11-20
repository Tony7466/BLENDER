#include "vk_attachments.hh"
#include "vk_texture.hh"

namespace blender::gpu {
bool VKAttachments::check_format(VKTexture &texture,
                                 VkAttachmentDescription2 &attachment_description,
                                 bool enabled_srgb)
{
  VkFormat vk_format = (enabled_srgb) ?
      to_vk_format(texture.device_format_get()):
          to_non_srgb_format(to_vk_format(texture.device_format_get()));

  if (attachment_description.format != vk_format) {
    attachment_description.format = vk_format;
    return false;
  };
  return true;
};
void VKAttachments::description_set(GPUTexture *tex,
                                    const VkAttachmentReference2 &attachment_reference,
                                    VkAttachmentDescription2 &attachment_description,
                                    eRenderpassType &render_pass_enum_)
{
  /*
   * From the best layout for each Texture, we get the type of transition
   * that should be and determine the type of transition as this render pass.
   * So far, it can be expressed in a simple transition structure.
   */
  VKTexture &texture = *reinterpret_cast<VKTexture *>(tex);
  eRenderpassType trans_ty = texture.render_pass_type_get();
  switch (trans_ty) {
    case eRenderpassType::ShaderBinding:
      attachment_description.finalLayout = attachment_description.initialLayout =
          VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
      break;
    case eRenderpassType::Attachment:
      BLI_assert(attachment_reference.layout != VK_IMAGE_LAYOUT_UNDEFINED);
      attachment_description.finalLayout = attachment_description.initialLayout =
          attachment_reference.layout;
      break;
    case eRenderpassType::Storage:
      attachment_description.finalLayout = attachment_description.initialLayout =
          VK_IMAGE_LAYOUT_GENERAL;
      break;
    case eRenderpassType::Any:
      BLI_assert_unreachable();
  };
  if (render_pass_enum_ == eRenderpassType::Any) {
    render_pass_enum_ = trans_ty;
  }
  else {
    /* Is the transition structure unified by several attachments? */
    BLI_assert((trans_ty == render_pass_enum_));
  }
  attachment_description.format = to_vk_format(texture.device_format_get());
}

int VKAttachments::type_get(int view_index, int info_id) const
{
  int i = 0;
  for (const auto &index : idx_[info_id]) {
    if (index == view_index) {
      return i;
    }
    i++;
  }
  BLI_assert_unreachable();
  return -1;
};
}  // namespace blender::gpu
