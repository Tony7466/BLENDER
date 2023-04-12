/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "MEM_guardedalloc.h"

#include "BLI_implicit_sharing.hh"

namespace blender {

class MEMFreeImplicitSharing : public ImplicitSharingInfo {
  void *data_;

 public:
  MEMFreeImplicitSharing(void *data) : ImplicitSharingInfo(1), data_(data)
  {
    BLI_assert(data_ != nullptr);
  }

 private:
  void delete_self_with_data() override
  {
    MEM_freeN(const_cast<void *>(data_));
    MEM_delete(this);
  }
};

ImplicitSharingInfo *sharing_info_for_mem_free(void *data)
{
  return MEM_new<MEMFreeImplicitSharing>(__func__, data);
}

}  // namespace blender
