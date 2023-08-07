/* SPDX-FileCopyrightText: 2011-2022 Blender Foundation
 *
 * SPDX-License-Identifier: Apache-2.0 */

#include "device/multi/device.h"

#include <sstream>
#include <stdlib.h>

#include "bvh/multi.h"

#include "device/device.h"
#include "device/queue.h"

#include "scene/geometry.h"

#include "util/foreach.h"
#include "util/list.h"
#include "util/log.h"
#include "util/map.h"
#include "util/time.h"
#include "util/tbb.h"

CCL_NAMESPACE_BEGIN

class MultiDevice : public Device {
 public:
  struct SubDevice {
    Stats stats;
    unique_ptr<Device> device;
    map<device_ptr, device_ptr> ptr_map;
    int peer_island_index = -1;
  };

  /* Switch from list to a vector to make the parallel_for easily map to the integer id.
     Also id now could be used to access the real device pointer more quickly. Also, since
     the vector reallocates the memory on resize the sub-devices are stored as pointers. */
  vector<unique_ptr<SubDevice>> devices;
  device_ptr unique_key;
  vector<vector<SubDevice *>> peer_islands;

  MultiDevice(const DeviceInfo &info, Stats &stats, Profiler &profiler)
      : Device(info, stats, profiler), unique_key(1)
  {
    int cpu_device_idx = -1;
    foreach (const DeviceInfo &subinfo, info.multi_devices) {
      /* Always add CPU devices at the back since GPU devices can change
       * host memory pointers, which CPU uses as device pointer. */
      unique_ptr<SubDevice> sub = make_unique<SubDevice>();
      if (subinfo.type == DEVICE_CPU) {
        assert(cpu_device_idx == -1);
        cpu_device_idx = devices.size();
      }
      sub->device = std::unique_ptr<Device>(Device::create(subinfo, sub->stats, profiler));
      devices.emplace_back(std::move(sub));
    }

    /* Swap the CPU device with the last device to ensure the CPU device is the last */
    {
      int last = devices.size() - 1;
      if ((cpu_device_idx != -1) && (cpu_device_idx != last)) {
        std::swap(devices[last], devices[cpu_device_idx]);
      }
    }
    /* Build a list of peer islands for the available render devices */
    foreach (auto &sub, devices) {
      /* First ensure that every device is in at least once peer island */
      if (sub->peer_island_index < 0) {
        peer_islands.emplace_back();
        sub->peer_island_index = (int)peer_islands.size() - 1;
        peer_islands[sub->peer_island_index].push_back(sub.get());
      }

      if (!info.has_peer_memory) {
        continue;
      }

      /* Second check peer access between devices and fill up the islands accordingly */
      foreach (auto &peer_sub, devices) {
        if (peer_sub->peer_island_index < 0 &&
            peer_sub->device->info.type == sub->device->info.type &&
            peer_sub->device->check_peer_access(sub->device.get()))
	{
          peer_sub->peer_island_index = sub->peer_island_index;
          peer_islands[sub->peer_island_index].push_back(peer_sub.get());
        }
      }
    }
  }

  ~MultiDevice() {}

  int get_num_devices() const override
  {
    return devices.size();
  }

  const string &error_message() override
  {
    error_msg.clear();

    foreach (auto &sub, devices)
      error_msg += sub->device->error_message();

    return error_msg;
  }

  virtual BVHLayoutMask get_bvh_layout_mask(uint kernel_features) const override
  {
    BVHLayoutMask bvh_layout_mask = BVH_LAYOUT_ALL;
    BVHLayoutMask bvh_layout_mask_all = BVH_LAYOUT_NONE;
    foreach (const auto &sub_device, devices) {
      BVHLayoutMask device_bvh_layout_mask = sub_device->device->get_bvh_layout_mask(
          kernel_features);
      bvh_layout_mask &= device_bvh_layout_mask;
      bvh_layout_mask_all |= device_bvh_layout_mask;
    }

    /* With multiple OptiX devices, every device needs its own acceleration structure */
    if (bvh_layout_mask == BVH_LAYOUT_OPTIX) {
      return BVH_LAYOUT_MULTI_OPTIX;
    }

    /* With multiple Metal devices, every device needs its own acceleration structure */
    if (bvh_layout_mask == BVH_LAYOUT_METAL) {
      return BVH_LAYOUT_MULTI_METAL;
    }

    if (bvh_layout_mask == BVH_LAYOUT_HIPRT) {
      return BVH_LAYOUT_MULTI_HIPRT;
    }

    /* With multiple oneAPI devices, every device needs its own acceleration structure */
    if (bvh_layout_mask == BVH_LAYOUT_EMBREEGPU) {
      return BVH_LAYOUT_MULTI_EMBREEGPU;
    }

    /* When devices do not share a common BVH layout, fall back to creating one for each */
    const BVHLayoutMask BVH_LAYOUT_OPTIX_EMBREE = (BVH_LAYOUT_OPTIX | BVH_LAYOUT_EMBREE);
    if ((bvh_layout_mask_all & BVH_LAYOUT_OPTIX_EMBREE) == BVH_LAYOUT_OPTIX_EMBREE) {
      return BVH_LAYOUT_MULTI_OPTIX_EMBREE;
    }
    const BVHLayoutMask BVH_LAYOUT_METAL_EMBREE = (BVH_LAYOUT_METAL | BVH_LAYOUT_EMBREE);
    if ((bvh_layout_mask_all & BVH_LAYOUT_METAL_EMBREE) == BVH_LAYOUT_METAL_EMBREE) {
      return BVH_LAYOUT_MULTI_METAL_EMBREE;
    }
    const BVHLayoutMask BVH_LAYOUT_EMBREEGPU_EMBREE = (BVH_LAYOUT_EMBREEGPU | BVH_LAYOUT_EMBREE);
    if ((bvh_layout_mask_all & BVH_LAYOUT_EMBREEGPU_EMBREE) == BVH_LAYOUT_EMBREEGPU_EMBREE) {
      return BVH_LAYOUT_MULTI_EMBREEGPU_EMBREE;
    }

    const BVHLayoutMask BVH_LAYOUT_HIPRT_EMBREE = (BVH_LAYOUT_HIPRT | BVH_LAYOUT_EMBREE);
    if ((bvh_layout_mask_all & BVH_LAYOUT_HIPRT_EMBREE) == BVH_LAYOUT_HIPRT_EMBREE) {
      return BVH_LAYOUT_MULTI_HIPRT_EMBREE;
    }

    return bvh_layout_mask;
  }

  bool load_kernels(const uint kernel_features) override
  {
    foreach (auto &sub, devices)
      if (!sub->device->load_kernels(kernel_features))
        return false;

    return true;
  }

  bool load_osl_kernels() override
  {
    foreach (auto &sub, devices)
      if (!sub->device->load_osl_kernels())
        return false;

    return true;
  }

  void build_bvh(BVH *bvh, DeviceScene *dscene, Progress &progress, bool refit) override
  {
    /* Try to build and share a single acceleration structure, if possible */
    if (bvh->params.bvh_layout == BVH_LAYOUT_BVH2 || bvh->params.bvh_layout == BVH_LAYOUT_EMBREE) {
      devices.back()->device->build_bvh(bvh, dscene, progress, refit);
      return;
    }

    assert(bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_HIPRT ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_EMBREEGPU ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX_EMBREE ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL_EMBREE ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_HIPRT_EMBREE ||
           bvh->params.bvh_layout == BVH_LAYOUT_MULTI_EMBREEGPU_EMBREE);

    BVHMulti *const bvh_multi = static_cast<BVHMulti *>(bvh);
    bvh_multi->sub_bvhs.resize(devices.size());

    /* Broadcast acceleration structure build to all render devices */
    parallel_for(
        size_t(0), devices.size(), [this, &bvh_multi, &dscene, refit, &progress](size_t id) {
          /* Pointer translation is removed as it is not thread safe. Instead a new method is added
             to retrieve the real device pointer. */
          auto &sub = devices[id];

          if (!bvh_multi->sub_bvhs[id]) {
            BVHParams params = bvh_multi->params;
	    
	    params.bvh_layout = get_bvh_layout(sub->device.get(), bvh_multi->params.bvh_layout);

            /* Skip building a bottom level acceleration structure for non-instanced geometry on
             * Embree (since they are put into the top level directly, see bvh_embree.cpp) */
            if (!params.top_level && params.bvh_layout == BVH_LAYOUT_EMBREE &&
                !bvh_multi->geometry[0]->is_instanced())
	    {
            }
            else {
              bvh_multi->sub_bvhs[id] = std::unique_ptr<BVH>(BVH::create(
									 params, bvh_multi->geometry, bvh_multi->objects, sub->device.get()));
            }
          }
          if (bvh_multi->sub_bvhs[id]) {
            sub->device->build_bvh(bvh_multi->sub_bvhs[id].get(), dscene, progress, refit);
          }
        });
  }

  virtual void *get_cpu_osl_memory() override
  {
    /* Always return the OSL memory of the CPU device (this works since the constructor above
     * guarantees that CPU devices are always added to the back). */
    if (devices.size() > 1 && devices.back()->device->info.type != DEVICE_CPU) {
      return NULL;
    }

    return devices.back()->device->get_cpu_osl_memory();
  }

  bool is_resident(device_ptr key, Device *sub_device) override
  {
    foreach (auto &sub, devices) {
      if (sub->device.get() == sub_device) {
        return find_matching_mem_device(key, sub.get())->device.get() == sub_device;
      }
    }
    return false;
  }

  SubDevice *find_matching_mem_device(device_ptr key, SubDevice *sub)
  {
    assert(key != 0 &&
           (sub->peer_island_index >= 0 || sub->ptr_map.find(key) != sub->ptr_map.end()));

    /* Get the memory owner of this key (first try current device, then peer devices) */
    SubDevice *owner_sub = sub;
    if (owner_sub->ptr_map.find(key) == owner_sub->ptr_map.end()) {
      foreach (SubDevice *island_sub, peer_islands[sub->peer_island_index]) {
        if (island_sub != owner_sub && island_sub->ptr_map.find(key) != island_sub->ptr_map.end())
	{
          owner_sub = island_sub;
        }
      }
    }
    return owner_sub;
  }

  SubDevice *find_suitable_mem_device(device_ptr key, const vector<SubDevice *> &island)
  {
    assert(!island.empty());

    /* Get the memory owner of this key or the device with the lowest memory usage when new */
    SubDevice *owner_sub = island.front();
    foreach (SubDevice *island_sub, island) {
      if (key ? (island_sub->ptr_map.find(key) != island_sub->ptr_map.end()) :
                (island_sub->device->stats.mem_used < owner_sub->device->stats.mem_used))
      {
        owner_sub = island_sub;
      }
    }
    return owner_sub;
  }

  inline device_ptr find_matching_mem(device_ptr key, Device *dev) override
  {
    device_ptr ptr = 0;
    foreach (auto &sub, devices) {
      if (sub->device.get() == dev) {
        return find_matching_mem_device(key, sub.get())->ptr_map[key];
      }
    }
    return ptr;
  }

  inline device_ptr find_matching_mem(device_ptr key, SubDevice *sub)
  {
    return find_matching_mem_device(key, sub)->ptr_map[key];
  }

  void mem_alloc(device_memory &mem) override
  {
    device_ptr key = unique_key++;

    assert(mem.type == MEM_READ_ONLY || mem.type == MEM_READ_WRITE || mem.type == MEM_DEVICE_ONLY);
    /* The remaining memory types can be distributed across devices */
    foreach (const vector<SubDevice *> &island, peer_islands) {
      SubDevice *owner_sub = find_suitable_mem_device(key, island);
      mem.device = owner_sub->device.get();
      mem.device_pointer = 0;
      mem.device_size = 0;

      owner_sub->device->mem_alloc(mem);
      owner_sub->ptr_map[key] = mem.device_pointer;
    }

    mem.device = this;
    mem.device_pointer = key;
    stats.mem_alloc(mem.device_size);
  }

  void mem_copy_to(device_memory &mem, size_t size, size_t offset) override
  {
    device_ptr existing_key = mem.device_pointer;
    device_ptr key = (existing_key) ? existing_key : unique_key++;
    size_t existing_size = mem.device_size;

    /* The tile buffers are allocated on each device (see below), so copy to all of them */
    foreach (const vector<SubDevice *> &island, peer_islands) {
      SubDevice *owner_sub = find_suitable_mem_device(existing_key, island);
      mem.device = owner_sub->device.get();
      mem.device_pointer = (existing_key) ? owner_sub->ptr_map[existing_key] : 0;
      mem.device_size = existing_size;

      owner_sub->device->mem_copy_to(mem, size, offset);
      owner_sub->ptr_map[key] = mem.device_pointer;

      if (mem.type == MEM_GLOBAL || mem.type == MEM_TEXTURE) {
        /* Need to create texture objects and update pointer in kernel globals on all devices */
        foreach (SubDevice *island_sub, island) {
          if (island_sub != owner_sub) {
            island_sub->device->mem_copy_to(mem, size, offset);
          }
        }
      }
    }

    mem.device = this;
    mem.device_pointer = key;
    stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_copy_from(device_memory &mem, size_t y, size_t w, size_t h, size_t elem) override
  {
    device_ptr key = mem.device_pointer;
    size_t i = 0, sub_h = h / devices.size();

    foreach (auto &sub, devices) {
      size_t sy = y + i * sub_h;
      size_t sh = (i == (size_t)devices.size() - 1) ? h - sub_h * i : sub_h;

      SubDevice *owner_sub = find_matching_mem_device(key, sub.get());
      mem.device = owner_sub->device.get();
      mem.device_pointer = owner_sub->ptr_map[key];

      owner_sub->device->mem_copy_from(mem, sy, w, sh, elem);
      i++;
    }

    mem.device = this;
    mem.device_pointer = key;
  }

  void mem_zero(device_memory &mem) override
  {
    device_ptr existing_key = mem.device_pointer;
    device_ptr key = (existing_key) ? existing_key : unique_key++;
    size_t existing_size = mem.device_size;

    foreach (const vector<SubDevice *> &island, peer_islands) {
      SubDevice *owner_sub = find_suitable_mem_device(existing_key, island);
      mem.device = owner_sub->device.get();
      mem.device_pointer = (existing_key) ? owner_sub->ptr_map[existing_key] : 0;
      mem.device_size = existing_size;

      owner_sub->device->mem_zero(mem);
      owner_sub->ptr_map[key] = mem.device_pointer;
    }

    mem.device = this;
    mem.device_pointer = key;
    stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_free(device_memory &mem) override
  {
    device_ptr key = mem.device_pointer;
    /* key is zero if the pointer is NULL */
    if (key != 0) {
      size_t existing_size = mem.device_size;

      /* Free memory that was allocated for all devices (see above) on each device */
      foreach (const vector<SubDevice *> &island, peer_islands) {
        SubDevice *owner_sub = find_matching_mem_device(key, island.front());
        mem.device = owner_sub->device.get();
        mem.device_pointer = owner_sub->ptr_map[key];
        mem.device_size = existing_size;
        owner_sub->device->mem_free(mem);
        owner_sub->ptr_map.erase(owner_sub->ptr_map.find(key));

        if (mem.type == MEM_TEXTURE) {
          /* Free texture objects on all devices */
          foreach (SubDevice *island_sub, island) {
            if (island_sub != owner_sub) {
              island_sub->device->mem_free(mem);
            }
          }
        }
      }

      /* restore the device */
      mem.device = this;

      /* NULL the pointer and size and update the memory tracking */
      mem.device_pointer = 0;
      mem.device_size = 0;
      stats.mem_free(existing_size);
    }
  }
  
  void const_copy_to(const char *name, void *host, size_t size) override
  {
    foreach (auto &sub, devices)
      sub->device->const_copy_to(name, host, size);
  }

  int device_number(const Device *sub_device) const override
  {
    int i = 0;

    for (const auto &sub : devices) {
      if (sub->device.get() == sub_device)
        return i;
      i++;
    }

    return -1;
  }

  virtual void foreach_device(const function<void(Device *)> &callback) override
  {
    foreach (auto &sub, devices) {
      sub->device->foreach_device(callback);
    }
  }
};

Device *device_multi_create(const DeviceInfo &info, Stats &stats, Profiler &profiler)
{
  return new MultiDevice(info, stats, profiler);
}

CCL_NAMESPACE_END
