/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

#include "device/multi/device.h"

#include <sstream>
#include <stdlib.h>

#include "bvh/multi.h"

#include "device/device.h"
#include "device/queue.h"

#include "scene/geometry.h"

#include "util/foreach.h"
#include "util/log.h"
#include "util/map.h"
#include "util/time.h"

CCL_NAMESPACE_BEGIN

class MultiDevice : public Device {
 public:
  /* Multiple devices used for rendering. */
  struct SubDevice {
    Stats stats;
    unique_ptr<Device> device;
    int peer_island_index = -1;
  };
  vector<SubDevice> devices;

  /* Peer islands for sharing memory allocations between devices. Every
   * memory allocation is done only once for all devices in one island. */
  vector<vector<SubDevice *>> peer_islands;

  /* Map that tracks all memory allocations performed for this multi device.
   * The key is fake device pointer, which is mapped to the actual per island
   * pointer when performing memory operations. */
  struct MemoryAlloc {
    SubDevice *owner = nullptr;
    device_ptr device_pointer = 0;
  };

  struct MemoryEntry {
    MemoryEntry(device_memory &mem, size_t num_islands) : mem(mem), allocations(num_islands) {}

    device_memory &mem;
    vector<MemoryAlloc> allocations;
  };

  map<device_ptr, MemoryEntry> memory_map;
  device_ptr unique_key = 1;

  MultiDevice(const DeviceInfo &info, Stats &stats, Profiler &profiler)
      : Device(info, stats, profiler)
  {
    devices.resize(info.multi_devices.size());

    int device_index = 0;
    foreach (const DeviceInfo &subinfo, info.multi_devices) {
      /* Always add CPU devices at the back since GPU devices can change
       * host memory pointers, which CPU uses as device pointer. */
      SubDevice &sub = (subinfo.type == DEVICE_CPU) ? devices[devices.size() - 1] :
                                                      devices[device_index++];
      sub.device.reset(Device::create(subinfo, sub.stats, profiler));
    }

    /* Build a list of peer islands for the available render devices */
    foreach (SubDevice &sub, devices) {
      /* First ensure that every device is in at least once peer island */
      if (sub.peer_island_index < 0) {
        peer_islands.emplace_back();
        sub.peer_island_index = (int)peer_islands.size() - 1;
        peer_islands[sub.peer_island_index].push_back(&sub);
      }

      if (!info.has_peer_memory) {
        continue;
      }

      /* Second check peer access between devices and fill up the islands accordingly */
      foreach (SubDevice &peer_sub, devices) {
        if (peer_sub.peer_island_index < 0 &&
            peer_sub.device->info.type == sub.device->info.type &&
            peer_sub.device->check_peer_access(sub.device.get()))
        {
          peer_sub.peer_island_index = sub.peer_island_index;
          peer_islands[sub.peer_island_index].push_back(&peer_sub);
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

    foreach (SubDevice &sub, devices)
      error_msg += sub.device->error_message();

    return error_msg;
  }

  virtual BVHLayoutMask get_bvh_layout_mask(uint kernel_features) const override
  {
    BVHLayoutMask bvh_layout_mask = BVH_LAYOUT_ALL;
    BVHLayoutMask bvh_layout_mask_all = BVH_LAYOUT_NONE;
    foreach (const SubDevice &sub_device, devices) {
      BVHLayoutMask device_bvh_layout_mask = sub_device.device->get_bvh_layout_mask(
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

    return bvh_layout_mask;
  }

  bool load_kernels(const uint kernel_features) override
  {
    foreach (SubDevice &sub, devices)
      if (!sub.device->load_kernels(kernel_features))
        return false;

    return true;
  }

  bool load_osl_kernels() override
  {
    foreach (SubDevice &sub, devices)
      if (!sub.device->load_osl_kernels())
        return false;

    return true;
  }

  void build_bvh(BVH *bvh, Progress &progress, bool refit) override
  {
    /* Try to build and share a single acceleration structure, if possible */
    if (bvh->params.bvh_layout == BVH_LAYOUT_BVH2 || bvh->params.bvh_layout == BVH_LAYOUT_EMBREE) {
      devices.back().device->build_bvh(bvh, progress, refit);
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

    vector<BVHMulti *> geom_bvhs;
    geom_bvhs.reserve(bvh->geometry.size());
    foreach (Geometry *geom, bvh->geometry) {
      geom_bvhs.push_back(static_cast<BVHMulti *>(geom->bvh));
    }

    /* Broadcast acceleration structure build to all render devices */
    size_t i = 0;
    foreach (SubDevice &sub, devices) {
      /* Change geometry BVH pointers to the sub BVH */
      for (size_t k = 0; k < bvh->geometry.size(); ++k) {
        bvh->geometry[k]->bvh = geom_bvhs[k]->sub_bvhs[i];
      }

      if (!bvh_multi->sub_bvhs[i]) {
        BVHParams params = bvh->params;
        if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX)
          params.bvh_layout = BVH_LAYOUT_OPTIX;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL)
          params.bvh_layout = BVH_LAYOUT_METAL;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_HIPRT)
          params.bvh_layout = BVH_LAYOUT_HIPRT;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_EMBREEGPU)
          params.bvh_layout = BVH_LAYOUT_EMBREEGPU;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_OPTIX_EMBREE)
          params.bvh_layout = sub.device->info.type == DEVICE_OPTIX ? BVH_LAYOUT_OPTIX :
                                                                      BVH_LAYOUT_EMBREE;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_METAL_EMBREE)
          params.bvh_layout = sub.device->info.type == DEVICE_METAL ? BVH_LAYOUT_METAL :
                                                                      BVH_LAYOUT_EMBREE;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_HIPRT_EMBREE)
          params.bvh_layout = sub.device->info.type == DEVICE_HIPRT ? BVH_LAYOUT_HIPRT :
                                                                      BVH_LAYOUT_EMBREE;
        else if (bvh->params.bvh_layout == BVH_LAYOUT_MULTI_EMBREEGPU_EMBREE)
          params.bvh_layout = sub.device->info.type == DEVICE_ONEAPI ? BVH_LAYOUT_EMBREEGPU :
                                                                       BVH_LAYOUT_EMBREE;
        /* Skip building a bottom level acceleration structure for non-instanced geometry on Embree
         * (since they are put into the top level directly, see bvh_embree.cpp) */
        if (!params.top_level && params.bvh_layout == BVH_LAYOUT_EMBREE &&
            !bvh->geometry[0]->is_instanced())
        {
          i++;
          continue;
        }

        bvh_multi->sub_bvhs[i] = BVH::create(
            params, bvh->geometry, bvh->objects, sub.device.get());
      }

      sub.device->build_bvh(bvh_multi->sub_bvhs[i], progress, refit);
      i++;
    }

    /* Change geometry BVH pointers back to the multi BVH. */
    for (size_t k = 0; k < bvh->geometry.size(); ++k) {
      bvh->geometry[k]->bvh = geom_bvhs[k];
    }
  }

  virtual void *get_cpu_osl_memory() override
  {
    /* Always return the OSL memory of the CPU device (this works since the constructor above
     * guarantees that CPU devices are always added to the back). */
    if (devices.size() > 1 && devices.back().device->info.type != DEVICE_CPU) {
      return NULL;
    }
    return devices.back().device->get_cpu_osl_memory();
  }

  bool is_resident(device_ptr key, Device *sub_device) override
  {
    foreach (SubDevice &sub, devices) {
      if (sub.device.get() == sub_device) {
        return find_matching_mem_device(key, sub).device.get() == sub_device;
      }
    }
    return false;
  }

  SubDevice &find_matching_mem_device(device_ptr key, SubDevice &sub)
  {
    assert(key != 0);
    return *(memory_map.at(key).allocations[sub.peer_island_index].owner);
  }

  SubDevice &find_suitable_mem_device(device_ptr key, const int peer_island_index)
  {
    /* Already allocated, get owner. */
    if (key) {
      return *(memory_map.at(key).allocations[peer_island_index].owner);
    }

    /* Not allocated yet, find device with lowest memory usage. */
    vector<SubDevice *> &island = peer_islands[peer_island_index];
    assert(!island.empty());

    SubDevice *owner_sub = island.front();
    foreach (SubDevice *island_sub, island) {
      if (island_sub->device->stats.mem_used < owner_sub->device->stats.mem_used) {
        owner_sub = island_sub;
      }
    }
    return *owner_sub;
  }

  inline device_ptr find_matching_mem(device_ptr key, Device *dev) override
  {
    foreach (SubDevice &sub, devices) {
      if (sub.device.get() == dev) {
        return find_matching_mem(key, sub);
      }
    }
    return 0;
  }

  inline device_ptr find_matching_mem(device_ptr key, SubDevice &sub)
  {
    assert(key != 0);
    return memory_map.at(key).allocations[sub.peer_island_index].device_pointer;
  }

  MemoryEntry &ensure_memory_entry(device_ptr key, device_memory &mem)
  {
    auto it = memory_map.find(key);
    if (it != memory_map.end()) {
      assert(&memory_entry.mem == &mem);
      return it->second;
    }

    return memory_map.insert({key, MemoryEntry(mem, peer_islands.size())}).first->second;
  }

  void mem_alloc(device_memory &mem) override
  {
    device_ptr key = unique_key++;
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    assert(mem.type == MEM_READ_ONLY || mem.type == MEM_READ_WRITE || mem.type == MEM_DEVICE_ONLY);
    /* The remaining memory types can be distributed across devices */
    for (size_t peer_island_index = 0; peer_island_index < peer_islands.size();
         peer_island_index++) {
      SubDevice &owner_sub = find_suitable_mem_device(key, peer_island_index);
      mem.device = owner_sub.device.get();
      mem.device_pointer = 0;
      mem.device_size = 0;

      owner_sub.device->mem_alloc(mem);

      MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
      alloc.device_pointer = mem.device_pointer;
      alloc.owner = &owner_sub;
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
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    /* The tile buffers are allocated on each device (see below), so copy to all of them */
    for (size_t peer_island_index = 0; peer_island_index < peer_islands.size();
         peer_island_index++) {
      MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
      SubDevice &owner_sub = find_suitable_mem_device(existing_key, peer_island_index);
      mem.device = owner_sub.device.get();
      mem.device_pointer = alloc.device_pointer;
      mem.device_size = existing_size;

      owner_sub.device->mem_copy_to(mem, size, offset);

      alloc.device_pointer = mem.device_pointer;
      alloc.owner = &owner_sub;

      if (mem.type == MEM_GLOBAL || mem.type == MEM_TEXTURE) {
        /* Need to create texture objects and update pointer in kernel globals on all devices */
        foreach (SubDevice *island_sub, peer_islands[peer_island_index]) {
          if (island_sub != &owner_sub) {
            island_sub->device->mem_copy_to(mem, size, offset);
          }
        }
      }
    }

    mem.device = this;
    mem.device_pointer = key;
    stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_copy_from(device_memory &mem) override
  {
    assert(!"mem_copy_form not support for multi device");
  }

  void mem_zero(device_memory &mem) override
  {
    device_ptr existing_key = mem.device_pointer;
    device_ptr key = (existing_key) ? existing_key : unique_key++;
    size_t existing_size = mem.device_size;
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    for (size_t peer_island_index = 0; peer_island_index < peer_islands.size();
         peer_island_index++) {
      MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
      SubDevice &owner_sub = find_suitable_mem_device(existing_key, peer_island_index);
      mem.device = owner_sub.device.get();
      mem.device_pointer = alloc.device_pointer;
      mem.device_size = existing_size;

      owner_sub.device->mem_zero(mem);

      alloc.device_pointer = mem.device_pointer;
      alloc.owner = &owner_sub;
    }

    mem.device = this;
    mem.device_pointer = key;
    stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_free(device_memory &mem) override
  {
    device_ptr key = mem.device_pointer;
    /* Key is zero if the pointer is NULL. */
    if (key == 0) {
      return;
    }

    size_t existing_size = mem.device_size;
    MemoryEntry &entry = memory_map.at(key);

    /* Free memory that was allocated for all devices (see above) on each device */
    for (size_t peer_island_index = 0; peer_island_index < peer_islands.size();
         peer_island_index++) {
      MemoryAlloc &alloc = entry.allocations[peer_island_index];
      SubDevice &owner_sub = *alloc.owner;
      mem.device = owner_sub.device.get();
      mem.device_pointer = alloc.device_pointer;
      mem.device_size = existing_size;

      owner_sub.device->mem_free(mem);

      if (mem.type == MEM_TEXTURE) {
        /* Free texture objects on all devices */
        foreach (SubDevice *island_sub, peer_islands[peer_island_index]) {
          if (island_sub != &owner_sub) {
            island_sub->device->mem_free(mem);
          }
        }
      }
    }

    memory_map.erase(key);

    /* Restore the device. */
    mem.device = this;

    /* NULL the pointer and size and update the memory tracking. */
    mem.device_pointer = 0;
    mem.device_size = 0;
    stats.mem_free(existing_size);
  }

  void const_copy_to(const char *name, void *host, size_t size) override
  {
    foreach (SubDevice &sub, devices)
      sub.device->const_copy_to(name, host, size);
  }

  int device_number(const Device *sub_device) const override
  {
    int i = 0;

    foreach (const SubDevice &sub, devices) {
      if (sub.device.get() == sub_device)
        return i;
      i++;
    }

    return -1;
  }

  virtual void foreach_device(const function<void(Device *)> &callback) override
  {
    foreach (SubDevice &sub, devices) {
      sub.device->foreach_device(callback);
    }
  }
};

Device *device_multi_create(const DeviceInfo &info, Stats &stats, Profiler &profiler)
{
  return new MultiDevice(info, stats, profiler);
}

CCL_NAMESPACE_END
