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
#include "util/tbb.h"
#include "util/thread.h"
#include "util/time.h"

CCL_NAMESPACE_BEGIN

// TODO: properly integrate this ino device/memory.h somehow
// TODO: this is incorrect for host memory fallback, the shared pointer
// should be shared across clones and mutex protected
class device_memory_clone : public device_texture {
 public:
  device_memory_clone(const device_memory &mem, Device *sub_device, device_ptr sub_device_pointer)
      : device_texture(sub_device,
                       mem.name,
                       0,
                       IMAGE_DATA_TYPE_FLOAT,
                       INTERPOLATION_NONE,
                       EXTENSION_REPEAT)  // mem.type)
  {
    data_type = mem.data_type;
    data_elements = mem.data_elements;
    data_size = mem.data_size;
    device_size = mem.device_size;
    data_width = mem.data_width;
    data_height = mem.data_height;
    data_depth = mem.data_depth;
    type = mem.type;
    name = mem.name;

    /* Pointers. */
    device = sub_device;
    device_pointer = sub_device_pointer;

    host_pointer = mem.host_pointer;
    shared_pointer = mem.shared_pointer;
    /* reference counter for shared_pointer */
    shared_counter = mem.shared_counter;
    modified = mem.modified;

    if (type == MEM_TEXTURE) {
      const device_texture *p_tex = static_cast<const device_texture *>(&mem);
      memcpy(&info, &(p_tex->info), sizeof(TextureInfo));
      slot = p_tex->slot;
    }
  }

  ~device_memory_clone()
  {
    // Don't free anything
    host_pointer = 0;
    device_pointer = 0;
  }
};

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
   * device pointer when performing memory operations. */
  struct MemoryAlloc {
    SubDevice *owner = nullptr;
    device_ptr device_pointer = 0;
  };

  struct MemoryEntry {
    MemoryEntry(device_memory &mem, size_t num_islands) : mem(mem), allocations(num_islands) {}

    device_memory &mem;
    vector<MemoryAlloc> allocations;

    bool need_alloc = false;
    bool need_copy = false;
    bool need_zero = false;
    size_t copy_size = 0, copy_offset = 0;
  };

  map<device_ptr, MemoryEntry> memory_map;
  thread_mutex memory_map_mutex;
  device_ptr unique_key = 1;
  bool queued_memory_operation = false;

  /* Queued BVH build operations. */
  struct BVHBuildOperation {
    BVHBuildOperation(BVH *bvh, bool refit) : bvh(bvh), refit(refit) {}

    BVH *bvh;
    bool refit;
  };
  vector<BVHBuildOperation> bvh_bottom_level_build_queue;
  vector<BVHBuildOperation> bvh_top_level_build_queue;
  thread_mutex bvh_mutex;

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
    flush_memory_operations();

    foreach (SubDevice &sub, devices)
      if (!sub.device->load_kernels(kernel_features))
        return false;

    return true;
  }

  bool load_osl_kernels() override
  {
    flush_memory_operations();

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

    // TODO: should not be needed
    // BVHMulti *const bvh_multi = static_cast<BVHMulti *>(bvh);
    // bvh_multi->sub_bvhs.resize(devices.size());

    thread_scoped_lock lock(bvh_mutex);
    if (bvh->params.top_level) {
      bvh_top_level_build_queue.push_back(BVHBuildOperation(bvh, refit));
    }
    else {
      bvh_bottom_level_build_queue.push_back(BVHBuildOperation(bvh, refit));
    }
  }

  void build_bvh_(BVH *bvh, Progress &progress, const bool refit, const size_t device_index)
  {
    BVHMulti *const bvh_multi = static_cast<BVHMulti *>(bvh);
    SubDevice &sub = devices[device_index];

    if (!bvh_multi->sub_bvhs[device_index]) {
      BVHParams params = bvh_multi->params;
      params.bvh_layout = get_bvh_layout(sub.device.get(), bvh_multi->params.bvh_layout);

      /* Skip building a bottom level acceleration structure for non-instanced geometry on
       * Embree (since they are put into the top level directly, see bvh_embree.cpp) */
      if (!params.top_level && params.bvh_layout == BVH_LAYOUT_EMBREE &&
          !bvh_multi->geometry[0]->is_instanced())
      {
      }
      else {
        bvh_multi->sub_bvhs[device_index] = std::unique_ptr<BVH>(
            BVH::create(params, bvh_multi->geometry, bvh_multi->objects, sub.device.get()));
      }
    }
    if (bvh_multi->sub_bvhs[device_index]) {
      sub.device->build_bvh(bvh_multi->sub_bvhs[device_index].get(), progress, refit);
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
      SubDevice *owner = memory_map.at(key).allocations[peer_island_index].owner;
      if (owner) {
        return *owner;
      }
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
    thread_scoped_lock lock(memory_map_mutex);
    auto it = memory_map.find(key);
    if (it != memory_map.end()) {
      assert(&it->second.mem == &mem);
      return it->second;
    }

    return memory_map.insert({key, MemoryEntry(mem, peer_islands.size())}).first->second;
  }

  void mem_alloc(device_memory &mem) override
  {
    device_ptr key = unique_key++;
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    /* Only these types can be handled by multi device, others need to be done for individual
     * devices. */
    assert(mem.type == MEM_READ_ONLY || mem.type == MEM_READ_WRITE || mem.type == MEM_DEVICE_ONLY);

    memory_entry.need_alloc = true;
    assert(!memory_entry.need_copy);
    assert(!memory_entry.need_zero);
    queued_memory_operation = true;

    mem.device_pointer = key;
  }

  void mem_alloc_(device_ptr key, MemoryEntry &memory_entry, size_t peer_island_index)
  {
    SubDevice &owner_sub = find_suitable_mem_device(key, peer_island_index);

    device_memory &mem = memory_entry.mem;
    device_memory_clone sub_mem(mem, owner_sub.device.get(), 0);
    sub_mem.device_size = 0;

    owner_sub.device->mem_alloc(sub_mem);

    MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
    alloc.device_pointer = sub_mem.device_pointer;
    alloc.owner = &owner_sub;

    // TODO: do just once: stats.mem_alloc(mem.device_size);
  }

  void mem_copy_to(device_memory &mem, size_t size, size_t offset) override
  {
    device_ptr existing_key = mem.device_pointer;
    device_ptr key = (existing_key) ? existing_key : unique_key++;
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    const size_t new_start = min(memory_entry.copy_offset, offset);
    const size_t new_end = max(memory_entry.copy_offset + memory_entry.copy_size, offset + size);

    memory_entry.need_copy = true;
    memory_entry.copy_offset = new_start;
    memory_entry.copy_size = new_end - new_start;
    queued_memory_operation = true;

    mem.device_pointer = key;
  }

  void mem_copy_to_(device_ptr key, MemoryEntry &memory_entry, size_t peer_island_index)
  {
    device_memory &mem = memory_entry.mem;
    const size_t size = memory_entry.copy_size;
    const size_t offset = memory_entry.copy_offset;

    /* The tile buffers are allocated on each device (see below), so copy to all of them */
    MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
    SubDevice &owner_sub = find_suitable_mem_device(key, peer_island_index);

    device_memory_clone sub_mem(mem, owner_sub.device.get(), alloc.device_pointer);
    owner_sub.device->mem_copy_to(sub_mem, size, offset);

    alloc.device_pointer = sub_mem.device_pointer;
    alloc.owner = &owner_sub;

    if (mem.type == MEM_GLOBAL || mem.type == MEM_TEXTURE) {
      /* Need to create texture objects and update pointer in kernel globals on all devices */
      foreach (SubDevice *island_sub, peer_islands[peer_island_index]) {
        if (island_sub != &owner_sub) {
          island_sub->device->mem_copy_to(sub_mem, size, offset);
        }
      }
    }

    // TODO: do just once. stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_copy_from(device_memory &mem) override
  {
    assert(!"mem_copy_form not support for multi device");
  }

  void mem_zero(device_memory &mem) override
  {
    device_ptr existing_key = mem.device_pointer;
    device_ptr key = (existing_key) ? existing_key : unique_key++;
    MemoryEntry &memory_entry = ensure_memory_entry(key, mem);

    memory_entry.need_zero = true;
    memory_entry.need_copy = false;
    queued_memory_operation = true;

    mem.device_pointer = key;
  }

  void mem_zero_(device_ptr key, MemoryEntry &memory_entry, size_t peer_island_index)
  {
    device_memory &mem = memory_entry.mem;

    MemoryAlloc &alloc = memory_entry.allocations[peer_island_index];
    SubDevice &owner_sub = find_suitable_mem_device(key, peer_island_index);

    device_memory_clone sub_mem(mem, owner_sub.device.get(), alloc.device_pointer);

    owner_sub.device->mem_zero(mem);

    alloc.device_pointer = mem.device_pointer;
    alloc.owner = &owner_sub;

    // TODO: do just once. stats.mem_alloc(mem.device_size - existing_size);
  }

  void mem_free(device_memory &mem) override
  {
    device_ptr key = mem.device_pointer;
    /* Key is zero if the pointer is NULL. */
    if (key == 0) {
      return;
    }

    // TODO: postpone and multithread memory free as well?

    thread_scoped_lock lock(memory_map_mutex);
    MemoryEntry &entry = memory_map.at(key);
    memory_map.erase(key);
    lock.unlock();

    size_t existing_size = mem.device_size;

    /* Free memory that was allocated for all devices (see above) on each device */
    for (size_t peer_island_index = 0; peer_island_index < peer_islands.size();
         peer_island_index++) {
      MemoryAlloc &alloc = entry.allocations[peer_island_index];

      if (alloc.owner) {
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
    }

    /* Restore the device. */
    mem.device = this;

    /* NULL the pointer and size and update the memory tracking. */
    mem.device_pointer = 0;
    mem.device_size = 0;
    // TODO: fix other places before re-enabling this. stats.mem_free(existing_size);
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

  void flush_memory_operations()
  {
    if (!queued_memory_operation) {
      return;
    }

    parallel_for(size_t(0), peer_islands.size(), [&](const size_t peer_island_index) {
      for (auto &it : memory_map) {
        device_ptr key = it.first;
        MemoryEntry &memory_entry = it.second;

        if (memory_entry.need_alloc) {
          mem_alloc_(key, memory_entry, peer_island_index);
        }
        if (memory_entry.need_zero) {
          mem_zero_(key, memory_entry, peer_island_index);
        }
        if (memory_entry.need_copy) {
          mem_copy_to_(key, memory_entry, peer_island_index);
        }
      }
    });

    for (auto &it : memory_map) {
      MemoryEntry &memory_entry = it.second;

      memory_entry.need_alloc = false;
      memory_entry.need_copy = false;
      memory_entry.need_zero = false;
      memory_entry.copy_size = 0;
      memory_entry.copy_offset = 0;
    }

    queued_memory_operation = false;
  }

  void flush_bvh_operations(Progress &progress)
  {
    if (bvh_bottom_level_build_queue.empty() && bvh_top_level_build_queue.empty()) {
      return;
    }

    parallel_for(size_t(0), devices.size(), [&](const size_t device_index) {
      /* Bottom level BVHs first, since top level needs them. */
      for (BVHBuildOperation &operation : bvh_bottom_level_build_queue) {
        build_bvh_(operation.bvh, progress, operation.refit, device_index);
      }

      for (BVHBuildOperation &operation : bvh_top_level_build_queue) {
        build_bvh_(operation.bvh, progress, operation.refit, device_index);
      }
    });

    bvh_bottom_level_build_queue.clear();
    bvh_top_level_build_queue.clear();
  }

  void flush_operations(Progress &progress) override
  {
    flush_memory_operations();
    flush_bvh_operations(progress);
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
