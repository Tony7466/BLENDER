/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#include "BLI_sys_types.h"
#include "BLI_utility_mixins.hh"
#include <string>

#ifdef _WIN32
typedef void *HANDLE;
#else
struct sem_t;
#endif

namespace blender::gpu {

class SharedMemory : NonCopyable {
 private:
  std::string name_;
#ifdef _WIN32
  HANDLE handle_;
#else
  int handle_;
#endif
  void *data_;
  size_t data_size_;
  bool is_owner_;

 public:
  SharedMemory(std::string name, size_t size, bool already_exists);
  ~SharedMemory();

  void *get_data()
  {
    return data_;
  }

  size_t get_size()
  {
    return data_size_;
  }
};

class SharedSemaphore : NonCopyable {
 private:
  std::string name_;
#if defined(_WIN32)
  HANDLE handle_;
#else
  sem_t *handle_;
#endif

 public:
  SharedSemaphore(std::string name);
  ~SharedSemaphore();

  void increment();
  void decrement();
  bool try_decrement();
};

}  // namespace blender::gpu
