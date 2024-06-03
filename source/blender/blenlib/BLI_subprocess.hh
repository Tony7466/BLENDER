/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

#if defined(_WIN32) || defined(__linux__)
/* The Subprocess API is only supported on Windows and Linux. */
#  define BLI_SUBPROCESS_SUPPORT
#endif

#ifdef BLI_SUBPROCESS_SUPPORT

#  include "BLI_span.hh"
#  include "BLI_string_ref.hh"
#  include "BLI_sys_types.h"
#  include "BLI_utility_mixins.hh"
#  include <string>

#  ifdef _WIN32
typedef void *HANDLE;
#  else
#    include <semaphore.h>
#  endif

namespace blender {

class Subprocess : NonCopyable {
 private:
#  ifdef _WIN32
  HANDLE handle_ = nullptr;
#  else
  pid_t pid_ = 0;
#  endif
 public:
  ~Subprocess();

  bool create(Span<StringRefNull> args);
  bool is_running();
};

class SharedMemory : NonCopyable {
 private:
  std::string name_;
#  ifdef _WIN32
  HANDLE handle_;
#  else
  int handle_;
#  endif
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
#  if defined(_WIN32)
  HANDLE handle_;
#  else
  sem_t *handle_;
#  endif
  bool is_owner_;

 public:
  SharedSemaphore(std::string name, bool is_owner);
  ~SharedSemaphore();

  void increment();
  void decrement();
  bool try_decrement(int wait_ms = 0);
};

}  // namespace blender

#endif
