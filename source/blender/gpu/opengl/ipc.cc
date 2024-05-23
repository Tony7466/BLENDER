/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Based on https://github.com/jarikomppa/ipc (Unlicense) */

#include "ipc.hh"
#include "BLI_assert.h"

#ifdef _WIN32
#  include <windows.h>

namespace blender::gpu {

SharedMemory::SharedMemory(std::string name, size_t size, bool already_exists)
    : name_(name), is_owner_(!already_exists)
{
  if (already_exists) {
    handle_ = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, name.c_str());
  }
  else {
    handle_ = CreateFileMappingA(
        INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, name.c_str());
  }

  if (handle_) {
    data_ = MapViewOfFile(handle_, FILE_MAP_ALL_ACCESS, 0, 0, size);
  }
  else {
    data_ = nullptr;
  }

  data_size_ = data_ ? size : 0;
}

SharedMemory::~SharedMemory()
{
  if (data_) {
    UnmapViewOfFile(data_);
  }
  if (handle_) {
    CloseHandle(handle_);
  }
}

SharedSemaphore::SharedSemaphore(std::string name) : name_(name)
{
  handle_ = CreateSemaphoreA(NULL, 0, 1, name.c_str());
}

SharedSemaphore::~SharedSemaphore()
{
  if (handle_) {
    CloseHandle(handle_);
  }
}

void SharedSemaphore::increment()
{
  ReleaseSemaphore(handle_, 1, NULL);
}

void SharedSemaphore::decrement()
{
  WaitForSingleObject(handle_, INFINITE);
}

bool SharedSemaphore::try_decrement()
{
  return WaitForSingleObject(handle_, 0) == WAIT_OBJECT_0;
}

}  // namespace blender::gpu

#else
#  include <fcntl.h>
#  include <stdlib.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <unistd.h>

namespace blender::gpu {

SharedMemory::SharedMemory(std::string name, size_t size, bool already_exists)
    : name_(name), is_owner_(!already_exists)
{
  if (already_exists) {
    handle_ = shm_open(name.c_str(), O_RDWR, 0755);
  }
  else {
    handle_ = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0755);
  }

  if (handle_) {
    data_ = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, handle_, 0);
  }
  else {
    data_ = nullptr;
  }

  data_size_ = data_ ? size : 0;

  /* File descriptor can close after mmap. */
  close(handle_);
}

SharedMemory::~SharedMemory()
{
  if (data_) {
    munmap(data_, data_size_);
    if (is_owner_) {
      shm_unlink(name_.c_str());
    }
  }
}

SharedSemaphore::SharedSemaphore(std::string name) : name_(name)
{
  handle_ = sem_open(name.c_str(), O_CREAT, 0700, 0);
}

SharedSemaphore::~SharedSemaphore()
{
  if (handle_) {
    sem_close(handle_);
    sem_unlink(name_.c_str());
  }
}

void SharedSemaphore::increment()
{
  sem_post(handle_);
}

void SharedSemaphore::decrement()
{
  sem_wait(handle_);
}

bool SharedSemaphore::try_decrement()
{
  return sem_trywait(handle_) == 0;
}

}  // namespace blender::gpu

#endif
