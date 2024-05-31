/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/* Based on https://github.com/jarikomppa/ipc (Unlicense) */

#include "BLI_subprocess.hh"
#include "BLI_assert.h"
#include "BLI_string_utf8.h"
#include <iostream>

#ifdef _WIN32

#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>

namespace blender {

bool Subprocess::init(Span<StringRefNull> args)
{
  BLI_assert(handle_ == nullptr);

  wchar_t path[MAX_PATH];
  if (!GetModuleFileNameW(nullptr, path, MAX_PATH)) {
    /* TODO: FormatMessage. */
    std::cerr << "Subprocess: Failed to get Blender path.\n";
    return false;
  }

  std::string args_str;
  for (StringRefNull arg : args) {
    args_str += " " + arg;
  }

  std::wstring w_args;
  w_args.resize(args_str.size(), L' ');
  BLI_strncpy_wchar_from_utf8(w_args.data(), args_str.c_str(), w_args.size() + 1);

  STARTUPINFOW startup_info = {0};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info = {0};
  if (!CreateProcessW(path,
                      w_args.data(),
                      nullptr,
                      nullptr,
                      false,
                      0,
                      nullptr,
                      nullptr,
                      &startup_info,
                      &process_info))
  {
    /* TODO: FormatMessage. */
    std::cerr << "Subprocess: Failed to create subprocess.\n";
    return false;
  }

  handle_ = process_info.hProcess;
  CloseHandle(process_info.hThread);

  return true;
}

Subprocess::~Subprocess()
{
  if (handle_) {
    CloseHandle(handle_);
  }
}

bool Subprocess::is_running()
{
  if (!handle_) {
    return false;
  }

  DWORD exit_code = 0;
  if (GetExitCodeProcess(handle_, &exit_code)) {
    return exit_code == STILL_ACTIVE;
  }
  /* Assume the process is still running. */
  return true;
}

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

}  // namespace blender

#else
#  include "BLI_vector.hh"
#  include <fcntl.h>
#  include <linux/limits.h>
#  include <stdlib.h>
#  include <sys/mman.h>
#  include <sys/prctl.h>
#  include <sys/stat.h>
#  include <unistd.h>
#  include <wait.h>

namespace blender {

bool Subprocess::init(Span<StringRefNull> args)
{
  char path[PATH_MAX];
  size_t len = readlink("/proc/self/exe", path, PATH_MAX);
  if (len == -1) {
    return false;
  }
  /* readlink doesn't append a null terminator. */
  path[len] = '\0';

  Vector<char *> char_args;
  for (StringRefNull arg : args) {
    char_args.append((char *)arg.data());
  }
  char_args.append(nullptr);

  pid_ = fork();
  if (pid_ < 0) {
    return false;
  }
  else if (pid_ > 0) {
    return true;
  }

  /* Prevent this child process from outliving its parent THREAD. */
  if (prctl(PR_SET_PDEATHSIG, SIGHUP) == -1) {
    perror("Subprocess setup failed: ");
  }

  /* Child process initialization. */
  execv(path, char_args.data());

  perror("Subprocess creation failed: ");
  BLI_assert_unreachable();
  exit(errno);

  return false;
}

Subprocess::~Subprocess() {}

bool Subprocess::is_running()
{
  if (pid_ == 0) {
    return false;
  }

  pid_t result = waitpid(pid_, nullptr, WNOHANG);
  if (result == pid_) {
    pid_ = 0;
    return false;
  }
  else if (result == -1) {
    perror("Subprocess check failed: ");
  }

  return true;
}

SharedMemory::SharedMemory(std::string name, size_t size, bool already_exists)
    : name_(name), is_owner_(!already_exists)
{
  if (already_exists) {
    handle_ = shm_open(name.c_str(), O_RDWR, 0755);
  }
  else {
    handle_ = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0755);
    if (handle_) {
      ftruncate(handle_, size);
    }
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

}  // namespace blender

#endif
