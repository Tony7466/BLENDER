/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#include "BLI_subprocess.hh"

#ifdef BLI_SUBPROCESS_SUPPORT

/* Based on https://github.com/jarikomppa/ipc (Unlicense) */

#  include "BLI_assert.h"
#  include "BLI_string_utf8.h"
#  include <iostream>

#  ifdef _WIN32

#    define WIN32_LEAN_AND_MEAN
#    include <comdef.h>
#    include <windows.h>

namespace blender {

static void print_last_error(const char *msg)
{
  DWORD error_code = GetLastError();
  std::cerr << "ERROR (" << error_code << "): " << msg << std::endl;
}

static bool check(bool result, const char *msg)
{
  if (!result) {
    print_last_error(msg);
    BLI_assert(false);
  }
  return result;
}

#    define CHECK(result) check((result), __FUNCTION__ " : " #result)

bool Subprocess::create(Span<StringRefNull> args)
{
  BLI_assert(handle_ == nullptr);

  constexpr size_t max_path = 1024;
  wchar_t path[max_path];
  if (!CHECK(GetModuleFileNameW(nullptr, path, max_path))) {
    return false;
  }

  std::string args_str;
  for (StringRefNull arg : args) {
    args_str += arg + " ";
  }

  const int length_wc = MultiByteToWideChar(
      CP_UTF8, 0, args_str.c_str(), args_str.length(), NULL, 0);
  std::wstring w_args(length_wc, 0);
  CHECK(MultiByteToWideChar(
      CP_UTF8, 0, args_str.c_str(), args_str.length(), w_args.data(), length_wc));

  STARTUPINFOW startup_info = {0};
  startup_info.cb = sizeof(startup_info);
  PROCESS_INFORMATION process_info = {0};
  if (!CHECK(CreateProcessW(path,
                            w_args.data(),
                            nullptr,
                            nullptr,
                            false,
                            0,
                            nullptr,
                            nullptr,
                            &startup_info,
                            &process_info)))
  {
    return false;
  }

  handle_ = process_info.hProcess;
  CHECK(CloseHandle(process_info.hThread));

  return true;
}

Subprocess::~Subprocess()
{
  if (handle_) {
    CHECK(CloseHandle(handle_));
  }
}

bool Subprocess::is_running()
{
  if (!handle_) {
    return false;
  }

  DWORD exit_code = 0;
  if (CHECK(GetExitCodeProcess(handle_, &exit_code))) {
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
    CHECK(handle_ /*Open*/);
  }
  else {
    handle_ = CreateFileMappingA(
        INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, size, name.c_str());
    CHECK(handle_ /*Close*/);
  }

  if (handle_) {
    data_ = MapViewOfFile(handle_, FILE_MAP_ALL_ACCESS, 0, 0, size);
    CHECK(data_);
  }
  else {
    data_ = nullptr;
  }

  data_size_ = data_ ? size : 0;
}

SharedMemory::~SharedMemory()
{
  if (data_) {
    CHECK(UnmapViewOfFile(data_));
  }
  if (handle_) {
    CHECK(CloseHandle(handle_));
  }
}

SharedSemaphore::SharedSemaphore(std::string name, bool is_owner)
    : name_(name), is_owner_(is_owner)
{
  handle_ = CreateSemaphoreA(NULL, 0, 1, name.c_str());
  CHECK(handle_);
}

SharedSemaphore::~SharedSemaphore()
{
  if (handle_) {
    CHECK(CloseHandle(handle_));
  }
}

void SharedSemaphore::increment()
{
  CHECK(ReleaseSemaphore(handle_, 1, NULL));
}

void SharedSemaphore::decrement()
{
  CHECK(WaitForSingleObject(handle_, INFINITE) != WAIT_FAILED);
}

bool SharedSemaphore::try_decrement(int wait_ms)
{
  DWORD result = WaitForSingleObject(handle_, wait_ms);
  CHECK(result != WAIT_FAILED);
  return result == WAIT_OBJECT_0;
}

}  // namespace blender

#  elif defined(__linux__)

#    include "BLI_time.h"
#    include "BLI_vector.hh"
#    include <fcntl.h>
#    include <linux/limits.h>
#    include <stdlib.h>
#    include <sys/mman.h>
#    include <sys/stat.h>
#    include <unistd.h>
#    include <wait.h>

namespace blender {

static void print_last_error(const char *function, const char *msg)
{
  int error_code = errno;
  std::string error_msg = "ERROR (" + std::to_string(error_code) + "): " + function + " : " + msg;
  perror(error_msg.c_str());
}

static bool check(int result, const char *function, const char *msg)
{
  if (result == -1) {
    print_last_error(function, msg);
    BLI_assert(false);
    return false;
  }
  return true;
}

#    define CHECK(result) check((result), __FUNCTION__, #result)

bool Subprocess::create(Span<StringRefNull> args)
{
  char path[PATH_MAX];
  size_t len = readlink("/proc/self/exe", path, PATH_MAX);
  if (!CHECK(len)) {
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
  CHECK(pid_);

  if (pid_ < 0) {
    return false;
  }
  else if (pid_ > 0) {
    return true;
  }

  /* Child process initialization. */
  execv(path, char_args.data());

  CHECK(-1 /* execv failed */);
  exit(errno);

  return false;
}

Subprocess::~Subprocess() {}

bool Subprocess::is_running()
{
  if (pid_ == -1) {
    return false;
  }

  pid_t result = waitpid(pid_, nullptr, WNOHANG);
  CHECK(result);

  if (result == pid_) {
    pid_ = -1;
    return false;
  }

  return true;
}

SharedMemory::SharedMemory(std::string name, size_t size, bool already_exists)
    : name_(name), is_owner_(!already_exists)
{
  if (already_exists) {
    handle_ = shm_open(name.c_str(), O_RDWR, 0755);
    CHECK(handle_);
  }
  else {
    handle_ = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0755);
    if (CHECK(handle_)) {
      if (!CHECK(ftruncate(handle_, size))) {
        CHECK(close(handle_));
        handle_ = -1;
      }
    }
  }

  if (handle_ != -1) {
    data_ = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, handle_, 0);
    if (data_ == MAP_FAILED) {
      CHECK(-1 /*mmap failed*/);
      data_ = nullptr;
    }
    /* File descriptor can close after mmap. */
    CHECK(close(handle_));
  }
  else {
    data_ = nullptr;
  }

  data_size_ = data_ ? size : 0;
}

SharedMemory::~SharedMemory()
{
  if (data_) {
    CHECK(munmap(data_, data_size_));
    if (is_owner_) {
      CHECK(shm_unlink(name_.c_str()));
    }
  }
}

SharedSemaphore::SharedSemaphore(std::string name, bool is_owner)
    : name_(name), is_owner_(is_owner)
{
  handle_ = sem_open(name.c_str(), O_CREAT, 0700, 0);
  if (!handle_) {
    CHECK(-1);
  }
}

SharedSemaphore::~SharedSemaphore()
{
  if (handle_) {
    CHECK(sem_close(handle_));
    if (is_owner_) {
      CHECK(sem_unlink(name_.c_str()));
    }
  }
}

void SharedSemaphore::increment()
{
  CHECK(sem_post(handle_));
}

void SharedSemaphore::decrement()
{
  while (true) {
    int result = sem_wait(handle_);
    if (result == 0) {
      return;
    }
    else if (errno != EINTR) {
      CHECK(result);
      return;
    }
    /* Try again if interrupted by handler. */
  }
}

bool SharedSemaphore::try_decrement(int wait_ms)
{
  if (wait_ms == 0) {
    int result = sem_trywait(handle_);
    if (result == 0) {
      return true;
    }
    else if (errno == EINVAL) {
      CHECK(result);
    }
    return false;
  }

  timespec time;
  if (!CHECK(clock_gettime(CLOCK_REALTIME, &time))) {
    BLI_time_sleep_ms(wait_ms * 1000);
    return try_decrement(0);
  }

  time.tv_sec += wait_ms / 1000;
  time.tv_nsec += (wait_ms % 1000) * 10e6;

  while (true) {
    int result = sem_timedwait(handle_, &time);
    if (result == 0) {
      return true;
    }
    else if (errno == EINVAL) {
      CHECK(result);
      return true;
    }
    /* Try again if interrupted by handler. */
  }
}

}  // namespace blender

#  endif

#endif
