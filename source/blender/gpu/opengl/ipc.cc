#include "ipc.hh"

#if defined(_WIN32)
#  include <windows.h>
#else  // !_WIN32
#  include <errno.h>
#  include <fcntl.h>
#  include <stdlib.h>
#  include <sys/mman.h>
#  include <sys/stat.h>
#  include <unistd.h>
#endif  // !_WIN32

#ifndef IPC_ASSERT
#  define IPC_ASSERT(x) assert(x)
#endif

#ifndef IPC_MALLOC
#  define IPC_MALLOC(x) malloc(x)
#endif

#ifndef IPC_FREE
#  define IPC_FREE(x) free(x)
#endif

static char *ipc_strdup(const char *src)
{
  int i, len;
  char *dst = NULL;
  len = 0;
  while (src[len])
    len++;
#if !defined(_WIN32)
  len++;
#endif
  dst = (char *)IPC_MALLOC(len + 1);
  if (!dst)
    return NULL;
  dst[len] = 0;

#if defined(_WIN32)
  for (i = 0; i < len; i++)
    dst[i] = src[i];
#else
  dst[0] = '/';
  for (i = 0; i < len - 1; i++)
    dst[i + 1] = src[i];
#endif
  return dst;
}

void ipc_mem_init(ipc_sharedmemory *mem, const char *name, size_t size)
{
  mem->name = ipc_strdup(name);

  mem->size = size;
  mem->data = NULL;
#if defined(_WIN32)
  mem->handle = 0;
#else
  mem->fd = -1;
#endif
}

unsigned char *ipc_mem_access(ipc_sharedmemory *mem)
{
  return mem->data;
}

void ipc_sem_init(ipc_sharedsemaphore *sem, const char *name)
{
  sem->name = ipc_strdup(name);
#if defined(_WIN32)
  sem->handle = 0;
#else
  sem->semaphore = NULL;
#endif
}

#if defined(_WIN32)

int ipc_mem_open_existing(ipc_sharedmemory *mem)
{
  mem->handle = OpenFileMappingA(FILE_MAP_ALL_ACCESS, FALSE, mem->name);

  if (!mem->handle)
    return -1;

  mem->data = (unsigned char *)MapViewOfFile(mem->handle, FILE_MAP_ALL_ACCESS, 0, 0, mem->size);

  if (!mem->data)
    return -1;
  return 0;
}

int ipc_mem_create(ipc_sharedmemory *mem)
{
  mem->handle = CreateFileMappingA(
      INVALID_HANDLE_VALUE, NULL, PAGE_READWRITE, 0, mem->size, mem->name);

  if (!mem->handle)
    return -1;

  mem->data = (unsigned char *)MapViewOfFile(mem->handle, FILE_MAP_ALL_ACCESS, 0, 0, mem->size);

  if (!mem->data)
    return -1;

  return 0;
}

void ipc_mem_close(ipc_sharedmemory *mem, bool unlink)
{
  if (mem->data != NULL) {
    UnmapViewOfFile(mem->data);
    mem->data = NULL;
  }
  IPC_FREE(mem->name);
  mem->name = NULL;
  mem->size = 0;
}

int ipc_sem_create(ipc_sharedsemaphore *sem, int initialvalue)
{
  sem->handle = CreateSemaphoreA(NULL, initialvalue, 0x7fffffff, sem->name);
  if (!sem->handle)
    return -1;
  return 0;
}

void ipc_sem_close(ipc_sharedsemaphore *sem)
{
  CloseHandle(sem->handle);
  IPC_FREE(sem->name);
  sem->handle = 0;
}

void ipc_sem_increment(ipc_sharedsemaphore *sem)
{
  ReleaseSemaphore(sem->handle, 1, NULL);
}

void ipc_sem_decrement(ipc_sharedsemaphore *sem)
{
  WaitForSingleObject(sem->handle, INFINITE);
}

int ipc_sem_try_decrement(ipc_sharedsemaphore *sem)
{
  DWORD ret = WaitForSingleObject(sem->handle, 0);
  if (ret == WAIT_OBJECT_0)
    return 1;
  return 0;
}

#else  // !defined(_WIN32)

int ipc_mem_open_existing(ipc_sharedmemory *mem)
{
  mem->fd = shm_open(mem->name, O_RDWR, 0755);
  if (mem->fd < 0)
    return -1;

  mem->data = (unsigned char *)mmap(
      NULL, mem->size, PROT_READ | PROT_WRITE, MAP_SHARED, mem->fd, 0);
  if (!mem->data)
    return -1;

  // file descriptor can close after mmap
  close(mem->fd);

  return 0;
}

int ipc_mem_create(ipc_sharedmemory *mem)
{
  int ret;
  ret = shm_unlink(mem->name);
  if (ret < 0 && errno != ENOENT)
    return -1;

  mem->fd = shm_open(mem->name, O_CREAT | O_RDWR, 0755);
  if (mem->fd < 0)
    return -1;

  ftruncate(mem->fd, mem->size);

  mem->data = (unsigned char *)mmap(
      NULL, mem->size, PROT_READ | PROT_WRITE, MAP_SHARED, mem->fd, 0);
  if (!mem->data)
    return -1;

  // file descriptor can close after mmap
  close(mem->fd);

  return 0;
}

void ipc_mem_close(ipc_sharedmemory *mem, bool unlink)
{
  if (mem->data != NULL) {
    munmap(mem->data, mem->size);
    // close(mem->fd);
    if (unlink)
      shm_unlink(mem->name);
  }
  IPC_FREE(mem->name);
  mem->name = NULL;
  mem->size = 0;
}

int ipc_sem_create(ipc_sharedsemaphore *sem, int initialvalue)
{
  sem->semaphore = sem_open(sem->name, O_CREAT, 0700, initialvalue);
  if (sem->semaphore == SEM_FAILED)
    return -1;
  return 0;
}

void ipc_sem_close(ipc_sharedsemaphore *sem)
{
  sem_close(sem->semaphore);
  sem_unlink(sem->name);
  IPC_FREE(sem->name);
}

void ipc_sem_increment(ipc_sharedsemaphore *sem)
{
  sem_post(sem->semaphore);
}

void ipc_sem_decrement(ipc_sharedsemaphore *sem)
{
  sem_wait(sem->semaphore);
}

int ipc_sem_try_decrement(ipc_sharedsemaphore *sem)
{
  int res = sem_trywait(sem->semaphore);
  if (res == 0)
    return 1;
  return 0;
}

#endif  // !_WIN32
