/*  ipc.h - v0.2 - public domain cross platform inter process communication
                   no warranty implied; use at your own risk

    by Jari Komppa, http://iki.fi/sol/

(Modified version from Malt https://github.com/bnpr/Malt/tree/Development/Bridge/ipc)


You can #define IPC_MALLOC, and IPC_FREE to avoid using malloc,free

LICENSE

    This is free and unencumbered software released into the public domain.

    Anyone is free to copy, modify, publish, use, compile, sell, or
    distribute this software, either in source code form or as a compiled
    binary, for any purpose, commercial or non-commercial, and by any
    means.

    In jurisdictions that recognize copyright laws, the author or authors
    of this software dedicate any and all copyright interest in the
    software to the public domain. We make this dedication for the benefit
    of the public at large and to the detriment of our heirs and
    successors. We intend this dedication to be an overt act of
    relinquishment in perpetuity of all present and future rights to this
    software under copyright law.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
    IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
    OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
    ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
    OTHER DEALINGS IN THE SOFTWARE.

    For more information, please refer to <http://unlicense.org/>

USAGE

    Shared memory API:

    void ipc_mem_init(ipc_sharedmemory *mem, unsigned const char *name, size_t size);
    - Initialize ipc_sharedmemory structure for use. Call this first.

    int ipc_mem_open_existing(ipc_sharedmemory *mem);
    - Try to open existing shared memory. Returns 0 for success.

    int ipc_mem_create(ipc_sharedmemory *mem);
    - Try to create shared memory. Returns 0 for success.

    void ipc_mem_close(ipc_sharedmemory *mem);
    - Close shared memory and free allocated stuff. Shared memory will only get
      destroyed when nobody is accessing it. Call this last.

    unsigned char *ipc_mem_access(ipc_sharedmemory *mem);
    - Access the shared memory. Same as poking mem->data directly, but some people
      like accessors.



    Shared semaphore API:

    void ipc_sem_init(ipc_sharedsemaphore *sem, unsigned const char *name);
    - Initialize ipc_sharedsemaphore for use. Call this first.

    int ipc_sem_create(ipc_sharedsemaphore *sem, int initialvalue);
    - Try to create semaphore. Returns 0 for success.

    void ipc_sem_close(ipc_sharedsemaphore *sem);
    - Close the semaphore and deallocate. Shared semaphore will only go away after
      nobody is using it. Call this last.

    void ipc_sem_increment(ipc_sharedsemaphore *sem);
    - Increment the semaphore.

    void ipc_sem_decrement(ipc_sharedsemaphore *sem);
    - Decrement the semaphore, waiting forever if need be.

    int ipc_sem_try_decrement(ipc_sharedsemaphore *sem);
    - Try to decrement the semaphore, returns 1 for success, 0 for failure.

TROUBLESHOOTING

    - Thread programming issues apply; don't mess with the memory someone else might
      be reading, make sure you release semaphore after use not to hang someone else,
      etc.
    - If process crashes while holding a semaphore, the others may end up waiting
      forever for them to release.
    - On Linux, named items will remain after your application closes unless you
      close them. This is particularly fun if your application crashes. Having a
      commandline mode that just tries to create and then close all your shared
      resources may be convenient. Saves on rebooting, at least.. On windows, if
      nobody is around to use the resource, they disappear.
*/

#pragma once

typedef void *HANDLE;
#ifndef _WIN32
#  include <semaphore.h>
#endif

typedef struct ipc_sharedmemory_ {
  char *name;
  unsigned char *data;
  size_t size;
  HANDLE handle;
  int fd;
} ipc_sharedmemory;

void ipc_mem_init(ipc_sharedmemory *mem, const char *name, size_t size);
int ipc_mem_open_existing(ipc_sharedmemory *mem);
int ipc_mem_create(ipc_sharedmemory *mem);
void ipc_mem_close(ipc_sharedmemory *mem, bool unlink);
unsigned char *ipc_mem_access(ipc_sharedmemory *mem);

typedef struct ipc_sharedsemaphore_ {
  char *name;
#if defined(_WIN32)
  HANDLE handle;
#else
  sem_t *semaphore;
#endif
} ipc_sharedsemaphore;

void ipc_sem_init(ipc_sharedsemaphore *sem, const char *name);
int ipc_sem_create(ipc_sharedsemaphore *sem, int initialvalue);
void ipc_sem_close(ipc_sharedsemaphore *sem);
void ipc_sem_increment(ipc_sharedsemaphore *sem);
void ipc_sem_decrement(ipc_sharedsemaphore *sem);
int ipc_sem_try_decrement(ipc_sharedsemaphore *sem);
