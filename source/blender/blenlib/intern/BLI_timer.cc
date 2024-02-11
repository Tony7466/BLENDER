/* SPDX-FileCopyrightText: 2018 Blender Foundation
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bli
 */

#include "BLI_time.h"

#include "BLI_fileops.h"
#include "BLI_listbase.h"
#include "BLI_string.h"
#include "BLI_timer.hh"

#include "MEM_guardedalloc.h"

#define GET_TIME() BLI_check_seconds_timer()

typedef struct TimedFunction {
  struct TimedFunction *next, *prev;
  BLI_timer_func func;
  BLI_timer_data_free user_data_free;
  void *user_data;
  double next_time;
  uintptr_t uuid;
  bool tag_removal;
  bool persistent;
} TimedFunction;

typedef struct TimerContainer {
  ListBase funcs;
} TimerContainer;

static TimerContainer GlobalTimer = {{0}};

void BLI_timer_register(uintptr_t uuid,
                        BLI_timer_func func,
                        void *user_data,
                        BLI_timer_data_free user_data_free,
                        double first_interval,
                        bool persistent)
{
  TimedFunction *timed_func = (TimedFunction *)MEM_callocN(sizeof(TimedFunction), __func__);
  timed_func->func = func;
  timed_func->user_data_free = user_data_free;
  timed_func->user_data = user_data;
  timed_func->next_time = GET_TIME() + first_interval;
  timed_func->tag_removal = false;
  timed_func->persistent = persistent;
  timed_func->uuid = uuid;

  BLI_addtail(&GlobalTimer.funcs, timed_func);
}

static void clear_user_data(TimedFunction *timed_func)
{
  if (timed_func->user_data_free) {
    timed_func->user_data_free(timed_func->uuid, timed_func->user_data);
    timed_func->user_data_free = NULL;
  }
}

bool BLI_timer_unregister(uintptr_t uuid)
{
  LISTBASE_FOREACH (TimedFunction *, timed_func, &GlobalTimer.funcs) {
    if (timed_func->uuid == uuid && !timed_func->tag_removal) {
      timed_func->tag_removal = true;
      clear_user_data(timed_func);
      return true;
    }
  }
  return false;
}

bool BLI_timer_is_registered(uintptr_t uuid)
{
  LISTBASE_FOREACH (TimedFunction *, timed_func, &GlobalTimer.funcs) {
    if (timed_func->uuid == uuid && !timed_func->tag_removal) {
      return true;
    }
  }
  return false;
}

static void execute_functions_if_necessary(void)
{
  double current_time = GET_TIME();

  LISTBASE_FOREACH (TimedFunction *, timed_func, &GlobalTimer.funcs) {
    if (timed_func->tag_removal) {
      continue;
    }
    if (timed_func->next_time > current_time) {
      continue;
    }

    double ret = timed_func->func(timed_func->uuid, timed_func->user_data);

    if (ret < 0) {
      timed_func->tag_removal = true;
    }
    else {
      timed_func->next_time = current_time + ret;
    }
  }
}

static void remove_tagged_functions(void)
{
  for (TimedFunction *timed_func = (TimedFunction *)GlobalTimer.funcs.first; timed_func;) {
    TimedFunction *next = timed_func->next;
    if (timed_func->tag_removal) {
      clear_user_data(timed_func);
      BLI_freelinkN(&GlobalTimer.funcs, timed_func);
    }
    timed_func = next;
  }
}

void BLI_timer_execute(void)
{
  execute_functions_if_necessary();
  remove_tagged_functions();
}

void BLI_timer_free(void)
{
  LISTBASE_FOREACH (TimedFunction *, timed_func, &GlobalTimer.funcs) {
    timed_func->tag_removal = true;
  }

  remove_tagged_functions();
}

static void remove_non_persistent_functions(void)
{
  LISTBASE_FOREACH (TimedFunction *, timed_func, &GlobalTimer.funcs) {
    if (!timed_func->persistent) {
      timed_func->tag_removal = true;
    }
  }
}

void BLI_timer_on_file_load(void)
{
  remove_non_persistent_functions();
}

#if defined(VK_TIMER)
#  include "BLI_dynstr.h"

#  include <stdio.h>

#  define WIN32_LEAN_AND_MEAN
#  include <windows.h>

#  include <stdarg.h>
#  include <synchapi.h>

#  define LOG_THREAD_FILE "path/to/log/file"

#  include "dbghelp.h"
#  pragma comment(lib, "DbgHelp.lib ")
#  include <chrono>
#  include <fcntl.h>
#  include <io.h>

namespace blidebug {

static DWORD thread_main;
void BackTrace(char *dst)
{
#  if 1
  unsigned int i;
#    define STACK_NUMS 30
#    define MAX_INFO_MSG 1024
  void *stack[STACK_NUMS];
  unsigned short frames;
  HANDLE process;
  process = GetCurrentProcess();

  SymInitialize(process, NULL, TRUE);
  memset(&stack, 0, sizeof(uintptr_t) * STACK_NUMS);

  frames = CaptureStackBackTrace(0, STACK_NUMS, stack, NULL);
  memset(dst, 0, MAX_INFO_MSG);
  int total = 0;
  SYMBOL_INFO *symbol = (SYMBOL_INFO *)MEM_callocN(sizeof(SYMBOL_INFO) + 256 * sizeof(char),
                                                   "SYMBOL_INFO");
  symbol->MaxNameLen = 255;
  symbol->SizeOfStruct = sizeof(SYMBOL_INFO);
  static int CALLEE_CNT = 0;
  CALLEE_CNT++;
  for (i = 0; i < frames; i++) {
    if (!stack[i])
      break;
    // mem_lock_thread();
    SymFromAddr(process, (DWORD64)(stack[i]), 0, symbol);
    // mem_unlock_thread();
    if (total + symbol->NameLen > MAX_INFO_MSG) {
      break;
    }
    memcpy(dst, symbol->Name, symbol->NameLen);
    dst[symbol->NameLen] = '\n';
    dst += (symbol->NameLen + 1);
    total += (symbol->NameLen + 1);
    if (strcmp(symbol->Name, "main") == 0) {
      break;
    }
    /*  printf("%i: %s - 0x%0X\n", frames - i - 1, symbol->Name, symbol->Address); */
  }
  MEM_freeN(symbol);

#  endif
}

static SRWLOCK SrwLock;

typedef struct SynchronizedTime {
  FILE *fp[4] = {nullptr, nullptr, nullptr, nullptr};
  int file_id = 0;
  double time_last[12];
  bool write_off;
  bool capture_on;
  bool capture_on_partial;
  int batch_count = 0;
  int imm_count = 0;
  int frame_count = 0;
  int frame_num = 0;
  // std::chrono::time_point time_prev = std::chrono::time_point();
  SynchronizedTime()
  {
    for (int i = 0; i < 4; i++)
      fp[i] = nullptr;
    for (int i = 0; i < 12; i++)
      time_last[i] = 0.;
    write_off = true;
    capture_on_partial = capture_on = false;
    file_id = 0;
    batch_count = 0;
    imm_count = 0;
    frame_count = 0;
  };
  ~SynchronizedTime();
} SynchronizedTime;

static thread_local SynchronizedTime syncTimeCom;
static const int active_index_ = 0b010;
// 0b000000000011;

SynchronizedTime::~SynchronizedTime()
{
  if (fp != nullptr) {
    AcquireSRWLockExclusive(&SrwLock);
    for (int i = 0; i < 4; i++) {
      if (syncTimeCom.fp[i]) {
        fclose(syncTimeCom.fp[i]);
      }
    };
    ReleaseSRWLockExclusive(&SrwLock);
  }
}

#  define SYNC_ACTIVE_INDEX(index_) if (!(bool(1 & (active_index_ >> index_))))
void SyncTime_Open(SynchronizedTime &syncTime)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  char buf[256];
  sprintf(buf, "%sBLI_TID%d-%d.log", LOG_THREAD_FILE, _threadid, syncTimeCom.file_id);
  syncTime.fp[syncTimeCom.file_id] = fopen(buf, "a+");
  syncTime.write_off = false;
  syncTime.capture_on = false;
  syncTime.capture_on_partial = false;
}

void Sync_Write(const char *message)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  static thread_local SynchronizedTime syncTime;
  // printf("SyncWrite  %d   %llx   \n", _threadid, syncTime.fp);
  thread_local char buf[1024];
  AcquireSRWLockExclusive(&SrwLock);
  if (syncTime.fp[syncTimeCom.file_id] == nullptr) {
    SyncTime_Open(syncTime);
  }
  sprintf(buf, "%s\n", message);
  fputs(buf, syncTime.fp[syncTimeCom.file_id]);
  rewind(syncTime.fp[syncTimeCom.file_id]);
  ReleaseSRWLockExclusive(&SrwLock);
}

void SyncTime_Write(const char *message, double time)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  char buf[256];
  sprintf(buf, "TIMEIT[%d]  %.6f s\n%s\n", syncTimeCom.frame_num, time, message);
  Sync_Write(buf);
}

void BLI_sync_time_init()
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  InitializeSRWLock(&SrwLock);
};

void BLI_sync_time_deinit()
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
}

void BLI_timeit_start(int i)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  SYNC_ACTIVE_INDEX(i)
  return;
  syncTimeCom.time_last[i] = BLI_check_seconds_timer();
}

void BLI_timeit_step(const char *message, int i)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  SYNC_ACTIVE_INDEX(i)
  return;
  BLI_timeit_end(message, i);
  // printf("BLI_timeit_step %s\n", message);
  syncTimeCom.time_last[i] = BLI_check_seconds_timer();
}

void BLI_timeit_end(const char *message, int i)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  SYNC_ACTIVE_INDEX(i)
  return;
  double time = BLI_check_seconds_timer();
  SyncTime_Write(message, time - syncTimeCom.time_last[i]);
}

void BLI_timeit_write_off(bool off)
{
  syncTimeCom.write_off = off;
}

void BLI_timeit_info(const char *format, ...)
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  if (syncTimeCom.write_off) {
    return;
  }
  DynStr *ds;
  va_list arg;
  char *info;

  va_start(arg, format);
  ds = BLI_dynstr_new();
  BLI_dynstr_vappendf(ds, format, arg);
  info = BLI_dynstr_get_cstring(ds);
  BLI_dynstr_free(ds);

  va_end(arg);
  Sync_Write(info);
  MEM_freeN((void *)info);
}

bool BLI_timeit_is_write_off()
{
  return syncTimeCom.write_off;
};

void BLI_timeit_capture_on(bool on)
{
  syncTimeCom.capture_on = on;
}

bool BLI_timeit_is_capture_on()
{
  return syncTimeCom.capture_on;
};

void BLI_timeit_capture_on_partial(bool on)
{
  syncTimeCom.capture_on_partial = on;
}

bool BLI_timeit_is_capture_on_partial()
{
  return syncTimeCom.capture_on_partial;
};

void BLI_timeit_backtrace()
{
#  ifdef VK_TIMER_OFF
  return;
#  endif
  if (syncTimeCom.write_off) {
    return;
  }
  char Dst[MAX_INFO_MSG];
  BackTrace(&Dst[0]);
  Sync_Write((const char *)Dst);
}

void BLI_timeit_file_id(int i)
{
  syncTimeCom.file_id = i;
}

void BLI_timeit_count_start(int cnt)
{
  syncTimeCom.batch_count = 0;
  syncTimeCom.imm_count = 0;
  double time = BLI_check_seconds_timer();
  double t = time - syncTimeCom.time_last[2];
  syncTimeCom.frame_num = cnt;
  if (t > 1) {
    int id = syncTimeCom.file_id;
    syncTimeCom.file_id = 2;
    BLI_info_always(
        "===================================== FrameCount %d  time %f  No.[%d] "
        "==============================\n",
        syncTimeCom.frame_count,
        t,
        cnt);
    syncTimeCom.file_id = id;
    syncTimeCom.frame_count = 0;
    syncTimeCom.time_last[2] = BLI_check_seconds_timer();
  }
}

void BLI_timeit_count_end(const char *message)
{
  if (syncTimeCom.batch_count == 0 && syncTimeCom.imm_count == 0) {
    return;
  }
  syncTimeCom.frame_count++;
  int id = syncTimeCom.file_id;
  syncTimeCom.file_id = 2;
  BLI_info_always("BatchCount  %d  ImmCount %d  %s\n",
                  syncTimeCom.batch_count,
                  syncTimeCom.imm_count,
                  message);
  syncTimeCom.file_id = id;
}

void BLI_timeit_fps(bool /*start*/)
{
  static std::chrono::time_point lastTimestamp = std::chrono::high_resolution_clock::now();
  static int frame_count = 0;
  frame_count++;
  std::chrono::time_point tEnd = std::chrono::high_resolution_clock::now();

  float fpsTimer =
      (float)(std::chrono::duration<double, std::milli>(tEnd - lastTimestamp).count());
  if (fpsTimer > 1000.0f) {
    uint32_t lastFPS = static_cast<uint32_t>((float)frame_count * (1000.0f / fpsTimer));
    frame_count = 0;
    lastTimestamp = tEnd;
    BLI_info_always("  %.2f ms     frame(% .1d fps)    fpsTimer %.4f ms  \n",
                    (1000.0f / lastFPS),
                    lastFPS,
                    fpsTimer);
  }
}

void BLI_timeit_batch_count()
{
  syncTimeCom.batch_count++;
}

void BLI_timeit_imm_count()
{
  syncTimeCom.imm_count++;
}
}  // namespace blidebug
#endif
