/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup spseq
 */

#include "DNA_sequence_types.h"
#include "DNA_sound_types.h"

#include "BLI_listbase.h"
#include "BLI_task.h"
#include "BLI_threads.h"

#include "BKE_context.h"
#include "BKE_global.h"
#include "BKE_sound.h"

#include "WM_api.h"
#include "WM_types.h"

#include "ED_screen.h"

#include "MEM_guardedalloc.h"

#include "sequencer_intern.h"

typedef struct PreviewJob {
  ListBase previews;
  ThreadMutex *mutex;
  Scene *scene;
  int total;
  int processed;
} PreviewJob;

typedef struct PreviewJobAudio {
  struct PreviewJobAudio *next, *prev;
  struct Main *bmain;
  bSound *sound;
  int lr; /* Sample left or right. */
  int startframe;
  bool waveform; /* Reload sound or waveform. */
} PreviewJobAudio;

typedef struct ReadSoundWaveformTask {
  PreviewJob *wm_job;
  bool *wm_do_update;
  float *wm_progress;

  PreviewJobAudio *preview_job_audio;
  bool *stop;
} ReadSoundWaveformTask;

static void free_preview_job(void *data)
{
  PreviewJob *pj = (PreviewJob *)data;

  BLI_mutex_free(pj->mutex);
  BLI_freelistN(&pj->previews);
  MEM_freeN(pj);
}

static void clear_sound_waveform_loading_tag(bSound *sound) {
    BLI_spin_lock(sound->spinlock);
    sound->tags &= ~SOUND_TAGS_WAVEFORM_LOADING;
    BLI_spin_unlock(sound->spinlock);
}

static void free_read_sound_waveform_task(struct TaskPool *__restrict task_pool, void *data)
{
  UNUSED_VARS(task_pool);

  ReadSoundWaveformTask *task = (ReadSoundWaveformTask *)data;

  /* The job audio has already been removed from the list, now we just need to free it. */
  MEM_freeN(task->preview_job_audio);

  MEM_freeN(task);
}

static void execute_read_sound_waveform_task(struct TaskPool *__restrict task_pool,
                                             void *task_data)
{
  ReadSoundWaveformTask *task = (ReadSoundWaveformTask *)task_data;

  if (BLI_task_pool_current_canceled(task_pool)) {
    clear_sound_waveform_loading_tag(task->preview_job_audio->sound);
    return;
  }

  PreviewJobAudio *audio_job = task->preview_job_audio;
  BKE_sound_read_waveform(audio_job->bmain, audio_job->sound, task->stop);

  BLI_mutex_lock(task->wm_job->mutex);
  task->wm_job->processed++;
  *task->wm_progress = (task->wm_job->total > 0) ? (float)task->wm_job->processed / (float)task->wm_job->total : 1.0f;
  *task->wm_do_update = true;
  BLI_mutex_unlock(task->wm_job->mutex);
}

/* Only this runs inside thread. */
static void preview_startjob(void *data, bool *stop, bool *do_update, float *progress)
{
  TaskPool *task_pool = BLI_task_pool_create(NULL, TASK_PRIORITY_LOW);

  PreviewJob *pj = data;
  PreviewJobAudio *previewjb;

  BLI_mutex_lock(pj->mutex);
  previewjb = pj->previews.first;
  BLI_mutex_unlock(pj->mutex);

  while (previewjb) {
    PreviewJobAudio *preview_next;

    ReadSoundWaveformTask *task = (ReadSoundWaveformTask *)MEM_mallocN(sizeof(ReadSoundWaveformTask), "read sound waveform task");
    task->wm_job = pj;
    task->wm_progress = progress;
    task->wm_do_update = do_update;
    task->preview_job_audio = previewjb;
    task->stop = stop;

    BLI_task_pool_push(task_pool, execute_read_sound_waveform_task, task, false, NULL);

    if (*stop || G.is_break) {
      BLI_task_pool_cancel(task_pool);

      BLI_mutex_lock(pj->mutex);
      previewjb = previewjb->next;
      BLI_mutex_unlock(pj->mutex);
      while (previewjb) {
        clear_sound_waveform_loading_tag(previewjb->sound);

        BLI_mutex_lock(pj->mutex);
        previewjb = previewjb->next;
        BLI_mutex_unlock(pj->mutex);
      }

      BLI_mutex_lock(pj->mutex);
      BLI_freelistN(&pj->previews);

      /* This is set to zero without an atomic operation because at this point there will be no other threads competing for access to this variable.
       * BLI_task_pool_cancel ensures that all tasks submitted to the pool are stopped before returning.
       */

      pj->total = 0;
      pj->processed = 0;
      BLI_mutex_unlock(pj->mutex);
      break;
    }

    BLI_mutex_lock(pj->mutex);
    preview_next = previewjb->next;
    BLI_remlink(&pj->previews, previewjb);
    previewjb = preview_next;

    BLI_mutex_unlock(pj->mutex);
  }

  BLI_task_pool_work_and_wait(task_pool);
  BLI_task_pool_free(task_pool);
}

static void preview_endjob(void *data)
{
  PreviewJob *pj = data;

  WM_main_add_notifier(NC_SCENE | ND_SEQUENCER, pj->scene);
}

void sequencer_preview_add_sound(const bContext *C, Sequence *seq)
{
  wmJob *wm_job;
  PreviewJob *pj;
  ScrArea *area = CTX_wm_area(C);
  PreviewJobAudio *audiojob = MEM_callocN(sizeof(PreviewJobAudio), "preview_audio");
  wm_job = WM_jobs_get(CTX_wm_manager(C),
                       CTX_wm_window(C),
                       CTX_data_scene(C),
                       "Strip Previews",
                       WM_JOB_PROGRESS,
                       WM_JOB_TYPE_SEQ_BUILD_PREVIEW);

  /* Get the preview job if it exists. */
  pj = WM_jobs_customdata_get(wm_job);

  if (!pj) {
    pj = MEM_callocN(sizeof(PreviewJob), "preview rebuild job");

    pj->mutex = BLI_mutex_alloc();
    pj->scene = CTX_data_scene(C);

    WM_jobs_customdata_set(wm_job, pj, free_preview_job);
    WM_jobs_timer(wm_job, 0.1, NC_SCENE | ND_SEQUENCER, NC_SCENE | ND_SEQUENCER);
    WM_jobs_callbacks(wm_job, preview_startjob, NULL, NULL, preview_endjob);
  }

  audiojob->bmain = CTX_data_main(C);
  audiojob->sound = seq->sound;

  BLI_mutex_lock(pj->mutex);
  BLI_addtail(&pj->previews, audiojob);
  pj->total++;
  BLI_mutex_unlock(pj->mutex);

  if (!WM_jobs_is_running(wm_job)) {
    G.is_break = false;
    WM_jobs_start(CTX_wm_manager(C), wm_job);
  }

  ED_area_tag_redraw(area);
}
