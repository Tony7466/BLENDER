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

#include "atomic_ops.h"

#include "sequencer_intern.h"

typedef struct PreviewJob {
  ListBase previews;
  ThreadMutex *mutex;
  Scene *scene;
  int total;
  int processed;
  ThreadCondition preview_suspend_cond;
  bool running;
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

static void clear_sound_waveform_loading_tag(bSound *sound)
{
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

  BLI_mutex_lock(task->wm_job->mutex);
  task->wm_job->processed++;
  BLI_mutex_unlock(task->wm_job->mutex);

  BLI_condition_notify_one(&task->wm_job->preview_suspend_cond);

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
}

/* Only this runs inside thread. */
static void preview_startjob(void *data, bool *stop, bool *do_update, float *progress)
{
  TaskPool *task_pool = BLI_task_pool_create(NULL, TASK_PRIORITY_LOW);
  PreviewJob *pj = data;
  bool keep_looping = true;

  BLI_mutex_lock(pj->mutex);
  int previous_processed = pj->processed;
  BLI_mutex_unlock(pj->mutex);

  while (keep_looping) {
    /* Wait until there's either a new audio job to process or one of the previously submitted jobs
     * is done.*/
    BLI_mutex_lock(pj->mutex);
    while (pj->previews.first == NULL && previous_processed == pj->processed) {
      BLI_condition_wait(&pj->preview_suspend_cond, pj->mutex);
    }

    previous_processed = pj->processed;
    float current_progress = (pj->total > 0) ? (float)pj->processed / (float)pj->total : 1.0f;

    if (current_progress != *progress) {
      *progress = current_progress;
      *do_update = true;
    }

    if (pj->processed == pj->total) {
      keep_looping = false;
      pj->running = false;
    }
    else if (*stop || G.is_break) {
      BLI_task_pool_cancel(task_pool);

      /* Clear all pending jobs data. */
      for (PreviewJobAudio *previewjb = pj->previews.first; previewjb != NULL;
           previewjb = previewjb->next)
      {
        clear_sound_waveform_loading_tag(previewjb->sound);
      }
      BLI_freelistN(&pj->previews);
      pj->processed = 0;
      pj->total = 0;

      keep_looping = false;
      pj->running = false;
    }
    else {
      /* Push all available jobs to the processing queue. */
      PreviewJobAudio *previewjb = pj->previews.first;
      while (previewjb) {
        ReadSoundWaveformTask *task = (ReadSoundWaveformTask *)MEM_mallocN(
            sizeof(ReadSoundWaveformTask), "read sound waveform task");
        task->wm_job = pj;
        task->wm_progress = progress;
        task->wm_do_update = do_update;
        task->preview_job_audio = previewjb;
        task->stop = stop;

        BLI_task_pool_push(task_pool,
                           execute_read_sound_waveform_task,
                           task,
                           true,
                           free_read_sound_waveform_task);

        PreviewJobAudio *next_previewjb = previewjb->next;
        BLI_remlink(&pj->previews, previewjb);

        previewjb = next_previewjb;
      }

      BLI_mutex_unlock(pj->mutex);
    }
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

  if (pj) {
    BLI_mutex_lock(pj->mutex);

    /* If the job exists but is not running, bail and try again on the next draw call. */
    if (!pj->running) {
      BLI_mutex_unlock(pj->mutex);

      /* Clear the sound loading tag to that it can be reattempted. */
      clear_sound_waveform_loading_tag(seq->sound);
      WM_event_add_notifier(C, NC_SCENE | ND_SPACE_SEQUENCER, CTX_data_scene(C));
      return;
    }

    BLI_mutex_unlock(pj->mutex);
  }
  else { /* There's no existig preview job. */
    pj = MEM_callocN(sizeof(PreviewJob), "preview rebuild job");

    pj->mutex = BLI_mutex_alloc();
    BLI_condition_init(&pj->preview_suspend_cond);
    pj->scene = CTX_data_scene(C);
    pj->running = true;

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

  BLI_condition_notify_one(&pj->preview_suspend_cond);

  if (!WM_jobs_is_running(wm_job)) {
    G.is_break = false;
    WM_jobs_start(CTX_wm_manager(C), wm_job);
  }

  ED_area_tag_redraw(area);
}
