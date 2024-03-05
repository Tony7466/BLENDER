/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup bke
 *
 * Generic CLI "--command" declarations.
 *
 * Duplicate Commands
 * ==================
 *
 * When two or more commands share the same identifier, a warning is printed and both are disabled.
 *
 * This is done because command-line actions may be destructive so the down-side of running the
 * wrong command could be severe. The reason this is not considered an error is we can't prevent
 * it so easily, unlike operator ID's which may be longer, commands are typically short terms
 * which wont necessarily include an add-ons identifier as a prefix for e.g.
 * Further, an error would break loading add-ons who's primary is *not*
 * necessarily to provide command-line access.
 * An alternative solution could be to generate unique names (number them for example)
 * but this isn't reliable as it would depend on it the order add-ons are loaded which
 * isn't under user control.
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "BLI_array.hh"
#include "BLI_listbase.h"

#include "BKE_blender_cli_command.hh" /* own include */

#include "MEM_guardedalloc.h"

/* -------------------------------------------------------------------- */
/** \name Internal Types
 * \{ */

struct CommandData {
  CommandData *next, *prev;
  CommandExecFn *exec_fn;
  CommandFreeFn *free_fn;

  void *user_data;

  /** See top-level description for handling of duplicate commands. */
  bool is_duplicate;

  /** Over allocate. */
  char id[0];
};

/** \} */

/* -------------------------------------------------------------------- */
/** \name Internal API
 * \{ */

static ListBase g_command_handlers = {};

static CommandData *blender_cli_command_lookup(const char *id)
{
  return static_cast<CommandData *>(
      BLI_findstring(&g_command_handlers, id, offsetof(CommandData, id)));
}

static void blender_cli_command_free(CommandData *cmd)
{
  if (cmd->free_fn) {
    cmd->free_fn(cmd->user_data);
  }
  MEM_freeN(static_cast<void *>(cmd));
}

/** \} */

/* -------------------------------------------------------------------- */
/** \name Public API
 * \{ */

CommandHandle *BKE_blender_cli_command_register(const char *id,
                                                CommandExecFn *exec_fn,
                                                CommandFreeFn *free_fn,
                                                void *user_data)
{
  bool is_duplicate = false;
  if (CommandData *cmd_exists = blender_cli_command_lookup(id)) {
    fprintf(
        stderr, "warning: registered duplicate command \"%s\", this will be inaccessible.\n", id);
    cmd_exists->is_duplicate = true;
    is_duplicate = true;
  }
  size_t id_size = strlen(id) + 1;
  CommandData *cmd = static_cast<CommandData *>(MEM_mallocN(sizeof(*cmd) + id_size, __func__));
  cmd->exec_fn = exec_fn;
  cmd->free_fn = free_fn;
  cmd->user_data = user_data;
  cmd->is_duplicate = is_duplicate;
  memcpy(cmd->id, id, id_size);
  BLI_addtail(&g_command_handlers, cmd);
  return static_cast<CommandHandle *>(cmd);
}

bool BKE_blender_cli_command_unregister(CommandHandle *cmd_handle)
{
  CommandData *cmd = (CommandData *)cmd_handle;
  if (!BLI_remlink_safe(&g_command_handlers, cmd)) {
    fprintf(stderr, "failed to unregister command handler\n");
    return false;
  }

  /* Update duplicates after removal. */
  if (cmd->is_duplicate) {
    CommandData *cmd_other = nullptr;
    int duplicate_count = 0;
    LISTBASE_FOREACH (CommandData *, cmd_iter, &g_command_handlers) {
      if (cmd_iter->is_duplicate && STREQ(cmd_iter->id, cmd->id)) {
        duplicate_count += 1;
        cmd_other = cmd_iter;
      }
    }
    if (duplicate_count == 1) {
      cmd_other->is_duplicate = false;
    }
  }

  blender_cli_command_free(cmd);
  return true;
}

int BKE_blender_cli_command_exec(bContext *C, const char *id, const int argc, const char **argv)
{
  CommandData *cmd = blender_cli_command_lookup(id);
  if (cmd == nullptr) {
    fprintf(stderr, "Unrecognized command: \"%s\"\n", id);
    return EXIT_FAILURE;
  }
  if (cmd->is_duplicate) {
    fprintf(stderr,
            "Command: \"%s\" was registered multiple times, must be resolved, aborting!\n",
            id);
    return EXIT_FAILURE;
  }

  return cmd->exec_fn(C, cmd->user_data, argc, argv);
}

void BKE_blender_cli_command_print_help()
{
  printf("Blender Command Listing:\n");
  const int command_num = BLI_listbase_count(&g_command_handlers);
  blender::Array<const char *> command_ids(command_num);

  /* First show commands, then duplicate commands (which are skipped). */
  for (int pass = 0; pass < 2; pass++) {
    const bool is_duplicate = pass > 0;
    int command_num_for_pass = 0;
    LISTBASE_FOREACH (CommandData *, cmd_iter, &g_command_handlers) {
      if (cmd_iter->is_duplicate != is_duplicate) {
        continue;
      }
      command_ids[command_num_for_pass++] = cmd_iter->id;
    }
    if (command_num_for_pass == 0) {
      printf("\tNone found.\n");
    }
    else {
      std::sort(command_ids.begin(), command_ids.begin() + command_num_for_pass, strcmp);
      if (is_duplicate) {
        printf("Duplicate Command Listing (ignored):\n");
      }
      for (int i = 0; i < command_num_for_pass; i++) {
        if (is_duplicate) {
          if ((i > 0) && STREQ(command_ids[i - 1], command_ids[i])) {
            continue;
          }
        }
        printf("\t%s\n", command_ids[i]);
      }
    }
    /* No duplicates for pass zero. */
    if (command_num_for_pass == command_num) {
      break;
    }
  }
}

void BKE_blender_cli_command_free_all()
{
  while (CommandData *cmd = static_cast<CommandData *>(BLI_pophead(&g_command_handlers))) {
    blender_cli_command_free(cmd);
  }
}

/** \} */
