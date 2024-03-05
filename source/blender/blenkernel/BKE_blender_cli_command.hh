/* SPDX-FileCopyrightText: 2024 Blender Authors
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

#pragma once

/** \file
 * \ingroup bke
 * \brief Blender CLI Generic `--command` Support.
 *
 * \note all registered commands must print help to the STDOUT & exit with a zero exit-code
 * when `--help` is passed in as the first argument to a command.
 */

#include "BLI_compiler_attrs.h"

/**
 * Run the command with an argument list.
 * The arguments begin at the first argument after the command identifier.
 * The return value is used as the commands exit-code.
 */
using CommandExecFn = int(struct bContext *C, void *user_data, int argc, const char **argv);
/**
 * Frees user data (called on exit).
 */
using CommandFreeFn = void(void *user_data);
/**
 * Opaque pointer, use for un-registering previously registered commands.
 */
using CommandHandle = void;

/**
 * \param id: The command identifier (non-empty string).
 * \param exec_fn: The callback to execute the command.
 * \param free_fn: The callback to free the `user_data` (optional).
 * \param user_data: User data passed to `exec_fn` & may be freed by `free_fn`.
 * \return Command handle (allowing this to be unregistered).
 */
CommandHandle *BKE_blender_cli_command_register(const char *id,
                                                CommandExecFn *exec_fn,
                                                CommandFreeFn *free_fn,
                                                void *user_data) ATTR_NONNULL(1, 2);

/**
 * Unregister a previously registered command.
 */
bool BKE_blender_cli_command_unregister(CommandHandle *cmd_handle);

/**
 * Run the command by `id`, passing in the argument list & context.
 * The argument list must begin after the command identifier.
 */
int BKE_blender_cli_command_exec(struct bContext *C,
                                 const char *id,
                                 const int argc,
                                 const char **argv);

/**
 * Print all known commands (used for passing `--command help` in the command-line).
 */
void BKE_blender_cli_command_print_help();
/**
 * Frees all commands (using their #CommandFreeFn call-backs).
 */
void BKE_blender_cli_command_free_all();
