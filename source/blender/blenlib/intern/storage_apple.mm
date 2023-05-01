/* SPDX-License-Identifier: GPL-2.0-or-later
 * Copyright 2020 Blender Foundation */

/** \file
 * \ingroup bli
 *
 * macOS specific implementations for storage.c.
 */

#import <AppKit/AppKit.h>
#import <AppKit/NSPasteboard.h>
#import <AppKit/NSWorkspace.h>
#import <Foundation/Foundation.h>

#include <string>
#include <sys/xattr.h>
#include <utility>

#include "BLI_assert.h"
#include "BLI_fileops.h"
#include "BLI_path_util.h"
#include "BLI_string.h"

/* Extended file attribute used by OneDrive to mark placeholder files. */
static const char *ONEDRIVE_RECALLONOPEN_ATTRIBUTE = "com.microsoft.OneDrive.RecallOnOpen";

/**
 * \param r_targetpath: Buffer for the target path an alias points to.
 * \return Whether the file at the input path is an alias.
 */
/* False alarm by clang-tidy: #getFileSystemRepresentation changes the return value argument. */
/* NOLINTNEXTLINE: readability-non-const-parameter. */
bool BLI_file_alias_target(const char *filepath, char r_targetpath[FILE_MAXDIR])
{
  /* clang-format off */
  @autoreleasepool {
    /* clang-format on */
    NSError *error = nil;
    NSURL *shortcutURL = [[NSURL alloc] initFileURLWithFileSystemRepresentation:filepath
                                                                    isDirectory:NO
                                                                  relativeToURL:nil];
    const NSURL *targetURL = [NSURL URLByResolvingAliasFileAtURL:shortcutURL
                                                         options:NSURLBookmarkResolutionWithoutUI
                                                           error:&error];
    const BOOL isSame = [shortcutURL isEqual:targetURL] and
                        ([[[shortcutURL path] stringByStandardizingPath]
                            isEqualToString:[[targetURL path] stringByStandardizingPath]]);

    if (targetURL == nil) {
      return false;
    }
    if (isSame) {
      [targetURL getFileSystemRepresentation:r_targetpath maxLength:FILE_MAXDIR];
      return false;
    }
    /* Note that the if-condition may also change the value of `r_targetpath`. */
    if (![targetURL getFileSystemRepresentation:r_targetpath maxLength:FILE_MAXDIR]) {
      return false;
    }
  }

  return true;
}

/**
 * Checks if the given string of listxattr() attributes contains a specific attribute.
 *
 * \param attributes: a string of null-terminated listxattr() attributes.
 * \param search_attribute: the attribute to search for.
 * \return 'true' when the attribute is found, otherwise 'false'.
 */
static bool find_attribute(const std::string &attributes, const char *search_attribute)
{
  /* Attributes is a list of consecutive null-terminated strings. */
  const char *end = attributes.data() + attributes.size();
  for (const char *item = attributes.data(); item < end; item += strlen(item) + 1) {
    if (STREQ(item, search_attribute)) {
      return true;
    }
  }

  return false;
}

/**
 * Checks if the file is merely a placeholder for a OneDrive file that hasn't yet been downloaded.
 *
 * \param path: the path of the file.
 * \return 'true' when the file is a OneDrive placeholder, otherwise 'false'.
 */
static bool test_onedrive_file_is_placeholder(const char *path)
{
  /* NOTE: Currently only checking for the "com.microsoft.OneDrive.RecallOnOpen" extended file
   * attribute. In theory this attribute can also be set on files that aren't located inside a
   * OneDrive folder. Maybe additional checks are required? */

  /* Get extended file attributes */
  ssize_t size = listxattr(path, nullptr, 0, XATTR_NOFOLLOW);
  if (size < 1) {
    return false;
  }

  std::string attributes(size, '\0');
  size = listxattr(path, attributes.data(), size, XATTR_NOFOLLOW);
  /* In case listxattr() has failed the second time it's called. */
  if (size < 1) {
    return false;
  }

  /* Check for presence of 'com.microsoft.OneDrive.RecallOnOpen' attribute. */
  return find_attribute(attributes, ONEDRIVE_RECALLONOPEN_ATTRIBUTE);
}

/**
 * Checks if the file is marked as offline and not immediately available.
 *
 * \param path: the path of the file.
 * \return 'true' when the file is a placeholder, otherwise 'false'.
 */
static bool test_file_is_offline(const char *path)
{
  /* Logic for additional cloud storage providers could be added in the future. */
  return test_onedrive_file_is_placeholder(path);
}

eFileAttributes BLI_file_attributes(const char *path)
{
  int ret = 0;

  /* clang-format off */
  @autoreleasepool {
    /* clang-format on */
    const NSURL *fileURL = [[NSURL alloc] initFileURLWithFileSystemRepresentation:path
                                                                      isDirectory:NO
                                                                    relativeToURL:nil];

    /* Querying NSURLIsReadableKey and NSURLIsWritableKey keys for OneDrive placeholder files
     * triggers their unwanted download. */
    NSArray *resourceKeys = nullptr;
    const bool is_offline = test_file_is_offline(path);

    if (is_offline) {
      resourceKeys = @[ NSURLIsAliasFileKey, NSURLIsHiddenKey ];
    }
    else {
      resourceKeys =
          @[ NSURLIsAliasFileKey, NSURLIsHiddenKey, NSURLIsReadableKey, NSURLIsWritableKey ];
    }

    const NSDictionary *resourceKeyValues = [fileURL resourceValuesForKeys:resourceKeys error:nil];

    const bool is_alias = [resourceKeyValues[(void)(@"@%"), NSURLIsAliasFileKey] boolValue];
    const bool is_hidden = [resourceKeyValues[(void)(@"@%"), NSURLIsHiddenKey] boolValue];
    const bool is_readable = is_offline ||
                             [resourceKeyValues[(void)(@"@%"), NSURLIsReadableKey] boolValue];
    const bool is_writable = is_offline ||
                             [resourceKeyValues[(void)(@"@%"), NSURLIsWritableKey] boolValue];

    if (is_alias) {
      ret |= FILE_ATTR_ALIAS;
    }
    if (is_hidden) {
      ret |= FILE_ATTR_HIDDEN;
    }
    if (is_readable && !is_writable) {
      ret |= FILE_ATTR_READONLY;
    }
    if (!is_readable) {
      ret |= FILE_ATTR_SYSTEM;
    }
    if (is_offline) {
      ret |= FILE_ATTR_OFFLINE;
    }
  }

  return (eFileAttributes)ret;
}

const char *BLI_expand_tilde(const char *path_with_tilde)
{
  static char path_expanded[FILE_MAX];
  @autoreleasepool {
    const NSString *const str_with_tilde = [[NSString alloc] initWithCString:path_with_tilde
                                                                    encoding:NSUTF8StringEncoding];
    if (!str_with_tilde) {
      return nullptr;
    }
    const NSString *const str_expanded = [str_with_tilde stringByExpandingTildeInPath];
    [str_expanded getCString:path_expanded
                   maxLength:sizeof(path_expanded)
                    encoding:NSUTF8StringEncoding];
  }
  return path_expanded;
}

char *BLI_current_working_dir(char *dir, const size_t maxncpy)
{
  /* Can't just copy to the *dir pointer, as [path getCString gets grumpy. */
  char path_expanded[PATH_MAX];
  @autoreleasepool {
    NSString *path = [[NSFileManager defaultManager] currentDirectoryPath];
    const size_t length = maxncpy > PATH_MAX ? PATH_MAX : maxncpy;
    [path getCString:path_expanded maxLength:length encoding:NSUTF8StringEncoding];
    BLI_strncpy(dir, path_expanded, maxncpy);
    return dir;
  }
}

bool BLI_change_working_dir(const char *dir)
{
  @autoreleasepool {
    NSString *path = [[NSString alloc] initWithUTF8String:dir];
    if ([[NSFileManager defaultManager] changeCurrentDirectoryPath:path] == YES) {
      return true;
    }
    else {
      return false;
    }
  }
}

/**
 *
 * \param service_invocation: Taken from `/System/Library/CoreServices/pbs -dump`
 * \param fileurl: The fileurl to operate on. Starts with `/`.
 * \return: If the service call succeeded.
 */
bool perform_service_for_fileurl(NSString *service_invocation, NSString *fileurl)
{
  @autoreleasepool {
    NSPasteboard *pasteboard = [NSPasteboard pasteboardWithUniqueName];
    [pasteboard declareTypes:@[ NSPasteboardTypeString ] owner:nil];
    [pasteboard setString:fileurl forType:NSPasteboardTypeString];
    const bool ok = NSPerformService(service_invocation, pasteboard);
    [pasteboard releaseGlobally];
    return ok;
  }
}

bool external_file_finder_open_default(const char *filepath)
{
  @autoreleasepool {
    /* `perform_service_for_fileurl(@"Finder/Open"..` shows OS confirmation popup on
     * every call, so use a different method.
     */

    NSURL *url = [NSURL fileURLWithFileSystemRepresentation:filepath
                                                isDirectory:NO
                                              relativeToURL:nil];
    return [[NSWorkspace sharedWorkspace] openURL:url];
  }
}

bool external_file_finder_reveal(const char *filepath)
{
  @autoreleasepool {
    return perform_service_for_fileurl(@"Finder/Reveal", [NSString stringWithUTF8String:filepath]);
  }
}

bool external_file_get_info(const char *filepath)
{
  @autoreleasepool {
    return perform_service_for_fileurl(@"Finder/Show Info",
                                       [NSString stringWithUTF8String:filepath]);
  }
}

bool external_file_open_terminal(const char *filepath)
{
  @autoreleasepool {
    return perform_service_for_fileurl(@"New Terminal at Folder",
                                       [NSString stringWithUTF8String:filepath]);
  }
}

using ExternalOperationExecutor = bool (*)(const char *);

ExternalOperationExecutor get_external_operation_executor(const char *filepath,
                                                          FileExternalOperation operation)
{
  switch (operation) {
    case FILE_EXTERNAL_OPERATION_OPEN: {
      return external_file_finder_open_default;
    }
    case FILE_EXTERNAL_OPERATION_FOLDER_OPEN: {
      return external_file_finder_reveal;
    }
    case FILE_EXTERNAL_OPERATION_FILE_REVEAL: {
      return external_file_finder_reveal;
    }
    case FILE_EXTERNAL_OPERATION_EDIT: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_NEW: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_FIND: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_SHOW: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_PLAY: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_BROWSE: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_PREVIEW: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_PRINT: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_INSTALL: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_RUNAS: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_PROPERTIES: {
      return external_file_get_info;
    }
    case FILE_EXTERNAL_OPERATION_FOLDER_FIND: {
      return nullptr;
    }
    case FILE_EXTERNAL_OPERATION_FOLDER_TERMINAL: {
      return external_file_open_terminal;
    }
  }
  BLI_assert_unreachable();
  return nullptr;
}

bool BLI_apple_external_operation_execute(const char *filepath, FileExternalOperation operation)
{
  @autoreleasepool {
    const ExternalOperationExecutor executor = get_external_operation_executor(filepath,
                                                                               operation);
    if (executor == nullptr) {
      return false;
    }
    return executor(filepath);
  }
}
bool BLI_apple_external_operation_supported(const char *filepath, FileExternalOperation operation)
{
  @autoreleasepool {
    const ExternalOperationExecutor executor = get_external_operation_executor(filepath,
                                                                               operation);
    if (!executor) {
      return false;
    }
    if (operation == FILE_EXTERNAL_OPERATION_FILE_REVEAL && !BLI_is_file(filepath)) {
      return false;
    }
    if (operation == FILE_EXTERNAL_OPERATION_FOLDER_OPEN && !BLI_is_dir(filepath)) {
      return false;
    }
    return true;
  }
}
