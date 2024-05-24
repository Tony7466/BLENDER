# SPDX-FileCopyrightText: 2017-2024 Blender Authors
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Only download, will be copied as submodule for hiprt.

ExternalProject_Add(external_orochi
  URL file://${PACKAGE_DIR}/${OROCHI_FILE}
  DOWNLOAD_DIR ${DOWNLOAD_DIR}
  URL_HASH ${OROCHI_HASH_TYPE}=${OROCHI_HASH}
  PREFIX ${BUILD_DIR}/orochi

  CONFIGURE_COMMAND echo .
  BUILD_COMMAND echo .
  INSTALL_COMMAND echo .
)
