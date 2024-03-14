/* SPDX-FileCopyrightText: 2001-2002 NaN Holding BV. All rights reserved.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later */

/** \file
 * \ingroup imbuf
 */
#pragma once

struct ImBuf;
enum ImBufOwnership : uint8_t;

void imb_refcounter_lock_init();
void imb_refcounter_lock_exit();

#ifndef WIN32
void imb_mmap_lock_init();
void imb_mmap_lock_exit();
void imb_mmap_lock();
void imb_mmap_unlock();
#else
#  define imb_mmap_lock_init()
#  define imb_mmap_lock_exit()
#  define imb_mmap_lock()
#  define imb_mmap_unlock()
#endif

bool imb_addencodedbufferImBuf(ImBuf *ibuf);
bool imb_enlargeencodedbufferImBuf(ImBuf *ibuf);
void imb_assign_encoded_buffer(ImBuf *ibuf,
                               uint8_t *buffer_data,
                               unsigned int buffer_size,
                               unsigned int size,
                               ImBufOwnership ownership);
