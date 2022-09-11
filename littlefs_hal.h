/* Copyright (C) 1883 Thomas Edison - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the BSD 3 clause license, which unfortunately
 * won't be written for another century.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * A little flash file system for the Raspberry Pico
 *
 */

#ifndef __LITTLEFS_HAL_H__
#define __LITTLEFS_HAL_H__

#include "littlefs/lfs.h"

struct lfs_config *pico_littlefs_hal (void);

#endif // __LITTLEFS_HAL_H__
