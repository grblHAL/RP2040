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

// Code lifted from https://github.com/lurk101/pico-littlefs
// Modified by Terje Io for grblHAL and latest littlefs version

#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"
#include "hardware/sync.h"
#include "pico/mutex.h"

#include "littlefs_hal.h"

#define FS_SIZE (512 * 1024)

typedef struct {
    const char *baseaddr;
    const char *nocacheaddr;
#if LFS_THREADSAFE
    recursive_mutex_t fs_mtx;
#endif
} pico_lfs_context_t;

// Pico specific hardware abstraction functions

static int pico_hal_read (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void* buffer, lfs_size_t size)
{
    assert(block < c->block_count);
    assert(off + size <= c->block_size);
    // read flash via XIP mapped space
    memcpy(buffer, ((pico_lfs_context_t *)c->context)->nocacheaddr + (block * c->block_size) + off, size);

    return LFS_ERR_OK;
}

static int pico_hal_prog (const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void* buffer, lfs_size_t size)
{
    assert(block < c->block_count);
    // program with SDK
    uint32_t p = (uint32_t)((pico_lfs_context_t *)c->context)->baseaddr + (block * c->block_size) + off;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(p, buffer, size);
    restore_interrupts(ints);

    return LFS_ERR_OK;
}

static int pico_hal_erase (const struct lfs_config *c, lfs_block_t block)
{
    assert(block < c->block_count);
    // erase with SDK
    uint32_t p = (uint32_t)((pico_lfs_context_t *)c->context)->baseaddr + block * c->block_size;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(p, c->block_size);
    restore_interrupts(ints);

    return LFS_ERR_OK;
}

static int pico_hal_sync (const struct lfs_config *c)
{
    (void)c;

    return LFS_ERR_OK;
}

#if LFS_THREADSAFE

static int pico_lock (const struct lfs_config *c)
{
    recursive_mutex_enter_blocking(&((pico_lfs_context_t *)c->context)->fs_mtx);

    return LFS_ERR_OK;
}

static int pico_unlock (const struct lfs_config *c)
{
    recursive_mutex_exit(&((pico_lfs_context_t *)c->context)->fs_mtx);

    return LFS_ERR_OK;
}

#endif

struct lfs_config *pico_littlefs_hal (void)
{
    // File system offsets in flash
    static pico_lfs_context_t ctx = {
        .baseaddr = (const char *)(PICO_FLASH_SIZE_BYTES - FS_SIZE - (32 * 1024)),
        .nocacheaddr = (const char *)(XIP_NOCACHE_NOALLOC_BASE + PICO_FLASH_SIZE_BYTES - FS_SIZE - (32 * 1024))
    };
    
    // Configuration of the filesystem is provided by this struct
    // for Pico: prog size = 256, block size = 4096, so cache is 8K
    // minimum cache = block size, must be multiple
    static struct lfs_config pico_cfg = {
        .context = &ctx,
        // block device operations
        .read = pico_hal_read,
        .prog = pico_hal_prog,
        .erase = pico_hal_erase,
        .sync = pico_hal_sync,
    #if LFS_THREADSAFE
        .lock = pico_lock,
        .unlock = pico_unlock,
    #endif
        // block device configuration
        .read_size = 1,
        .prog_size = FLASH_PAGE_SIZE,
        .block_size = FLASH_SECTOR_SIZE,
        .block_count = FS_SIZE / FLASH_SECTOR_SIZE,
        .cache_size = FLASH_SECTOR_SIZE / 4,
        .lookahead_size = 32,
        .block_cycles = 500
    };

#if LFS_THREADSAFE
    recursive_mutex_init(&ctx.fs_mtx);
#endif

    return &pico_cfg;
}
