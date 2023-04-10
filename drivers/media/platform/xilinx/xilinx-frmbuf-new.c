// SPDX-License-Identifier: GPL-2.0-or-later                                                               
/*
 *Xilinx Framebuffer IP Driver
 *
 * Copyright (C) 2016 - 2021 Xilinx, Inc.
 *
 * Authors: Radhey Shyam Pandey <radheys@xilinx.com>
 *          John Nichols <jnichol@xilinx.com>
 *          Jeffrey Mouroux <jmouroux@xilinx.com>
 *
 * Based on the Freescale DMA driver.
 *
 * Description:
 * The AXI Framebuffer core is a soft Xilinx IP core that
 * provides high-bandwidth direct memory access between memory
 * and AXI4-Stream.
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/slab.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/xilinx-frmbuf-new.h>

#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>

#include "xilinx-vip.h"
#include "xilinx-vipp.h"

static LIST_HEAD(frmbuf_chan_list);
static DEFINE_MUTEX(frmbuf_chan_list_lock);

const struct xilinx_frmbuf_format_desc xilinx_frmbuf_formats[] = {
        {
                .dts_name = "xbgr8888",
                .id = XILINX_FRMBUF_FMT_RGBX8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_BGRX32,
                .fmt_bitmask = BIT(0),
        },
        {
                .dts_name = "xbgr2101010",
                .id = XILINX_FRMBUF_FMT_RGBX10,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XBGR30,
                .fmt_bitmask = BIT(1),
        },
        {
                .dts_name = "xrgb8888",
                .id = XILINX_FRMBUF_FMT_BGRX8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XBGR32,
                .fmt_bitmask = BIT(2),
        },
        {
                .dts_name = "xvuy8888",
                .id = XILINX_FRMBUF_FMT_YUVX8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XVUY32,
                .fmt_bitmask = BIT(5),
        },
        {
                .dts_name = "vuy888",
                .id = XILINX_FRMBUF_FMT_YUV8,
                .bpw = 24,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_VUY24,
                .fmt_bitmask = BIT(6),
        },
        {
                .dts_name = "yuvx2101010",
                .id = XILINX_FRMBUF_FMT_YUVX10,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XVUY10,
                .fmt_bitmask = BIT(7),
        },
        {
                .dts_name = "yuyv",
                .id = XILINX_FRMBUF_FMT_YUYV8,
                .bpw = 32,
                .ppw = 2,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_YUYV,
                .fmt_bitmask = BIT(8),
        },
        {
                .dts_name = "uyvy",
                .id = XILINX_FRMBUF_FMT_UYVY8,
                .bpw = 32,
                .ppw = 2,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_UYVY,
                .fmt_bitmask = BIT(9),
        },
        {
                .dts_name = "nv16",
                .id = XILINX_FRMBUF_FMT_Y_UV8,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_NV16M,
                .fmt_bitmask = BIT(11),
        },
        {
                .dts_name = "nv16",
                .id = XILINX_FRMBUF_FMT_Y_UV8,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_NV16,
                .fmt_bitmask = BIT(11),
        },
        {
                .dts_name = "nv12",
                .id = XILINX_FRMBUF_FMT_Y_UV8_420,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_NV12M,
                .fmt_bitmask = BIT(12),
        },
        {
                .dts_name = "nv12",
                .id = XILINX_FRMBUF_FMT_Y_UV8_420,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_NV12,
                .fmt_bitmask = BIT(12),
        },
        {
                .dts_name = "xv15",
                .id = XILINX_FRMBUF_FMT_Y_UV10_420,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_XV15M,
                .fmt_bitmask = BIT(13),
        },
        {
                .dts_name = "xv15",
                .id = XILINX_FRMBUF_FMT_Y_UV10_420,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_XV15,
                .fmt_bitmask = BIT(13),
        },
        {
                .dts_name = "xv20",
                .id = XILINX_FRMBUF_FMT_Y_UV10,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_XV20M,
                .fmt_bitmask = BIT(14),
        },
        {
                .dts_name = "xv20",
                .id = XILINX_FRMBUF_FMT_Y_UV10,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 2,
                .v4l2_fmt = V4L2_PIX_FMT_XV20,
                .fmt_bitmask = BIT(14),
        },
        {
                .dts_name = "bgr888",
                .id = XILINX_FRMBUF_FMT_RGB8,
                .bpw = 24,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_RGB24,
                .fmt_bitmask = BIT(15),
        },
        {
                .dts_name = "y8",
                .id = XILINX_FRMBUF_FMT_Y8,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_GREY,
                .fmt_bitmask = BIT(16),
        },
        {
                .dts_name = "y10",
                .id = XILINX_FRMBUF_FMT_Y10,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XY10,
                .fmt_bitmask = BIT(17),
        },
        {
                .dts_name = "rgb888",
                .id = XILINX_FRMBUF_FMT_BGR8,
                .bpw = 24,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_BGR24,
                .fmt_bitmask = BIT(18),
        },
        {
                .dts_name = "abgr8888",
                .id = XILINX_FRMBUF_FMT_RGBA8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = 0,
                .fmt_bitmask = BIT(19),
        },
        {
                .dts_name = "argb8888",
                .id = XILINX_FRMBUF_FMT_BGRA8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = 0,
                .fmt_bitmask = BIT(20),
        },
        {
                .dts_name = "avuy8888",
                .id = XILINX_FRMBUF_FMT_YUVA8,
                .bpw = 32,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = 0,
                .fmt_bitmask = BIT(21),
        },
        {
                .dts_name = "xbgr4121212",
                .id = XILINX_FRMBUF_FMT_RGBX12,
                .bpw = 40,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_XBGR40,
                .fmt_bitmask = BIT(22),
        },
        {
                .dts_name = "rgb16",
                .id = XILINX_FRMBUF_FMT_RGB16,
                .bpw = 48,
                .ppw = 1,
                .num_planes = 1,
                .v4l2_fmt = V4L2_PIX_FMT_BGR48,
                .fmt_bitmask = BIT(23),
        },
        {
                .dts_name = "y_u_v8",
                .id = XILINX_FRMBUF_FMT_Y_U_V8,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 3,
                .v4l2_fmt = V4L2_PIX_FMT_YUV444M,
                .fmt_bitmask = BIT(24),
        },
        {
                .dts_name = "y_u_v8",
                .id = XILINX_FRMBUF_FMT_Y_U_V8,
                .bpw = 32,
                .ppw = 4,
                .num_planes = 3,
                .v4l2_fmt = V4L2_PIX_FMT_YUV444P,
                .fmt_bitmask = BIT(24),
        },
        {
                .dts_name = "y_u_v10",
                .id = XILINX_FRMBUF_FMT_Y_U_V10,
                .bpw = 32,
                .ppw = 3,
                .num_planes = 3,
                .v4l2_fmt = V4L2_PIX_FMT_X403,
                .fmt_bitmask = BIT(25),
        },
};

/**
 * struct xilinx_frmbuf_feature - dt or IP property structure
 * @flags: Bitmask of properties enabled in IP or dt
 */
struct xilinx_frmbuf_feature {
        enum fb_transfer_direction direction;
        u32 flags;
};

static const struct xilinx_frmbuf_feature xlnx_fbwr_cfg_v20 = {
        .direction = FB_DEV_TO_MEM,
};

static const struct xilinx_frmbuf_feature xlnx_fbwr_cfg_v21 = {
        .direction = FB_DEV_TO_MEM,
        .flags = XILINX_PPC_PROP | XILINX_FLUSH_PROP
                | XILINX_FID_PROP | XILINX_CLK_PROP,
};

static const struct xilinx_frmbuf_feature xlnx_fbwr_cfg_v22 = {
        .direction = FB_DEV_TO_MEM,
        .flags = XILINX_PPC_PROP | XILINX_FLUSH_PROP
                | XILINX_FID_PROP | XILINX_CLK_PROP
                | XILINX_THREE_PLANES_PROP,
};

static const struct xilinx_frmbuf_feature xlnx_fbrd_cfg_v20 = {
        .direction = FB_MEM_TO_DEV,
};

static const struct xilinx_frmbuf_feature xlnx_fbrd_cfg_v21 = {
        .direction = FB_MEM_TO_DEV,
        .flags = XILINX_PPC_PROP | XILINX_FLUSH_PROP
                | XILINX_FID_PROP | XILINX_CLK_PROP,
};

static const struct xilinx_frmbuf_feature xlnx_fbrd_cfg_v22 = {
        .direction = FB_MEM_TO_DEV,
        .flags = XILINX_PPC_PROP | XILINX_FLUSH_PROP
                | XILINX_FID_PROP | XILINX_CLK_PROP
                | XILINX_THREE_PLANES_PROP
                | XILINX_FID_ERR_DETECT_PROP,
};

static const struct of_device_id xilinx_frmbuf_new_of_ids[] = {
        { .compatible = "xlnx,axi-frmbuf-wr-v200",
                .data = (void *)&xlnx_fbwr_cfg_v20},
        { .compatible = "xlnx,axi-frmbuf-wr-v2.100",
                .data = (void *)&xlnx_fbwr_cfg_v21},
        { .compatible = "xlnx,axi-frmbuf-wr-v2.200",
                .data = (void *)&xlnx_fbwr_cfg_v22},
        { .compatible = "xlnx,axi-frmbuf-rd-v2",
                .data = (void *)&xlnx_fbrd_cfg_v20},
        { .compatible = "xlnx,axi-frmbuf-rd-v2.1",
                .data = (void *)&xlnx_fbrd_cfg_v21},
        { .compatible = "xlnx,axi-frmbuf-rd-v2.2",
                .data = (void *)&xlnx_fbrd_cfg_v22},                                                       
        {/* end of list */}
};

/* -----------------------------------------------------------------------------
 * Helper functions
 */

static inline void frmbuf_write(struct xilinx_frmbuf_chan *chan, u32 reg,
                                 u32 value)
{
        iowrite32(value, chan->frmbuf->regs + reg);
}

static inline void frmbuf_writeq(struct xilinx_frmbuf_chan *chan, u32 reg,
                                 u64 value)
{
        iowrite32(lower_32_bits(value), chan->frmbuf->regs + reg);
        iowrite32(upper_32_bits(value), chan->frmbuf->regs + reg + 4);
}

static inline u32 frmbuf_read(struct xilinx_frmbuf_chan *chan, u32 reg)
{
        return ioread32(chan->frmbuf->regs + reg);
}

static inline void frmbuf_set(struct xilinx_frmbuf_chan *chan, u32 reg,
                              u32 set)
{
        frmbuf_write(chan, reg, frmbuf_read(chan, reg) | set);
}

static inline void frmbuf_clr(struct xilinx_frmbuf_chan *chan, u32 reg,
                               u32 clr)
{
        frmbuf_write(chan, reg, frmbuf_read(chan, reg) & ~clr);
}

static void writeq_addr(struct xilinx_frmbuf_chan *chan, u32 reg,
                         fb_addr_t addr)
{
        frmbuf_writeq(chan, reg, (u64)addr);
}

static void write_addr(struct xilinx_frmbuf_chan *chan, u32 reg,
                       fb_addr_t addr)
{
        frmbuf_write(chan, reg, addr);
}

/**
 * fb_cookie_assign - assign a FB engine cookie to the descriptor
 * @tx: descriptor needing cookie
 *       
 * Assign a unique non-zero per-channel cookie to the descriptor.
 * Note: caller is expected to hold a lock to prevent concurrency.
 */
static inline fb_cookie_t fb_cookie_assign(struct xilinx_frmbuf_chan *chan, struct xilinx_frmbuf_tx_descriptor *desc)
{
        fb_cookie_t cookie;

        cookie = chan->cookie + 1;
        if (cookie < FB_MIN_COOKIE)
                cookie = FB_MIN_COOKIE;
        desc->cookie = chan->cookie = cookie; 

        return cookie;
}

/**
 * fb_cookie_complete - complete a descriptor 
 * @tx: descriptor to complete
 *                           
 * Mark this descriptor complete by updating the channels completed
 * cookie marker.  Zero the descriptors cookie to prevent accidental
 * repeated completions.
 *
 * Note: caller is expected to hold a lock to prevent concurrency.
 */
static inline void fb_cookie_complete(struct xilinx_frmbuf_chan *chan, struct xilinx_frmbuf_tx_descriptor *desc)                             
{
        chan->completed_cookie = desc->cookie; 
        desc->cookie = 0;
}        

/* -----------------------------------------------------------------------------
 * Driver functions
 */

/**
 * xilinx_frmbuf_reset - Reset frmbuf channel
 * @chan: Driver specific frmbuf channel
 */
static void xilinx_frmbuf_reset(struct xilinx_frmbuf_chan *chan)
{
        /* reset ip */
        gpiod_set_value(chan->frmbuf->rst_gpio, 1);
        udelay(1);
        gpiod_set_value(chan->frmbuf->rst_gpio, 0);
}

/**
 * xilinx_frmbuf_chan_reset - Reset frmbuf channel and enable interrupts
 * @chan: Driver specific frmbuf channel
 */
static void xilinx_frmbuf_chan_reset(struct xilinx_frmbuf_chan *chan)
{
        xilinx_frmbuf_reset(chan);
	frmbuf_write(chan, XILINX_FRMBUF_IE_OFFSET, XILINX_FRMBUF_IE_AP_DONE);
	frmbuf_write(chan, XILINX_FRMBUF_GIE_OFFSET, XILINX_FRMBUF_GIE_EN);
        chan->fid_err_flag = 0;
        chan->fid_out_val = 0;
}

/**
 * xilinx_frmbuf_chan_remove - Per Channel remove function
 * @chan: Driver specific frmbuf channel
 */
static void xilinx_frmbuf_chan_remove(struct xilinx_frmbuf_chan *chan)
{
        /* Disable all interrupts */
        frmbuf_clr(chan, XILINX_FRMBUF_IE_OFFSET,
                   XILINX_FRMBUF_ISR_ALL_IRQ_MASK);

        tasklet_kill(&chan->tasklet);

        mutex_lock(&frmbuf_chan_list_lock);
        list_del(&chan->chan_node);
        mutex_unlock(&frmbuf_chan_list_lock);
}

/**
 * xilinx_frmbuf_init_format_array - Start framebuffer channel
 * @chan: Driver specific framebuffer channel
 */
static void xilinx_frmbuf_init_format_array(struct xvip_frmbuf *frmbuf)
{
        u32 i, cnt;
	frmbuf->v4l2_memory_fmts = kzalloc(sizeof(xilinx_frmbuf_formats), GFP_KERNEL);

        for (i = 0; i < ARRAY_SIZE(xilinx_frmbuf_formats); i++) {
                if (!(frmbuf->enabled_vid_fmts &
                      xilinx_frmbuf_formats[i].fmt_bitmask))
                        continue;


                if (xilinx_frmbuf_formats[i].v4l2_fmt) {
                         cnt = frmbuf->v4l2_fmt_cnt++;
                         frmbuf->v4l2_memory_fmts[cnt] =
                                 xilinx_frmbuf_formats[i].v4l2_fmt;
                 }

        }
}

/**
 * xilinx_frmbuf_alloc_chan_resources - Allocate channel resources                                    
 * @chan: FB channel
 *
 * Return: '0' on success and failure value on error
 */
static int xilinx_frmbuf_alloc_chan_resources(struct xilinx_frmbuf_chan *chan)
{
        chan->cookie = FB_MIN_COOKIE;
	chan->completed_cookie = FB_MIN_COOKIE;

        return 0;
}

/**
 * xilinx_frmbuf_halt - Halt frmbuf channel
 * @chan: Driver specific FrameBuffer channel
 */
static void xilinx_frmbuf_halt(struct xilinx_frmbuf_chan *chan)
{
        frmbuf_clr(chan, XILINX_FRMBUF_CTRL_OFFSET,
                   XILINX_FRMBUF_CTRL_AP_START | chan->mode);
        chan->idle = true;
}

/* xilinx_frmbuf_start - Start framebuffer channel
 * @chan: Driver specific framebuffer channel
 */
static void xilinx_frmbuf_start(struct xilinx_frmbuf_chan *chan)
{
        frmbuf_set(chan, XILINX_FRMBUF_CTRL_OFFSET,
                   XILINX_FRMBUF_CTRL_AP_START | chan->mode);
        chan->idle = false;
}

/**
 * xilinx_frmbuf_alloc_tx_descriptor - Allocate transaction descriptor
 * @chan: Driver specific dma channel
 *
 * Return: The allocated descriptor on success and NULL on failure.
 */
static struct xilinx_frmbuf_tx_descriptor *
xilinx_frmbuf_alloc_tx_descriptor(struct xilinx_frmbuf_chan *chan)
{
        struct xilinx_frmbuf_tx_descriptor *desc;

        desc = kzalloc(sizeof(*desc), GFP_KERNEL);
        if (!desc)
                return NULL;

	desc->chan = chan;

        return desc;
}

/**
 * xilinx_frmbuf_free_desc_list - Free descriptors list
 * @chan: Driver specific FrameBuffer channel
 * @list: List to parse and delete the descriptor
 */
static void xilinx_frmbuf_free_desc_list(struct xilinx_frmbuf_chan *chan,
                                         struct list_head *list)
{
        struct xilinx_frmbuf_tx_descriptor *desc, *next;

        list_for_each_entry_safe(desc, next, list, node) {
                list_del(&desc->node);
                kfree(desc);
        }
}

/**
 * xilinx_frmbuf_free_descriptors - Free channel descriptors
 * @chan: Driver specific FrameBuffer channel
 */
static void xilinx_frmbuf_free_descriptors(struct xilinx_frmbuf_chan *chan)
{
        unsigned long flags;

        spin_lock_irqsave(&chan->lock, flags);

        xilinx_frmbuf_free_desc_list(chan, &chan->pending_list);
        xilinx_frmbuf_free_desc_list(chan, &chan->done_list);
        kfree(chan->active_desc);
        kfree(chan->staged_desc);

        chan->staged_desc = NULL;
        chan->active_desc = NULL;
        INIT_LIST_HEAD(&chan->pending_list);
        INIT_LIST_HEAD(&chan->done_list);

        spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * xilinx_frmbuf_free_chan_resources - Free channel resources
 * @dchan: DMA channel
 */
static void xilinx_frmbuf_free_chan_resources(struct xilinx_frmbuf_chan *chan)                                 
{
        xilinx_frmbuf_free_descriptors(chan);
}

/**
 * xilinx_frmbuf_start_transfer - Starts frmbuf transfer
 * @chan: Driver specific channel struct pointer
 */
static void xilinx_frmbuf_start_transfer(struct xilinx_frmbuf_chan *chan)
{
        struct xilinx_frmbuf_tx_descriptor *desc;
        struct xvip_frmbuf *frmbuf;

        frmbuf = chan->frmbuf;

        if (!chan->idle)
                return;

        if (chan->staged_desc) {
                chan->active_desc = chan->staged_desc;
                chan->staged_desc = NULL;
        }

        if (list_empty(&chan->pending_list))
                return;

        desc = list_first_entry(&chan->pending_list,
                                struct xilinx_frmbuf_tx_descriptor,
                                node);
        if (desc->earlycb == EARLY_CALLBACK_START_DESC) {
                fb_tx_callback callback;
                void *callback_param;

                callback = desc->callback;
                callback_param = desc->callback_param;
                if (callback) {
                        callback(callback_param);
                        desc->callback = NULL;
                        chan->active_desc = desc;
                }
        }

       /* Start the transfer */
        chan->write_addr(chan, XILINX_FRMBUF_ADDR_OFFSET,
                         desc->hw.luma_plane_addr);
        chan->write_addr(chan, XILINX_FRMBUF_ADDR2_OFFSET,
                         desc->hw.chroma_plane_addr[0]);
        if (frmbuf->cfg->flags & XILINX_THREE_PLANES_PROP) {
                if (chan->direction == FB_MEM_TO_DEV)
                        chan->write_addr(chan, XILINX_FRMBUF_RD_ADDR3_OFFSET,
                                         desc->hw.chroma_plane_addr[1]);
                else
                        chan->write_addr(chan, XILINX_FRMBUF_ADDR3_OFFSET,
                                         desc->hw.chroma_plane_addr[1]);
        }

        /* HW expects these parameters to be same for one transaction */
        frmbuf_write(chan, XILINX_FRMBUF_WIDTH_OFFSET, desc->hw.hsize);
        frmbuf_write(chan, XILINX_FRMBUF_STRIDE_OFFSET, desc->hw.stride);
        frmbuf_write(chan, XILINX_FRMBUF_HEIGHT_OFFSET, desc->hw.vsize);
        frmbuf_write(chan, XILINX_FRMBUF_FMT_OFFSET, desc->hw.fmt_id);

        /* If it is framebuffer read IP set the FID */
        if (chan->direction == FB_MEM_TO_DEV && chan->hw_fid)
                frmbuf_write(chan, XILINX_FRMBUF_FID_OFFSET, desc->fid);

        /* Start the hardware */
        xilinx_frmbuf_start(chan);
        list_del(&desc->node);

        /* No staging descriptor required when auto restart is disabled */
        if (chan->mode == AUTO_RESTART)
                chan->staged_desc = desc;
        else
                chan->active_desc = desc;
}

/* xilinx_frmbuf_issue_pending - Issue pending transactions
 * @chan: FrameBuffer channel
 */
static void xilinx_frmbuf_issue_pending(struct xilinx_frmbuf_chan *chan)
{
        unsigned long flags;

        spin_lock_irqsave(&chan->lock, flags);
        xilinx_frmbuf_start_transfer(chan);
        spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * xilinx_frmbuf_tx_submit - Submit FrameBuffer transaction
 *
 */
static void xilinx_frmbuf_tx_submit(struct xilinx_frmbuf_chan *chan ,struct xilinx_frmbuf_tx_descriptor *desc)
{
        unsigned long flags;
	fb_cookie_t cookie;

        spin_lock_irqsave(&chan->lock, flags);
	cookie = fb_cookie_assign(chan, desc);	
        list_add_tail(&desc->node, &chan->pending_list);
        spin_unlock_irqrestore(&chan->lock, flags);

	return;

}

/**
* xilinx_frmbuf_prep_interleaved - prepare a descriptor for a
*      DMA_SLAVE transaction
* @dchan: DMA channel
* @xt: Interleaved template pointer
* @flags: transfer ack flags
*
* Return: Async transaction descriptor on success and NULL on failure
*/
static struct xilinx_frmbuf_tx_descriptor *
xilinx_frmbuf_prep_interleaved(struct xilinx_frmbuf_chan *chan,
				   struct fb_interleaved_template *xt,
                                   unsigned long flags)
{
        struct xilinx_frmbuf_tx_descriptor *desc;
        struct xilinx_frmbuf_desc_hw *hw;
        u32 vsize, hsize;

        if (chan->direction != xt->dir || !chan->vid_fmt)
                goto error;

        if (!xt->numf || !xt->sgl[0].size)
                goto error;
                                                                                                      
        if (xt->frame_size != chan->vid_fmt->num_planes)
                goto error;


        vsize = xt->numf;
        hsize = (xt->sgl[0].size * chan->vid_fmt->ppw * 8) /
                 chan->vid_fmt->bpw;
        /* hsize calc should not have resulted in an odd number */
        if (hsize & 1)
                hsize++;

        if (vsize > chan->xdev->max_height || hsize > chan->xdev->max_width) {
                dev_dbg(chan->xdev->dev,
                        "vsize %d max vsize %d hsize %d max hsize %d\n",
                        vsize, chan->xdev->max_height, hsize,
                        chan->xdev->max_width);
                dev_err(chan->xdev->dev, "Requested size not supported!\n");
                goto error;
        }

        desc = xilinx_frmbuf_alloc_tx_descriptor(chan);
        if (!desc)
                return NULL;

        hw = &desc->hw;
        hw->vsize = xt->numf;
        hw->stride = xt->sgl[0].icg + xt->sgl[0].size;
        hw->hsize = (xt->sgl[0].size * chan->vid_fmt->ppw * 8) /
                     chan->vid_fmt->bpw;
        hw->fmt_id = chan->vid_fmt->id;

        /* hsize calc should not have resulted in an odd number */
        if (hw->hsize & 1)
                hw->hsize++;

        if (chan->direction == DMA_MEM_TO_DEV) {
                hw->luma_plane_addr = xt->src_start;
                if (xt->frame_size == 2 || xt->frame_size == 3)
                        hw->chroma_plane_addr[0] =
                                xt->src_start +
                                xt->numf * hw->stride +
                                xt->sgl[0].src_icg;
                if (xt->frame_size == 3)
                        hw->chroma_plane_addr[1] =
                                hw->chroma_plane_addr[0] +
                                xt->numf * hw->stride +
                                xt->sgl[0].src_icg;
        } else {
                hw->luma_plane_addr = xt->dst_start;
                if (xt->frame_size == 2 || xt->frame_size == 3)
                        hw->chroma_plane_addr[0] =
                                xt->dst_start +
                                xt->numf * hw->stride +
                                xt->sgl[0].dst_icg;
                if (xt->frame_size == 3)
                        hw->chroma_plane_addr[1] =
                                hw->chroma_plane_addr[0] +
                                xt->numf * hw->stride +
                                xt->sgl[0].dst_icg;
        }

        return desc;

error:
        dev_err(chan->xdev->dev,
                "Invalid frmbuf template or missing frmbuf video fmt config\n");
        return NULL;
}

/**
 * xilinx_frmbuf_chan_desc_cleanup - Clean channel descriptors
 * @chan: Driver specific framebuffer channel
 */
static void xilinx_frmbuf_chan_desc_cleanup(struct xilinx_frmbuf_chan *chan)
{
        struct xilinx_frmbuf_tx_descriptor *desc, *next;
        unsigned long flags;

        spin_lock_irqsave(&chan->lock, flags);

        list_for_each_entry_safe(desc, next, &chan->done_list, node) {
                void *callback_param;

                list_del(&desc->node);

                kfree(desc);
        }

        spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * xilinx_frmbuf_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the Xilinx frmbuf channel structure
 */
static void xilinx_frmbuf_do_tasklet(unsigned long data)
{
        struct xilinx_frmbuf_chan *chan = (struct xilinx_frmbuf_chan *)data;

        xilinx_frmbuf_chan_desc_cleanup(chan);
}

/**
 * xilinx_frmbuf_complete_descriptor - Mark the active descriptor as complete
 * This function is invoked with spinlock held
 * @chan : xilinx frmbuf channel
 *
 * CONTEXT: hardirq
 */
static void xilinx_frmbuf_complete_descriptor(struct xilinx_frmbuf_chan *chan)
{
        struct xilinx_frmbuf_tx_descriptor *desc = chan->active_desc;

        /*
         * In case of frame buffer write, read the fid register
         * and associate it with descriptor
         */
        if (chan->direction == FB_DEV_TO_MEM && chan->hw_fid)
                desc->fid = frmbuf_read(chan, XILINX_FRMBUF_FID_OFFSET) &
                            XILINX_FRMBUF_FID_MASK;

	fb_cookie_complete(chan, desc);
        list_add_tail(&desc->node, &chan->done_list);
}

/**
 * xilinx_frmbuf_irq_handler - frmbuf Interrupt handler
 * @irq: IRQ number
 * @data: Pointer to the Xilinx frmbuf channel structure
 *
 * Return: IRQ_HANDLED/IRQ_NONE
 */
static irqreturn_t xilinx_frmbuf_irq_handler(int irq, void *data)
{
        struct xilinx_frmbuf_chan *chan = data;
        u32 status;
        fb_tx_callback callback = NULL;
        void *callback_param;
        struct xilinx_frmbuf_tx_descriptor *desc;

        status = frmbuf_read(chan, XILINX_FRMBUF_ISR_OFFSET);
        if (!(status & XILINX_FRMBUF_ISR_ALL_IRQ_MASK))
                return IRQ_NONE;

        frmbuf_write(chan, XILINX_FRMBUF_ISR_OFFSET,
                     status & XILINX_FRMBUF_ISR_ALL_IRQ_MASK);

        /* Check if callback function needs to be called early */
        desc = chan->staged_desc;
        if (desc && desc->earlycb == EARLY_CALLBACK) {
                callback = desc->callback;
                callback_param = desc->callback_param;
                if (callback) {
                        callback(callback_param);
                        desc->callback = NULL;
                }
        }

        if (status & XILINX_FRMBUF_ISR_AP_DONE_IRQ) {
                spin_lock(&chan->lock);
                chan->idle = true;
                if (chan->active_desc) {
                        xilinx_frmbuf_complete_descriptor(chan);
                        chan->active_desc = NULL;
                }

                /* Update fid err detect flag and out value */
                if (chan->direction == FB_MEM_TO_DEV &&
                    chan->hw_fid && chan->idle &&
                    chan->frmbuf->cfg->flags & XILINX_FID_ERR_DETECT_PROP) {
                        if (chan->mode == AUTO_RESTART)
                                chan->fid_mode = FID_MODE_2;
                        else
                                chan->fid_mode = FID_MODE_1;

                        frmbuf_write(chan, XILINX_FRMBUF_FID_MODE_OFFSET,
                                     chan->fid_mode);
                        dev_dbg(chan->frmbuf->dev, "fid mode = %d\n",
                                frmbuf_read(chan, XILINX_FRMBUF_FID_MODE_OFFSET));

                        chan->fid_err_flag = frmbuf_read(chan,
                                                         XILINX_FRMBUF_FID_ERR_OFFSET) &
                                                        XILINX_FRMBUF_FID_ERR_MASK;
                        chan->fid_out_val = frmbuf_read(chan,
                                                        XILINX_FRMBUF_FID_OUT_OFFSET) &
                                                        XILINX_FRMBUF_FID_OUT_MASK;
                        dev_dbg(chan->frmbuf->dev, "fid err cnt = 0x%x\n",
                                frmbuf_read(chan, XILINX_FRMBUF_FID_ERR_OFFSET));
                }

                xilinx_frmbuf_start_transfer(chan);
                spin_unlock(&chan->lock);
        }

        tasklet_schedule(&chan->tasklet);
        return IRQ_HANDLED;
}

/**
 * xilinx_frmbuf_terminate_all - Halt the channel and free descriptors
 * @chan: Driver specific FrameBuffer channel pointer
 *
 * Return: 0
 */
static int xilinx_frmbuf_terminate_all(struct xilinx_frmbuf_chan *chan)
{
        xilinx_frmbuf_halt(chan);
        xilinx_frmbuf_free_descriptors(chan);
        /* worst case frame-to-frame boundary; ensure frame output complete */
        msleep(50);

        if (chan->frmbuf->cfg->flags & XILINX_FLUSH_PROP) {
                u8 count;

                /*
                 * Flush the framebuffer FIFO and
                 * wait for max 50ms for flush done
                 */
                frmbuf_set(chan, XILINX_FRMBUF_CTRL_OFFSET,
                           XILINX_FRMBUF_CTRL_FLUSH);
                for (count = WAIT_FOR_FLUSH_DONE; count > 0; count--) {
                        if (frmbuf_read(chan, XILINX_FRMBUF_CTRL_OFFSET) &
                                        XILINX_FRMBUF_CTRL_FLUSH_DONE)
                                break;
                        usleep_range(2000, 2100);
                }

                if (!count)
                        dev_err(chan->frmbuf->dev, "Framebuffer Flush not done!\n");
        }

        xilinx_frmbuf_chan_reset(chan);

        return 0;
}

/**
 * xilinx_frmbuf_synchronize - kill tasklet to stop further descr processing                          
 * @chan: Driver specific FB channel pointer
 */
static void xilinx_frmbuf_synchronize(struct xilinx_frmbuf_chan *chan)
{
        tasklet_kill(&chan->tasklet);
}

/**
 * xilinx_frmbuf_chan_probe - Per Channel Probing
 * It get channel features from the device tree entry and
 * initialize special channel handling routines
 *
 * @node: Device node
 *
 * Return: '0' on success and failure value on error
 */
static int xilinx_frmbuf_chan_probe(struct xvip_frmbuf *frmbuf,
                                    struct device_node *node)
{
        struct xilinx_frmbuf_chan *chan;
        u32 fb_addr_size = 0;
        int err;

        chan = &frmbuf->chan;

        chan->dev = frmbuf->dev;

        chan->frmbuf = frmbuf;

        chan->idle = true;
        chan->fid_err_flag = 0;
        chan->fid_out_val = 0;
        chan->mode = AUTO_RESTART;

        if (frmbuf->cfg->flags & XILINX_FID_PROP)
                chan->hw_fid = of_property_read_bool(node, "xlnx,fid");


        err = of_property_read_u32(node, "xlnx,fb-addr-width",
                                   &fb_addr_size);
        if (err || (fb_addr_size != 32 && fb_addr_size != 64)) {
                dev_err(frmbuf->dev, "missing or invalid addr width dts prop\n");
                return err;
        }

        if (fb_addr_size == 64 && sizeof(fb_addr_t) == sizeof(u64))
                chan->write_addr = writeq_addr;
        else
                chan->write_addr = write_addr;

        spin_lock_init(&chan->lock);
        INIT_LIST_HEAD(&chan->pending_list);
        INIT_LIST_HEAD(&chan->done_list);

        chan->irq = irq_of_parse_and_map(node, 0);
        err = devm_request_irq(frmbuf->dev, chan->irq, xilinx_frmbuf_irq_handler,
                               IRQF_SHARED, "xilinx_framebuffer_new", chan);

        if (err) {
                dev_err(frmbuf->dev, "unable to request IRQ %d\n", chan->irq);
                return err;
        }

        tasklet_init(&chan->tasklet, xilinx_frmbuf_do_tasklet,
                     (unsigned long)chan);

        mutex_lock(&frmbuf_chan_list_lock);
        list_add_tail(&chan->chan_node, &frmbuf_chan_list);
        mutex_unlock(&frmbuf_chan_list_lock);

        xilinx_frmbuf_chan_reset(chan);

        return 0;
}

/**
 * xilinx_frmbuf_parse_dt_node - Parse frmbuf DT node
 * @pdev:
 *
 * Return: '0' on success and failure value on error
 */

int xilinx_frmbuf_parse_dt_node(struct xvip_frmbuf *frmbuf,struct device_node *node)
{
       int ret;
        struct resource *io;
        struct platform_device *pdev;
        const struct of_device_id *match;
        enum fb_transfer_direction fb_dir;
        int err;
        u32 i, j, max_width, max_height;
        int hw_vid_fmt_cnt;
        const char *vid_fmts[ARRAY_SIZE(xilinx_frmbuf_formats)];

        pdev = of_find_device_by_node(node);
        if (!pdev) {
               ret = -EPROBE_DEFER;
                dev_info(frmbuf->dev, "failed to find pdev by node (%d)\n",ret);
                goto error;
        }

       frmbuf->dev = &pdev->dev;

       match = of_match_node(xilinx_frmbuf_new_of_ids, node);
        if (!match)
        {
               ret = -ENODEV;
                dev_info(frmbuf->dev, "failed to match frmbuf compat string (%d)\n",ret);
                goto error;
        }

        frmbuf->cfg = match->data;

        fb_dir = (enum fb_transfer_direction)frmbuf->cfg->direction;

        if (frmbuf->cfg->flags & XILINX_CLK_PROP) {
                frmbuf->ap_clk = devm_clk_get(frmbuf->dev, "ap_clk");
                if (IS_ERR(frmbuf->ap_clk)) {
                        err = PTR_ERR(frmbuf->ap_clk);
                        dev_err(frmbuf->dev, "failed to get ap_clk (%d)\n", err);
                        of_node_put(node);
                        return err;
                }
        } else {
                dev_info(frmbuf->dev, "assuming clock is enabled!\n");
        }

       frmbuf->rst_gpio = devm_gpiod_get(&pdev->dev, "reset",
                                        GPIOD_OUT_HIGH);
        if (IS_ERR(frmbuf->rst_gpio)) {
                err = PTR_ERR(frmbuf->rst_gpio);
                if (err == -EPROBE_DEFER)
                        dev_info(&pdev->dev,
                                 "new frmbuf Probe deferred due to GPIO reset defer %s %d \n",__FUNCTION__,__LINE__);
                else
                        dev_err(&pdev->dev,
                                "Unable to locate reset property in dt\n");

                return err;
        }

        gpiod_set_value_cansleep(frmbuf->rst_gpio, 0x0);

        io = platform_get_resource(pdev, IORESOURCE_MEM, 0);
        frmbuf->regs = devm_ioremap_resource(&pdev->dev, io);
        if (IS_ERR(frmbuf->regs))
                return PTR_ERR(frmbuf->regs);

        if (frmbuf->cfg->flags & XILINX_THREE_PLANES_PROP)
                max_height = 8640;
        else
                max_height = 4320;

        err = of_property_read_u32(node, "xlnx,max-height", &frmbuf->max_height);
        if (err < 0) {
               ret = -EINVAL;
                dev_err(frmbuf->dev, "xlnx,max-height is missing! (%d)\n ",ret);
               goto error;
        } else if (frmbuf->max_height > max_height ||
                   frmbuf->max_height < XILINX_FRMBUF_MIN_HEIGHT) {
               ret = -EINVAL;
                dev_err(&pdev->dev, "Invalid height in dt");
               goto error;
        }

        if (frmbuf->cfg->flags & XILINX_THREE_PLANES_PROP)
                max_width = 15360;
        else
                max_width = 8192;

        err = of_property_read_u32(node, "xlnx,max-width", &frmbuf->max_width);
        if (err < 0) {
               ret = -EINVAL;
                dev_err(frmbuf->dev, "xlnx,max-width is missing!");
                goto error;
        } else if (frmbuf->max_width > max_width ||
                   frmbuf->max_width < XILINX_FRMBUF_MIN_WIDTH) {
               ret = -EINVAL;
                dev_err(&pdev->dev, "Invalid width in dt");
                goto error;
        }

        if (frmbuf->cfg->flags & XILINX_PPC_PROP) {
                err = of_property_read_u32(node, "xlnx,pixels-per-clock", &frmbuf->ppc);
                if (err || (frmbuf->ppc != 1 && frmbuf->ppc != 2 &&
                            frmbuf->ppc != 4 && frmbuf->ppc != 8)) {
                        dev_err(&pdev->dev, "missing or invalid pixels per clock dts prop\n");
                        return err;
                }
                err = of_property_read_u32(node, "xlnx,fb-align", &frmbuf->align);
                if (err)
                        frmbuf->align = frmbuf->ppc * XILINX_FRMBUF_ALIGN_MUL;

                if (frmbuf->align < (frmbuf->ppc * XILINX_FRMBUF_ALIGN_MUL) ||
                    ffs(frmbuf->align) != fls(frmbuf->align)) {
                       ret = -EINVAL;
                        dev_err(&pdev->dev, "invalid fb align dts prop\n");
                        goto error;
                }
        }
        if (frmbuf->cfg->flags & XILINX_CLK_PROP) {
                err = clk_prepare_enable(frmbuf->ap_clk);
                if (err) {
                        dev_err(frmbuf->dev, " failed to enable ap_clk (%d)\n",
                                err);
                        return err;
                }
        }
        /* Initialize the channels */
        err = xilinx_frmbuf_chan_probe(frmbuf, node);
        if (err < 0)
                goto disable_clk;

        frmbuf->chan.direction = fb_dir;

        /* read supported video formats and update internal table */
        hw_vid_fmt_cnt = of_property_count_strings(node, "xlnx,vid-formats");

        err = of_property_read_string_array(node, "xlnx,vid-formats",
                                            vid_fmts, hw_vid_fmt_cnt);
        if (err < 0) {
                dev_err(frmbuf->dev,
                        "Missing or invalid xlnx,vid-formats dts prop\n");
                goto remove_chan;
        }

        for (i = 0; i < hw_vid_fmt_cnt; i++) {
                const char *vid_fmt_name = vid_fmts[i];

                for (j = 0; j < ARRAY_SIZE(xilinx_frmbuf_formats); j++) {
                        const char *dts_name =
                                xilinx_frmbuf_formats[j].dts_name;

                        if (strcmp(vid_fmt_name, dts_name))
                                continue;

                        frmbuf->enabled_vid_fmts |=
                                xilinx_frmbuf_formats[j].fmt_bitmask;
                }
        }

        /* Determine supported vid framework formats */
        xilinx_frmbuf_init_format_array(frmbuf);

        platform_set_drvdata(pdev, frmbuf);

	xilinx_frmbuf_alloc_chan_resources(&frmbuf->chan);

        dev_info(frmbuf->dev, "Xilinx NEW FrameBuffer Channels Intialized!!\n");

        return 0;

error:
       of_node_put(node);
       return ret;

remove_chan:
       of_node_put(node);
        xilinx_frmbuf_chan_remove(&frmbuf->chan);

disable_clk:
        clk_disable_unprepare(frmbuf->ap_clk);
        of_node_put(node);
        return err;

}

/**
 * xilinx_frmbuf_init - Initialize framebuffer structure
 * @pdev:
 *
 * Return: '0' on success and failure value on error
 */

int xilinx_frmbuf_init(struct xvip_composite_device *xdev, struct xvip_frmbuf *frmbuf, struct device_node *node)
{
        int ret;

        frmbuf->xdev = xdev;

        /* ... and the frmbuf channel. */
        ret = xilinx_frmbuf_parse_dt_node(frmbuf,node);
        if(ret < 0) {

                dev_err(frmbuf->dev, "%pOF initialization failed\n", node);
                return ret;
        }

        return 0;

} EXPORT_SYMBOL(xilinx_frmbuf_init);

/* -----------------------------------------------------------------------------
 * Platform Device Driver 
 */
             
/**
 * xilinx_frmbuf_new_probe - Driver probe function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: '0' on success and failure value on error
 */
static int xilinx_frmbuf_new_probe(struct platform_device *pdev)
{
        const struct of_device_id *match;
        struct device_node *node = pdev->dev.of_node;

        match = of_match_node(xilinx_frmbuf_new_of_ids, node);
        if (!match)
                 return -ENODEV;

        dev_info(&pdev->dev, "Xilinx AXI NEW-FrameBuffer Engine Driver Pdev created..pdev= 0x%08x pdev->name=%s!!\n", pdev,pdev->name);

        return 0;
}

/**
 * xilinx_frmbuf_remove - Driver remove function
 * @pdev: Pointer to the platform_device structure
 *
 * Return: Always '0'
 */
static int xilinx_frmbuf_new_remove(struct platform_device *pdev)
{
        struct xvip_frmbuf *frmbuf = platform_get_drvdata(pdev);

	xilinx_frmbuf_free_chan_resources(&frmbuf->chan);
        xilinx_frmbuf_chan_remove(&frmbuf->chan);
        clk_disable_unprepare(frmbuf->ap_clk);

        return 0;
}

MODULE_DEVICE_TABLE(of, xilinx_frmbuf_new_of_ids);

static struct platform_driver xilinx_frmbuf_new_driver = {
        .driver = {
                .name = "xilinx-frmbuf-new",
                .of_match_table = xilinx_frmbuf_new_of_ids,
        },
        .probe = xilinx_frmbuf_new_probe,
        .remove = xilinx_frmbuf_new_remove,
};

module_platform_driver(xilinx_frmbuf_new_driver);

MODULE_AUTHOR("Xilinx, Inc.");
MODULE_DESCRIPTION("Xilinx NEW Framebuffer driver");
MODULE_LICENSE("GPL v2");                     
