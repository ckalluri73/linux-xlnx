// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *Xilinx Framebuffer IP Driver
 *
 * Copyright (C) 2016 - 2021 Xilinx, Inc.
 *
 * Authors: Radhey Shyam Pandey <radheys@xilinx.com>
 *          John Nichols <jnichol@xilinx.com>
 *          Jeffrey Mouroux <jmouroux@xilinx.com>
 *          Sai Hari Chandana Kalluri <kalluri@amd.com>
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
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>
#include <media/v4l2-ioctl.h>

#include <linux/xilinx-frmbuf-new.h>

#include "xilinx-vip.h"
#include "xilinx-vipp.h"

static LIST_HEAD(frmbuf_chan_list);
static DEFINE_MUTEX(frmbuf_chan_list_lock);

/*
 * Select the mode of operation for pipeline that have multiple output FRMBUF
 * engines.
 *
 * @XVIP_FB_MULTI_OUT_MODE_SYNC: Wait for all outputs to be started before
 *      starting the pipeline
 * @XVIP_FB_MULTI_OUT_MODE_ASYNC: Start pipeline branches independently when
 *      outputs are started
 */
enum {
        XVIP_FB_MULTI_OUT_MODE_SYNC = 0,
        XVIP_FB_MULTI_OUT_MODE_ASYNC = 1,
};

static int xvip_frmbuf_multi_out_mode = 0;
module_param_named(multi_out_mode, xvip_frmbuf_multi_out_mode, int, 0444);
MODULE_PARM_DESC(multi_out_mode, "Multi-output FRMBUF mode (0: sync, 1: async)");

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

static int frmbuf_verify_format(struct xilinx_frmbuf_chan *chan, u32 fourcc, u32 type)
{
        u32 i, sz = ARRAY_SIZE(xilinx_frmbuf_formats);

        for (i = 0; i < sz; i++) {
                if ((type == XDMA_V4L2 &&
                    fourcc != xilinx_frmbuf_formats[i].v4l2_fmt))
                        continue;

                if (!(xilinx_frmbuf_formats[i].fmt_bitmask &
                      chan->frmbuf->enabled_vid_fmts))
                        return -EINVAL;

                chan->vid_fmt = &xilinx_frmbuf_formats[i];
                return 0;
        }
        return -EINVAL;
}

static void xilinx_frmbuf_set_config(struct xilinx_frmbuf_chan *chan, u32 fourcc, u32 type)
{
        const struct xilinx_frmbuf_format_desc *old_vid_fmt;
        int ret;
        struct xvip_frmbuf *frmbuf;

        frmbuf = chan->frmbuf;

        /* Save old video format */
        old_vid_fmt = chan->vid_fmt;

        ret = frmbuf_verify_format(chan, fourcc, type);
        if (ret == -EINVAL) {
                dev_err(frmbuf->xdev->dev,
                        "Framebuffer not configured for fourcc 0x%x\n",
                        fourcc);
                return;
        }

        if ((!(frmbuf->cfg->flags & XILINX_THREE_PLANES_PROP)) &&
            (chan->vid_fmt->id == XILINX_FRMBUF_FMT_Y_U_V8 ||
             chan->vid_fmt->id == XILINX_FRMBUF_FMT_Y_U_V10)) {
                dev_err(chan->dev, "doesn't support %s format\n",
                        chan->vid_fmt->dts_name);
                /* Restore to old video format */
                chan->vid_fmt = old_vid_fmt;
                return;
        }
}

static void xilinx_frmbuf_set_mode(struct xilinx_frmbuf_chan *chan, enum operation_mode mode)
{
       if (IS_ERR(chan))
                return;

        chan->mode = mode;
        return;

}

static void xilinx_frmbuf_v4l2_config(struct xilinx_frmbuf_chan *chan, u32 v4l2_fourcc)
{
        xilinx_frmbuf_set_config(chan, v4l2_fourcc, XDMA_V4L2);

}

static int xilinx_frmbuf_get_v4l2_vid_fmts(struct xilinx_frmbuf_chan *chan, u32 *fmt_cnt,
                                  u32 **fmts)
{
        struct xvip_frmbuf *frmbuf;

        frmbuf = chan->frmbuf;

        if (IS_ERR(frmbuf))
                return PTR_ERR(frmbuf);

        *fmt_cnt = frmbuf->v4l2_fmt_cnt;
        *fmts = frmbuf->v4l2_memory_fmts;

        return 0;
}

static struct v4l2_subdev *
xvip_frmbuf_remote_subdev(struct media_pad *local, u32 *pad)
{
        struct media_pad *remote;

        remote = media_pad_remote_pad_first(local);
        if (!remote || !is_media_entity_v4l2_subdev(remote->entity)) {
		 return NULL;
	}

        if (pad)
                *pad = remote->index;

        return media_entity_to_v4l2_subdev(remote->entity);
}

static int xilinx_frmbuf_get_fid(struct xilinx_frmbuf_chan *chan, struct xilinx_frmbuf_tx_descriptor * desc, u32 *fid)
{

	if (!fid)
		return -EINVAL;

        if (chan->direction != FB_DEV_TO_MEM)
                return -EINVAL;

        if (!desc)
                return -EINVAL;

        *fid = desc->fid;
        return 0;
}

static int xilinx_frmbuf_set_fid(struct xilinx_frmbuf_chan *chan,
                        struct xilinx_frmbuf_tx_descriptor *desc, u32 fid)
{
        if (fid > 1){
                return -EINVAL;
	}

        if (chan->direction != FB_MEM_TO_DEV){
                return -EINVAL;
	}

        if (!desc){
                return -EINVAL;
	}

        desc->fid = fid;
        return 0;
}

static int xilinx_frmbuf_set_earlycb(struct xilinx_frmbuf_tx_descriptor *desc,
                            u32 earlycb)
{
        if (!desc)
                return -EINVAL;

        desc->earlycb = earlycb;
        return 0;
}

static int xvip_frmbuf_verify_format(struct xvip_frmbuf *frmbuf)
{
        struct v4l2_subdev_format fmt;
        struct v4l2_subdev *subdev;
        int ret;

        subdev = xvip_frmbuf_remote_subdev(&frmbuf->pad, &fmt.pad);
        if (!subdev){
                return -EPIPE;
	}

        fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;
        ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
        if (ret < 0)
                return ret == -ENOIOCTLCMD ? -EINVAL : ret;

        if (frmbuf->fmtinfo->code != fmt.format.code){
                return -EINVAL;
	}

        /*
         * Crop rectangle contains format resolution by default, and crop
         * rectangle if s_selection is executed.
         */
        if (frmbuf->r.width != fmt.format.width ||
            frmbuf->r.height != fmt.format.height) {
                return -EINVAL;
	}

        if (fmt.format.field != frmbuf->format.field) {
                dev_dbg(frmbuf->xdev->dev, "%s(): field mismatch %u != %u\n",
                        __func__, fmt.format.field, frmbuf->format.field);
                return -EINVAL;
        }

        return 0;
}

int xilinx_frmbuf_chan_get_width_align(struct xilinx_frmbuf_chan *chan, u32 *width_align)
{
        struct xvip_frmbuf *frmbuf;

	frmbuf = chan->frmbuf;
        if (IS_ERR(frmbuf))
                return PTR_ERR(frmbuf);
        *width_align = frmbuf->ppc;

        return 0;
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
 * Framebuffer Driver functions
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
 * @chan: Driver specific frmbuf channel
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
 * @dchan: FMRBUF channel
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
*      FRMBUF_SLAVE transaction
* @dchan: FRMBUF channel
* @xt: Interleaved template pointer
* @flags: transfer ack flags
*
* Return: Async transaction descriptor on success and NULL on failure
*/
static struct xilinx_frmbuf_tx_descriptor *
xilinx_frmbuf_prep_interleaved(struct xilinx_frmbuf_chan *chan,
				   struct frmbuf_interleaved_template *xt,
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

        if (vsize > chan->frmbuf->max_height || hsize > chan->frmbuf->max_width) {
                dev_dbg(chan->frmbuf->dev,
                        "vsize %d max vsize %d hsize %d max hsize %d\n",
                        vsize, chan->frmbuf->max_height, hsize,
                        chan->frmbuf->max_width);
                dev_err(chan->frmbuf->dev, "Requested size not supported!\n");
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

        if (chan->direction == FB_MEM_TO_DEV) {
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
        dev_err(chan->frmbuf->dev,
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

                spin_unlock_irqrestore(&chan->lock, flags);
	        xvip_frmbuf_complete(desc->callback_param);
                spin_lock_irqsave(&chan->lock, flags);

                kfree(desc);
        }

        spin_unlock_irqrestore(&chan->lock, flags);
}

/**
 * xilinx_frmbuf_new_do_tasklet - Schedule completion tasklet
 * @data: Pointer to the Xilinx frmbuf channel structure
 */
static void xilinx_frmbuf_new_do_tasklet(unsigned long data)
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
static irqreturn_t xilinx_frmbuf_new_irq_handler(int irq, void *data)
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
 * xvip_frmbuf_cleanup - Unregister video, media entity and frmbuf chan
 * @frmbuf: frmbuf channel associated with the pipeline
 */
void xvip_frmbuf_cleanup(struct xvip_frmbuf *frmbuf)
{
        if (video_is_registered(&frmbuf->video))
                video_unregister_device(&frmbuf->video);

        if (!IS_ERR_OR_NULL(&frmbuf->chan))
                xilinx_frmbuf_chan_remove(&frmbuf->chan);

        v4l2_ctrl_handler_free(&frmbuf->ctrl_handler);
        media_entity_cleanup(&frmbuf->video.entity);

        mutex_destroy(&frmbuf->lock);
        mutex_destroy(&frmbuf->pipe.lock);

} EXPORT_SYMBOL(xvip_frmbuf_cleanup);


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
        err = devm_request_irq(frmbuf->dev, chan->irq, xilinx_frmbuf_new_irq_handler,
                               IRQF_SHARED, "xilinx_framebuffer_new", chan);

        if (err) {
		dev_err(frmbuf->dev, "unable to request IRQ %d\n", chan->irq);
                return err;
        }

        tasklet_init(&chan->tasklet, xilinx_frmbuf_new_do_tasklet,
                     (unsigned long)chan);

        mutex_lock(&frmbuf_chan_list_lock);
        list_add_tail(&chan->chan_node, &frmbuf_chan_list);
        mutex_unlock(&frmbuf_chan_list_lock);

        xilinx_frmbuf_chan_reset(chan);

        return 0;
}

/* -----------------------------------------------------------------------------
 * Buffer Handling
 */

static void xvip_frmbuf_complete(void *param)
{
        struct xvip_fb_buffer *buf = param;
        struct xvip_frmbuf *frmbuf = buf->frmbuf;
        unsigned int i;
        u32 fid;
        int status;

        spin_lock(&frmbuf->queued_lock);
        list_del(&buf->queue);
        spin_unlock(&frmbuf->queued_lock);

        buf->buf.field = V4L2_FIELD_NONE;                                                             
        buf->buf.sequence = frmbuf->sequence++;
        buf->buf.vb2_buf.timestamp = ktime_get_ns();
        
        status = xilinx_frmbuf_get_fid(&frmbuf->chan, buf->desc, &fid);
        if (!status) {
                if (frmbuf->format.field == V4L2_FIELD_ALTERNATE) {
                        /*
                         * fid = 1 is odd field i.e. V4L2_FIELD_TOP.
                         * fid = 0 is even field i.e. V4L2_FIELD_BOTTOM.
                         */
                        buf->buf.field = fid ?
                                         V4L2_FIELD_TOP : V4L2_FIELD_BOTTOM;
                        
                        if (fid == frmbuf->prev_fid) 
                                buf->buf.sequence = frmbuf->sequence++;
                        
                        buf->buf.sequence >>= 1;
                        frmbuf->prev_fid = fid;
                }
        }
        
        for (i = 0; i < frmbuf->fmtinfo->num_buffers; i++) {
                u32 sizeimage = frmbuf->format.plane_fmt[i].sizeimage;
                
                vb2_set_plane_payload(&buf->buf.vb2_buf, i, sizeimage);
        }
        
        vb2_buffer_done(&buf->buf.vb2_buf, VB2_BUF_STATE_DONE);
}

static int xvip_frmbuf_submit_buffer(struct xvip_fb_buffer *buf,
                                  enum fb_transfer_direction dir,
                                  fb_addr_t frmbuf_addrs[2],
                                  u32 format, unsigned int num_planes,
                                  unsigned int width, unsigned int height,
                                  unsigned int bpl, u32 fid)
{
        struct xvip_frmbuf *frmbuf = buf->frmbuf;
        struct xilinx_frmbuf_tx_descriptor *desc;
        u32 flags = 0;

        if (dir == FB_DEV_TO_MEM) {
                frmbuf->xt.dir = FB_DEV_TO_MEM;
                frmbuf->xt.src_sgl = false;
                frmbuf->xt.dst_sgl = true;
                frmbuf->xt.dst_start = frmbuf_addrs[0];
        } else  {
                frmbuf->xt.dir = FB_MEM_TO_DEV;
                frmbuf->xt.src_sgl = true;
                frmbuf->xt.dst_sgl = false;
                frmbuf->xt.src_start = frmbuf_addrs[0];
        }

        /*
         * FRMBUF IP supports only 2 planes, so one datachunk is sufficient
         * to get start address of 2nd plane
         */

        xilinx_frmbuf_v4l2_config(&frmbuf->chan, format);
        frmbuf->xt.frame_size = num_planes;

        frmbuf->sgl[0].size = width;
        frmbuf->sgl[0].icg = bpl - width;

        /*
         * dst_icg is the number of bytes to jump after last luma addr
         * and before first chroma addr
         */
        if (num_planes == 2)
                frmbuf->sgl[0].dst_icg = frmbuf_addrs[1] - frmbuf_addrs[0]
                                    - bpl * height;

        frmbuf->xt.numf = height;

        desc = xilinx_frmbuf_prep_interleaved(&frmbuf->chan, &frmbuf->xt, flags);
        if (!desc) {
                dev_err(frmbuf->xdev->dev, "Failed to prepare FRMBUF transfer\n");
                return -EINVAL;
        }
        desc->callback = xvip_frmbuf_complete;
        desc->callback_param = buf;
        buf->desc = desc;

        xilinx_frmbuf_set_fid(&frmbuf->chan, desc, fid);

        spin_lock_irq(&frmbuf->queued_lock);
        list_add_tail(&buf->queue, &frmbuf->queued_bufs);
        spin_unlock_irq(&frmbuf->queued_lock);

        xilinx_frmbuf_tx_submit(&frmbuf->chan,desc);

        return 0;
}

static void xvip_frmbuf_submit_vb2_buffer(struct xvip_frmbuf *frmbuf,
                                       struct xvip_fb_buffer *buf)
{
        struct vb2_buffer *vb = &buf->buf.vb2_buf;
        enum fb_transfer_direction dir;
        fb_addr_t frmbuf_addrs[2] = { };
        unsigned int width;
        unsigned int bpl;
        u32 fid;
        int ret;

        switch (frmbuf->queue.type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
        case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
        default:
                dir = FB_DEV_TO_MEM;
                break;

        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                dir = FB_MEM_TO_DEV;
                break;
        }

        bpl = frmbuf->format.plane_fmt[0].bytesperline;

        frmbuf_addrs[0] = vb2_dma_contig_plane_dma_addr(vb, 0);
        if (frmbuf->fmtinfo->num_buffers == 2)
                frmbuf_addrs[1] = vb2_dma_contig_plane_dma_addr(vb, 1);
        else if (frmbuf->fmtinfo->num_planes == 2)
                frmbuf_addrs[1] = frmbuf_addrs[0] + bpl * frmbuf->format.height;

        switch (buf->buf.field) {
        case V4L2_FIELD_TOP:
                fid = 1;
                break;
        case V4L2_FIELD_BOTTOM:
        case V4L2_FIELD_NONE:
                fid = 0;
                break;
        default:
                fid = ~0;
                break;
        }

        width = (size_t)frmbuf->r.width * frmbuf->fmtinfo->bytes_per_pixel[0].numerator
              / (size_t)frmbuf->fmtinfo->bytes_per_pixel[0].denominator;

        ret = xvip_frmbuf_submit_buffer(buf, dir, frmbuf_addrs, frmbuf->format.pixelformat,
                                     frmbuf->fmtinfo->num_planes, width, frmbuf->r.height,
                                     bpl, fid);
        if (ret < 0) {
                vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
                return;
        }
}

/* -----------------------------------------------------------------------------
* Pipeline Stream Management
*/

/*
* Pipelines carry one or more streams, with the sources and sinks being either
* live (such as camera sensors or HDMI connectors) or FRMBUF engines. FRMBUF engines
* at the outputs of the pipeline don't accept packets on their AXI stream slave
* interface until they are started, which may prevent the pipeline from running
* due to back-pressure building up along the pipeline all the way to the
* source if no IP core along the pipeline is able to drop packets. This affects
* pipelines that have multiple output FRMBUF engines.
*
* The runtime behaviour is controlled through the xvip_frmbuf_multi_out_mode
* parameter:
*
* - When set to XVIP_FB_MULTI_OUT_MODE_SYNC, the pipeline start is delayed
*   until all FRMBUF engines have been started. This mode of operation is the
*   default, and needed when the pipeline contains elements that can't drop
*   packets.
*
* - When set to XVIP_FB_MULTI_OUT_MODE_ASYNC, individual branches of the
*   pipeline are started and stopped as output FRMBUF engines are started and
*   stopped. This allows capturing multiple streams independently, but only
*   works if streams that are stopped can be blocked before they reach the FRMBUF
*   engine.
*/

/*
 * Start the FRMBUF engine when the pipeline starts. This function is called for
 * all FRMBUF engines when the pipeline starts.
 */
static int xvip_frmbuf_start(struct xvip_frmbuf *frmbuf)
{
        xilinx_frmbuf_issue_pending(&frmbuf->chan);

        return 0;
}

/*
 * Stop the FRMBUF engine when the pipeline stops. This function is called for all
 * FRMBUF engines when the pipeline stops.
 */
static void xvip_frmbuf_stop(struct xvip_frmbuf *frmbuf)
{
        xilinx_frmbuf_terminate_all(&frmbuf->chan);
}

/**
 * xvip_fb_pipeline_enable_branch - Enable streaming on all subdevs in a pipeline
 *      branch
 * @pipe: The pipeline
 * @frmbuf: The FRMBUF engine at the end of the branch
 *
 * Return: 0 for success, otherwise error code
 */
static int xvip_fb_pipeline_enable_branch(struct xvip_fb_pipeline *pipe,
                                       struct xvip_frmbuf *frmbuf)
{
        struct v4l2_subdev *sd;
        u32 pad;
        int ret;

        dev_dbg(frmbuf->xdev->dev, "Enabling streams on %s\n",
                frmbuf->video.entity.name);

        sd = xvip_frmbuf_remote_subdev(&frmbuf->pad, &pad);
        if (!sd)
                return -ENXIO;

        ret = v4l2_subdev_enable_streams(sd, pad, BIT(0));
        if (ret) {
                dev_err(frmbuf->xdev->dev, "Failed to enable streams for %s\n",
                        frmbuf->video.entity.name);
                return ret;
        }

        return 0;
}

/**
 * xvip_fb_pipeline_disable_branch - Disable streaming on all subdevs in a pipeline
 *      branch
 * @pipe: The pipeline
 * @frmbuf: The FRMBUF engine at the end of the branch
 *
 * Return: 0 for success, otherwise error code
 */
static int xvip_fb_pipeline_disable_branch(struct xvip_fb_pipeline *pipe,
                                        struct xvip_frmbuf *frmbuf)
{
        struct v4l2_subdev *sd;
        u32 pad;
        int ret;

        dev_dbg(frmbuf->xdev->dev, "Disabling streams on %s\n",
                frmbuf->video.entity.name);

        sd = xvip_frmbuf_remote_subdev(&frmbuf->pad, &pad);
        if (!sd)
                return -ENXIO;

        ret = v4l2_subdev_disable_streams(sd, pad, BIT(0));
        if (ret) {
                dev_err(frmbuf->xdev->dev, "Failed to disable streams for %s\n",
                        frmbuf->video.entity.name);
                return ret;
        }

        return 0;
}

#define xvip_pipeline_for_each_frmbuf(pipe, frmbuf, type)                     \
list_for_each_entry(frmbuf, &pipe->frmbufs, pipe_list)                        \
        if (frmbuf->video.vfl_dir == type)

#define xvip_pipeline_for_each_frmbuf_continue_reverse(pipe, frmbuf, type)    \
list_for_each_entry_continue_reverse(frmbuf, &pipe->frmbufs, pipe_list)       \
        if (frmbuf->video.vfl_dir == type)

/**
 * xvip_fb_pipeline_start - Start the full pipeline
 * @pipe: The pipeline
 *
 * This function is used in synchronous pipeline mode to start the full
 * pipeline when all FRMBUF engines have been started.
 *
 * Return: 0 for success, otherwise error code
 */
static int xvip_fb_pipeline_start(struct xvip_fb_pipeline *pipe)
{
        struct xvip_frmbuf *frmbuf;
        int ret;

        /*
         * First start all the output FRMBUF engines, before starting the
         * pipeline. This is required to avoid the slave AXI stream interface
         * applying back pressure and stopping the pipeline right when it gets
         * started.
         */
        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_RX) {
                ret = xvip_frmbuf_start(frmbuf);
                if (ret)
                        goto err_output;
        }

        /* Start all pipeline branches starting from the output FRMBUF engines. */
        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_RX) {
                ret = xvip_fb_pipeline_enable_branch(pipe, frmbuf);
                if (ret)
                        goto err_branch;
        }

        /* Finally start all input FRMBUF engines. */
        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_TX) {
                ret = xvip_frmbuf_start(frmbuf);
                if (ret)
                        goto err_input;
        }

        return 0;

err_input:
        xvip_pipeline_for_each_frmbuf_continue_reverse(pipe, frmbuf, VFL_DIR_TX)
                xvip_frmbuf_stop(frmbuf);

err_branch:
        xvip_pipeline_for_each_frmbuf_continue_reverse(pipe, frmbuf, VFL_DIR_RX)
                xvip_fb_pipeline_disable_branch(pipe, frmbuf);

err_output:
        xvip_pipeline_for_each_frmbuf_continue_reverse(pipe, frmbuf, VFL_DIR_RX)
                xvip_frmbuf_stop(frmbuf);

        return ret;
}

/**
 * xvip_fb_pipeline_stop - Stop the full pipeline
 * @pipe: The pipeline
 *
 * This function is used in synchronous pipeline mode to stop the full
 * pipeline when a FRMBUF engine is stopped.
 */
static void xvip_fb_pipeline_stop(struct xvip_fb_pipeline *pipe)
{
        struct xvip_frmbuf *frmbuf;

        /* There's no meaningful way to handle errors when disabling. */

        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_TX)
                xvip_frmbuf_stop(frmbuf);

        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_RX)
                xvip_fb_pipeline_disable_branch(pipe, frmbuf);

        xvip_pipeline_for_each_frmbuf(pipe, frmbuf, VFL_DIR_RX)
                xvip_frmbuf_stop(frmbuf);
}

/**
 * xvip_pipeline_start_frmbuf - Start a FRMBUF engine on a pipeline
 * @pipe: The pipeline
 * @frmbuf: The FRMBUF engine being started
 *
 * The pipeline is shared between all FRMBUF engines connect at its input and
 * output. While the stream state of FRMBUF engines can be controlled
 * independently, pipelines have a shared stream state that enable or disable
 * all entities in the pipeline. For this reason the pipeline uses a streaming
 * counter that tracks the number of FRMBUF engines that have requested the stream
 * to be enabled.
 *
 * This function increments the pipeline streaming count corresponding to the
 * @frmbuf direction. When the streaming count reaches the number of FRMBUF engines
 * in the pipeline, it enables all entities that belong to the pipeline.
 *
 * Return: 0 if successful, or the return value of the failed video::s_stream
 * operation otherwise. The pipeline state is not updated when the operation
 * fails.
 */
static int xvip_pipeline_start_frmbuf(struct xvip_fb_pipeline *pipe,
                                   struct xvip_frmbuf *frmbuf)
{
        int ret = 0;

        mutex_lock(&pipe->lock);

        switch (xvip_frmbuf_multi_out_mode) {
        case XVIP_FB_MULTI_OUT_MODE_SYNC:
        default:
                if (pipe->input_stream_count + pipe->output_stream_count ==
                    pipe->num_inputs + pipe->num_outputs - 1) {
                        ret = xvip_fb_pipeline_start(pipe);
                        if (ret < 0)
                                goto done;
                }

                if (frmbuf->video.vfl_dir == VFL_DIR_RX)
                        pipe->output_stream_count++;
                else
                        pipe->input_stream_count++;

                break;

        case XVIP_FB_MULTI_OUT_MODE_ASYNC:
                ret = xvip_frmbuf_start(frmbuf);
                if (ret)
                        goto done;

                ret = xvip_fb_pipeline_enable_branch(pipe, frmbuf);
                if (ret) {
                        xvip_frmbuf_stop(frmbuf);
                        goto done;
                }

                break;
        }

 done:
        mutex_unlock(&pipe->lock);
        return ret;
}

/**
 * xvip_pipeline_stop_frmbuf - Stop a FRMBUF engine on a pipeline
 * @pipe: The pipeline
 * @frmbuf: The FRMBUF engine being stopped
 *
 * The pipeline is shared between all FRMBUF engines connect at its input and
 * output. While the stream state of FRMBUF engines can be controlled
 * independently, pipelines have a shared stream state that enable or disable
 * all entities in the pipeline. For this reason the pipeline uses a streaming
 * counter that tracks the number of FRMBUF engines that have requested the stream
 * to be enabled.
 *
 * This function decrements the pipeline streaming count corresponding to the
 * @frmbuf direction. As soon as the streaming count goes lower than the number of
 * FRMBUF engines in the pipeline, it disables all entities in the pipeline.
 */
static void xvip_pipeline_stop_frmbuf(struct xvip_fb_pipeline *pipe,
                                   struct xvip_frmbuf *frmbuf)
{
        mutex_lock(&pipe->lock);

        switch (xvip_frmbuf_multi_out_mode) {
        case XVIP_FB_MULTI_OUT_MODE_SYNC:
        default:
                if (frmbuf->video.vfl_dir == VFL_DIR_RX)
                        pipe->output_stream_count--;
                else
                        pipe->input_stream_count--;

                if (pipe->input_stream_count + pipe->output_stream_count ==
                    pipe->num_inputs + pipe->num_outputs - 1)
                        xvip_fb_pipeline_stop(pipe);

                break;

        case XVIP_FB_MULTI_OUT_MODE_ASYNC:
                xvip_fb_pipeline_disable_branch(pipe, frmbuf);
                xvip_frmbuf_stop(frmbuf);
                break;
        }

        mutex_unlock(&pipe->lock);
}

static int xvip_fb_pipeline_init(struct xvip_fb_pipeline *pipe,
                              struct xvip_frmbuf *start)
{
        struct media_graph graph;
        struct media_entity *entity = &start->video.entity;
        struct media_device *mdev = entity->graph_obj.mdev;
        unsigned int num_inputs = 0;
        unsigned int num_outputs = 0;
        int ret;

        mutex_lock(&mdev->graph_mutex);

        /* Walk the graph to locate the video nodes. */
        ret = media_graph_walk_init(&graph, mdev);
        if (ret) {
                mutex_unlock(&mdev->graph_mutex);
                return ret;
        }

        media_graph_walk_start(&graph, entity);

        while ((entity = media_graph_walk_next(&graph))) {
                struct xvip_frmbuf *frmbuf;

                if (entity->function != MEDIA_ENT_F_IO_V4L)
                        continue;

                frmbuf = to_xvip_frmbuf(media_entity_to_video_device(entity));

                if (frmbuf->pad.flags & MEDIA_PAD_FL_SINK)
                        num_outputs++;
                else
                        num_inputs++;

                list_add_tail(&frmbuf->pipe_list, &pipe->frmbufs);
        }

        mutex_unlock(&mdev->graph_mutex);

        media_graph_walk_cleanup(&graph);

        /* We need at least one DMA to proceed */
        if (num_outputs == 0 && num_inputs == 0)
                return -EPIPE;

        pipe->num_inputs = num_inputs;
        pipe->num_outputs = num_outputs;
        pipe->xdev = start->xdev;

        return 0;
}


static void __xvip_fb_pipeline_cleanup(struct xvip_fb_pipeline *pipe)
{
        while (!list_empty(&pipe->frmbufs))
                 list_del(pipe->frmbufs.next);

         pipe->num_inputs = 0;
         pipe->num_outputs = 0;
}

/**
 * xvip_fb_pipeline_cleanup - Cleanup the pipeline after streaming                                       
 * @pipe: the pipeline
 *
 * Decrease the pipeline use count and clean it up if we were the last user.
 */
static void xvip_fb_pipeline_cleanup(struct xvip_fb_pipeline *pipe)
{
        mutex_lock(&pipe->lock);

        /* If we're the last user clean up the pipeline. */
        if (--pipe->use_count == 0)
                __xvip_fb_pipeline_cleanup(pipe);
        
        mutex_unlock(&pipe->lock);
}

/**
 * xvip_fb_pipeline_prepare - Prepare the pipeline for streaming
 * @pipe: the pipeline
 * @frmbuf: FRMBUF engine at one end of the pipeline
 *
 * Validate the pipeline if no user exists yet, otherwise just increase the use
 * count.
 *
 * Return: 0 if successful or -EPIPE if the pipeline is not valid.
 */
static int xvip_fb_pipeline_prepare(struct xvip_fb_pipeline *pipe,
                                 struct xvip_frmbuf *frmbuf)
{
        int ret;

        mutex_lock(&pipe->lock);

        /* If we're the first user validate and initialize the pipeline. */
        if (pipe->use_count == 0) {
                ret = xvip_fb_pipeline_init(pipe, frmbuf);
                if (ret < 0) {
                        __xvip_fb_pipeline_cleanup(pipe);
                        goto done;
                }
        }

        pipe->use_count++;
        ret = 0;

done:
        mutex_unlock(&pipe->lock);
        return ret;
}


/* -----------------------------------------------------------------------------
 * videobuf2 queue operations
 */

static void xvip_frmbuf_return_buffers(struct xvip_frmbuf *frmbuf,
                                    enum vb2_buffer_state state)
{
        struct xvip_fb_buffer *buf, *nbuf;

        spin_lock_irq(&frmbuf->queued_lock);
        list_for_each_entry_safe(buf, nbuf, &frmbuf->queued_bufs, queue) {
                vb2_buffer_done(&buf->buf.vb2_buf, state);
                list_del(&buf->queue);
        }
        spin_unlock_irq(&frmbuf->queued_lock);
}

static int xvip_frmbuf_queue_setup(struct vb2_queue *vq,
                     unsigned int *nbuffers, unsigned int *nplanes,
                     unsigned int sizes[], struct device *alloc_devs[])
{
         struct xvip_frmbuf *frmbuf = vb2_get_drv_priv(vq);
         unsigned int i;
         int sizeimage;

         /* Make sure the image size is large enough. */
         if (*nplanes) {
                 if (*nplanes != frmbuf->format.num_planes)
                         return -EINVAL;

                 for (i = 0; i < *nplanes; i++) {
                         sizeimage = frmbuf->format.plane_fmt[i].sizeimage;
                         if (sizes[i] < sizeimage)
                                 return -EINVAL;
                 }
         } else {
                 *nplanes = frmbuf->fmtinfo->num_buffers;
                 for (i = 0; i < frmbuf->fmtinfo->num_buffers; i++) {
                         sizeimage = frmbuf->format.plane_fmt[i].sizeimage;
                         sizes[i] = sizeimage;
                 }
         }

         return 0;

}

static int xvip_frmbuf_buffer_prepare(struct vb2_buffer *vb)
{
        struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
        struct xvip_frmbuf *frmbuf = vb2_get_drv_priv(vb->vb2_queue);
        struct xvip_fb_buffer *buf = to_xvip_fb_buffer(vbuf);

        buf->frmbuf = frmbuf;

        return 0;
}

static void xvip_frmbuf_buffer_queue(struct vb2_buffer *vb)
{
         struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
         struct xvip_fb_buffer *buf = to_xvip_fb_buffer(vbuf);
         struct xvip_frmbuf *frmbuf = buf->frmbuf;

         xvip_frmbuf_submit_vb2_buffer(frmbuf, buf);

         if (vb2_is_streaming(&frmbuf->queue))
                 xilinx_frmbuf_issue_pending(&frmbuf->chan);

}

static int xvip_frmbuf_start_streaming(struct vb2_queue *vq, unsigned int count)
{

         struct xvip_frmbuf *frmbuf = vb2_get_drv_priv(vq);
         struct xvip_fb_pipeline *pipe;
         int ret;

         frmbuf->sequence = 0;
         frmbuf->prev_fid = ~0;

         /*
          * Start streaming on the pipeline. No link touching an entity in the
          * pipeline can be activated or deactivated once streaming is started.
          *
          * Use the pipeline object embedded in the first FMRBUF object that starts
          * streaming.
          */
         mutex_lock(&frmbuf->xdev->lock);
         pipe = to_xvip_fb_pipeline(&frmbuf->video) ? : &frmbuf->pipe;

         ret = video_device_pipeline_start(&frmbuf->video, &pipe->pipe);
         mutex_unlock(&frmbuf->xdev->lock);
         if (ret < 0)
                 goto err_return_buffers;

         /* Verify that the configured format matches the output of the
          * connected subdev.
          */
         ret = xvip_frmbuf_verify_format(frmbuf);
         if (ret < 0)
                 goto err_pipe_stop;

         ret = xvip_fb_pipeline_prepare(pipe, frmbuf);
         if (ret < 0)
                 goto err_pipe_stop;

         /* Start the FRMBUF engine on the pipeline. */
         ret = xvip_pipeline_start_frmbuf(pipe, frmbuf);
         if (ret < 0)
                 goto err_pipe_cleanup;

         return 0;

 err_pipe_cleanup:
         xvip_fb_pipeline_cleanup(pipe);
 err_pipe_stop:
         video_device_pipeline_stop(&frmbuf->video);
 err_return_buffers:
         /* Give back all queued buffers to videobuf2. */
         xvip_frmbuf_return_buffers(frmbuf, VB2_BUF_STATE_QUEUED);

         return ret;

}

static void xvip_frmbuf_stop_streaming(struct vb2_queue *vq)
{
         struct xvip_frmbuf *frmbuf = vb2_get_drv_priv(vq);
         struct xvip_fb_pipeline *pipe = to_xvip_fb_pipeline(&frmbuf->video);

         /* Stop the FRMBUF engine on the pipeline. */
         xvip_pipeline_stop_frmbuf(pipe, frmbuf);

         /* Cleanup the pipeline and mark it as being stopped. */
         xvip_fb_pipeline_cleanup(pipe);
         video_device_pipeline_stop(&frmbuf->video);

         /* Give back all queued buffers to videobuf2. */
         xvip_frmbuf_return_buffers(frmbuf, VB2_BUF_STATE_ERROR);

}

static const struct vb2_ops xvip_frmbuf_queue_qops = {
        .queue_setup = xvip_frmbuf_queue_setup,
        .buf_prepare = xvip_frmbuf_buffer_prepare,
        .buf_queue = xvip_frmbuf_buffer_queue,
        .wait_prepare = vb2_ops_wait_prepare,
        .wait_finish = vb2_ops_wait_finish,
        .start_streaming = xvip_frmbuf_start_streaming,
        .stop_streaming = xvip_frmbuf_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int
xvip_frmbuf_querycap(struct file *file, void *fh, struct v4l2_capability *cap)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);

        cap->capabilities = frmbuf->xdev->v4l2_caps | V4L2_CAP_STREAMING |
                            V4L2_CAP_DEVICE_CAPS;

        strscpy((char *)cap->driver, "xilinx-vipp", sizeof(cap->driver));
        strscpy((char *)cap->card, (char *)frmbuf->video.name, sizeof(cap->card));
        snprintf((char *)cap->bus_info, sizeof(cap->bus_info),
                 "platform:%pOFn%u", frmbuf->xdev->dev->of_node,frmbuf->port);

        return 0;
}

static int
xvip_frmbuf_enum_input(struct file *file, void *priv, struct v4l2_input *i)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);
        struct v4l2_subdev *subdev;

        if (i->index > 0)
                return -EINVAL;

        subdev = xvip_frmbuf_remote_subdev(&frmbuf->pad, NULL);
        if (!subdev)
                return -EPIPE;

        /*
         * FIXME: right now only camera input type is handled.
         * There should be mechanism to distinguish other types of
         * input like V4L2_INPUT_TYPE_TUNER and V4L2_INPUT_TYPE_TOUCH.
         */
        i->type = V4L2_INPUT_TYPE_CAMERA;
        strlcpy((char *)i->name, (char *)subdev->name, sizeof(i->name));

        return 0;
}

static int
xvip_frmbuf_get_input(struct file *file, void *fh, unsigned int *i)
{
        *i = 0;
        return 0;
}

static int
xvip_frmbuf_set_input(struct file *file, void *fh, unsigned int i)
{
        if (i > 0)
                return -EINVAL;

        return 0;
}

/* FIXME: without this callback function, some applications are not configured
 * with correct formats, and it results in frames in wrong format. Whether this
 * callback needs to be required is not clearly defined, so it should be
 * clarified through the mailing list.
 */
static int
xvip_frmbuf_enum_format(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);
        const struct xvip_video_format *fmt;
        unsigned int i;
        u32 fmt_cnt = 0;
        u32 *fmts;

        xilinx_frmbuf_get_v4l2_vid_fmts(&frmbuf->chan, &fmt_cnt, &fmts);

        if (f->mbus_code) {
                /* A single 4CC is supported per media bus code. */
                if (f->index > 0)
                        return -EINVAL;

                /*
                 * If the FRMBUF engine returned a list of formats, find the one
                 * that matches the media bus code. Otherwise, search all the
                 * formats supported by this driver.
                 */
                if (fmt_cnt) {
                        for (i = 0; i < fmt_cnt; ++i) {
                                fmt = xvip_get_format_by_fourcc(fmts[i]);
                                if (!IS_ERR(fmt) && fmt->code == f->mbus_code)
                                        break;
                        }

                        if (i == fmt_cnt)
                                return -EINVAL;
                } else {
                        fmt = xvip_get_format_by_code(f->mbus_code);
                }
        } else {
                /*
                 * If the FRMBUF engine returned a list of formats, enumerate them,
                 * otherwise enumerate all the formats supported by this driver.
                 */
                if (fmt_cnt) {
                        if (f->index >= fmt_cnt)
                                return -EINVAL;

                        fmt = xvip_get_format_by_fourcc(fmts[f->index]);
                } else {
                        fmt = xvip_get_format_by_index(f->index);
                }
        }

        if (IS_ERR(fmt))
                return -EINVAL;

        f->pixelformat = fmt->fourcc;

        return 0;

}

static int
xvip_frmbuf_get_format_mplane(struct file *file, void *fh,                                               
                           struct v4l2_format *format)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);

        format->fmt.pix_mp = frmbuf->format;

        return 0;
}

static void
__xvip_frmbuf_try_format(struct xvip_frmbuf *frmbuf,
                      struct v4l2_pix_format_mplane *pix_mp,
                      const struct xvip_video_format **fmtinfo)
{
         const struct xvip_video_format *info;
         unsigned int min_width, max_width;
         unsigned int min_bpl, max_bpl;
         unsigned int width;
         unsigned int i;


         if (pix_mp->field != V4L2_FIELD_ALTERNATE)
                 pix_mp->field = V4L2_FIELD_NONE;

         /*
          * Retrieve format information and select the default format if the
          * requested format isn't supported.
          */
         info = xvip_get_format_by_fourcc(pix_mp->pixelformat);

         if (IS_ERR(info))
                 info = xvip_get_format_by_fourcc(XVIP_FB_DEF_FORMAT);

         /*
          * The width alignment requirements (width_align) are expressed in
          * pixels, while the stride alignment (align) requirements are
          * expressed in bytes.
          */
         min_width = roundup(XVIP_FB_MIN_WIDTH, frmbuf->width_align);
         max_width = rounddown(XVIP_FB_MAX_WIDTH, frmbuf->width_align);

         width = rounddown(pix_mp->width, frmbuf->width_align);
         pix_mp->width = clamp(width, min_width, max_width);
         pix_mp->height = clamp(pix_mp->height, XVIP_FB_MIN_HEIGHT,
                                XVIP_FB_MAX_HEIGHT);

         /*
          * Clamp the requested bytes per line value. If the maximum
          * bytes per line value is zero, the module doesn't support
          * user configurable line sizes. Override the requested value
          * with the minimum in that case.
          */
         max_bpl = rounddown(XVIP_FB_MAX_WIDTH, frmbuf->align);

         /* Calculate the bytesperline and sizeimage values for each plane. */
         for (i = 0; i < info->num_planes; i++) {
                 struct v4l2_plane_pix_format *plane = &pix_mp->plane_fmt[i];
                 unsigned int bpl;

                 min_bpl = pix_mp->width * info->bytes_per_pixel[i].numerator
                         / info->bytes_per_pixel[i].denominator;
                 min_bpl = roundup(min_bpl, frmbuf->align);

                 bpl = rounddown(plane->bytesperline, frmbuf->align);
                 plane->bytesperline = clamp(bpl, min_bpl, max_bpl);

                 plane->sizeimage = plane->bytesperline * pix_mp->height
                                  / (i ? info->vsub : 1);
         }

         /*
          * When using single-planar formats with multiple planes, add up all
          * sizeimage values in the first plane.
          */
         if (info->num_buffers == 1) {
                 for (i = 1; i < info->num_planes; ++i) {
                         struct v4l2_plane_pix_format *plane =
                                 &pix_mp->plane_fmt[i];

                         pix_mp->plane_fmt[0].sizeimage += plane->sizeimage;
                 }
         }

         if (fmtinfo)
                 *fmtinfo = info;

}

static int
xvip_frmbuf_try_format_mplane(struct file *file, void *fh,
                           struct v4l2_format *format)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);

        __xvip_frmbuf_try_format(frmbuf, &format->fmt.pix_mp, NULL);
        return 0;
}

static int
xvip_frmbuf_set_format_mplane(struct file *file, void *fh,
                           struct v4l2_format *format)
{
        struct v4l2_fh *vfh = file->private_data;
        struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);
        const struct xvip_video_format *info = NULL;

        __xvip_frmbuf_try_format(frmbuf, &format->fmt.pix_mp, &info);

        if (vb2_is_busy(&frmbuf->queue))
                return -EBUSY;

        frmbuf->format = format->fmt.pix_mp;

        /*
         * Save format resolution in crop rectangle. This will be
         * updated when s_slection is called.
         */
        frmbuf->r.width = format->fmt.pix_mp.width;
        frmbuf->r.height = format->fmt.pix_mp.height;

        frmbuf->fmtinfo = info;

        return 0;
}

/* Emulate the legacy single-planar API using the multi-planar operations. */
static void
xvip_frmbuf_single_to_multi_planar(const struct v4l2_format *fmt,
                                struct v4l2_format *fmt_mp)
{
        const struct v4l2_pix_format *pix = &fmt->fmt.pix;
        struct v4l2_pix_format_mplane *pix_mp = &fmt_mp->fmt.pix_mp;

        memset(fmt_mp, 0, sizeof(*fmt_mp));

        switch (fmt->type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
                fmt_mp->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
                break;
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
                fmt_mp->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
                break;
        }

        pix_mp->width = pix->width;
        pix_mp->height = pix->height;
        pix_mp->pixelformat = pix->pixelformat;
        pix_mp->field = pix->field;
        pix_mp->colorspace = pix->colorspace;
        pix_mp->plane_fmt[0].sizeimage = pix->sizeimage;
        pix_mp->plane_fmt[0].bytesperline = pix->bytesperline;
        pix_mp->num_planes = 1;
        pix_mp->flags = pix->flags;
        pix_mp->ycbcr_enc = pix->ycbcr_enc;
        pix_mp->quantization = pix->quantization;
        pix_mp->xfer_func = pix->xfer_func;
}

static void
xvip_frmbuf_multi_to_single_planar(const struct v4l2_format *fmt_mp,
                                struct v4l2_format *fmt)
{
        const struct v4l2_pix_format_mplane *pix_mp = &fmt_mp->fmt.pix_mp;
        struct v4l2_pix_format *pix = &fmt->fmt.pix;

        memset(fmt, 0, sizeof(*fmt));

        switch (fmt->type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
                fmt->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
                break;
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                fmt->type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
                break;
        }

        pix->width = pix_mp->width;
        pix->height = pix_mp->height;
        pix->pixelformat = pix_mp->pixelformat;
        pix->field = pix_mp->field;
        pix->colorspace = pix_mp->colorspace;
        pix->sizeimage = pix_mp->plane_fmt[0].sizeimage;
        pix->bytesperline = pix_mp->plane_fmt[0].bytesperline;
        pix->flags = pix_mp->flags;
        pix->ycbcr_enc = pix_mp->ycbcr_enc;
        pix->quantization = pix_mp->quantization;
        pix->xfer_func = pix_mp->xfer_func;
}

static int
xvip_frmbuf_get_format(struct file *file, void *fh, struct v4l2_format *format)
{
         struct v4l2_format fmt_mp;
         int ret;

         xvip_frmbuf_single_to_multi_planar(format, &fmt_mp);

         ret = xvip_frmbuf_get_format_mplane(file, fh, &fmt_mp);
         if (ret)
                 return ret;

         xvip_frmbuf_multi_to_single_planar(&fmt_mp, format);

         return 0;

}

static int
xvip_frmbuf_try_format(struct file *file, void *fh, struct v4l2_format *format)
{
        struct v4l2_format fmt_mp;
        int ret;

        xvip_frmbuf_single_to_multi_planar(format, &fmt_mp);

        ret = xvip_frmbuf_try_format_mplane(file, fh, &fmt_mp);
        if (ret)
                return ret;

        xvip_frmbuf_multi_to_single_planar(&fmt_mp, format);

        return 0;

}

static int
xvip_frmbuf_set_format(struct file *file, void *fh, struct v4l2_format *format)
{
         struct v4l2_format fmt_mp;
         int ret;

         xvip_frmbuf_single_to_multi_planar(format, &fmt_mp);

         ret = xvip_frmbuf_set_format_mplane(file, fh, &fmt_mp);
         if (ret)
                 return ret;

         xvip_frmbuf_multi_to_single_planar(&fmt_mp, format);

         return 0;

}

static int
xvip_frmbuf_g_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
         struct v4l2_fh *vfh = file->private_data;
         struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);
         bool crop_frame = false;

         switch (sel->target) {
         case V4L2_SEL_TGT_COMPOSE:
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                         return -EINVAL;

                 crop_frame = true;
                 break;
         case V4L2_SEL_TGT_COMPOSE_BOUNDS:
         case V4L2_SEL_TGT_COMPOSE_DEFAULT:
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                         return -EINVAL;
                 break;
         case V4L2_SEL_TGT_CROP:
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
                         return -EINVAL;

                 crop_frame = true;
                 break;
         case V4L2_SEL_TGT_CROP_BOUNDS:
         case V4L2_SEL_TGT_CROP_DEFAULT:
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
                         return -EINVAL;
                 break;
         default:
                 return -EINVAL;
         }

         sel->r.left = 0;
         sel->r.top = 0;

         if (crop_frame) {
                 sel->r.width = frmbuf->r.width;
                 sel->r.height = frmbuf->r.height;
         } else {
                 sel->r.width = frmbuf->format.width;
                 sel->r.height = frmbuf->format.height;
         }

         return 0;

}

static int
xvip_frmbuf_s_selection(struct file *file, void *fh, struct v4l2_selection *sel)
{
         struct v4l2_fh *vfh = file->private_data;
         struct xvip_frmbuf *frmbuf = to_xvip_frmbuf(vfh->vdev);
         u32 width, height;

         switch (sel->target) {
         case V4L2_SEL_TGT_COMPOSE:
                 /* COMPOSE target is only valid for capture buftype */
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
                         return -EINVAL;
                 break;
         case V4L2_SEL_TGT_CROP:
                 /* CROP target is only valid for output buftype */
                 if (sel->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
                         return -EINVAL;
                 break;
         default:
                 return -EINVAL;
         }

         width = frmbuf->format.width;
         height = frmbuf->format.height;

         if (sel->r.width > width || sel->r.height > height ||
             sel->r.top != 0 || sel->r.left != 0)
                 return -EINVAL;

         sel->r.width = rounddown(max(XVIP_FB_MIN_WIDTH, sel->r.width),
                                  frmbuf->width_align);
         sel->r.height = max(XVIP_FB_MIN_HEIGHT, sel->r.height);
         frmbuf->r.width = sel->r.width;
         frmbuf->r.height = sel->r.height;

         return 0;

}

static const struct v4l2_ioctl_ops xvip_frmbuf_ioctl_ops = {
        .vidioc_querycap                = xvip_frmbuf_querycap,
        .vidioc_enum_fmt_vid_cap        = xvip_frmbuf_enum_format,
        .vidioc_enum_fmt_vid_out        = xvip_frmbuf_enum_format,
        .vidioc_g_fmt_vid_cap           = xvip_frmbuf_get_format,
        .vidioc_g_fmt_vid_cap_mplane    = xvip_frmbuf_get_format_mplane,
        .vidioc_g_fmt_vid_out           = xvip_frmbuf_get_format,
        .vidioc_g_fmt_vid_out_mplane    = xvip_frmbuf_get_format_mplane,
        .vidioc_s_fmt_vid_cap           = xvip_frmbuf_set_format,
        .vidioc_s_fmt_vid_cap_mplane    = xvip_frmbuf_set_format_mplane,
        .vidioc_s_fmt_vid_out           = xvip_frmbuf_set_format,
        .vidioc_s_fmt_vid_out_mplane    = xvip_frmbuf_set_format_mplane,
        .vidioc_try_fmt_vid_cap         = xvip_frmbuf_try_format,
        .vidioc_try_fmt_vid_cap_mplane  = xvip_frmbuf_try_format_mplane,
        .vidioc_try_fmt_vid_out         = xvip_frmbuf_try_format,
        .vidioc_try_fmt_vid_out_mplane  = xvip_frmbuf_try_format_mplane,
        .vidioc_s_selection             = xvip_frmbuf_s_selection,
        .vidioc_g_selection             = xvip_frmbuf_g_selection,
        .vidioc_reqbufs                 = vb2_ioctl_reqbufs,
        .vidioc_querybuf                = vb2_ioctl_querybuf,
        .vidioc_qbuf                    = vb2_ioctl_qbuf,
        .vidioc_dqbuf                   = vb2_ioctl_dqbuf,
        .vidioc_create_bufs             = vb2_ioctl_create_bufs,
        .vidioc_expbuf                  = vb2_ioctl_expbuf,
        .vidioc_streamon                = vb2_ioctl_streamon,
        .vidioc_streamoff               = vb2_ioctl_streamoff,
        .vidioc_enum_input      = &xvip_frmbuf_enum_input,
        .vidioc_g_input         = &xvip_frmbuf_get_input,
        .vidioc_s_input         = &xvip_frmbuf_set_input,
};

/* -----------------------------------------------------------------------------
* V4L2 file operations
*/
static const struct v4l2_file_operations xvip_frmbuf_fops = {
        .owner          = THIS_MODULE,
        .unlocked_ioctl = video_ioctl2,
        .open           = v4l2_fh_open,
        .release        = vb2_fop_release,
        .poll           = vb2_fop_poll,
        .mmap           = vb2_fop_mmap,
};


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
		dev_info(frmbuf->dev, "failed to find pdev by node (%d)\n",ret);
		ret = -EPROBE_DEFER;
                goto error;
        }

	frmbuf->dev = &pdev->dev;

	match = of_match_node(xilinx_frmbuf_new_of_ids, node);
        if (!match)
        {
		dev_info(frmbuf->dev, "failed to match frmbuf compat string (%d)\n",ret);
		ret = -ENODEV;
                goto error;
        }

        frmbuf->cfg = match->data;

        fb_dir = (enum fb_transfer_direction)frmbuf->cfg->direction;

        if (frmbuf->cfg->flags & XILINX_CLK_PROP) {
                frmbuf->ap_clk = devm_clk_get(frmbuf->dev, "ap_clk");
                if (IS_ERR(frmbuf->ap_clk)) {
                        err = PTR_ERR(frmbuf->ap_clk);
                        dev_err(frmbuf->dev, " failed to get ap_clk (%d)\n", err);
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
                                " New frmbuf Probe deferred due to GPIO reset defer\n");
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
                        dev_err(frmbuf->dev, "failed to enable ap_clk (%d)\n",
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

} EXPORT_SYMBOL(xilinx_frmbuf_parse_dt_node);

/**
 * xilinx_frmbuf_init - Initialize framebuffer structure
 * @pdev:
 *
 * Return: '0' on success and failure value on error
 */

int xvip_frmbuf_init(struct xvip_composite_device *xdev, struct xvip_frmbuf *frmbuf, enum v4l2_buf_type type, struct device_node *node, unsigned int port)
{
        int ret;
	struct v4l2_pix_format_mplane *pix_mp;
	char name[16];

        frmbuf->xdev = xdev;
	frmbuf->port = port;

        mutex_init(&frmbuf->lock);
        mutex_init(&frmbuf->pipe.lock);

        INIT_LIST_HEAD(&frmbuf->queued_bufs);
	INIT_LIST_HEAD(&frmbuf->pipe.frmbufs);
        spin_lock_init(&frmbuf->queued_lock);

        /* ... Initialize the frmbuf channel. */
	snprintf(name, sizeof(name), "port%u", port);
        ret = xilinx_frmbuf_parse_dt_node(frmbuf,node);
        if(ret < 0) {
        	dev_dbg(frmbuf->dev, "frmbuf_parse_dt_failed ret=%d \n",ret);
                return ret;
        }

        xilinx_frmbuf_chan_get_width_align(&frmbuf->chan, &frmbuf->width_align);
        if (!frmbuf->width_align) {
                dev_dbg(frmbuf->xdev->dev,
                        "Using width align %d\n", XVIP_FB_DEF_WIDTH_ALIGN);
                frmbuf->width_align = XVIP_FB_DEF_WIDTH_ALIGN;
        }

        /* Initialize the default format. */
        frmbuf->fmtinfo = xvip_get_format_by_fourcc(XVIP_FB_DEF_FORMAT);

        pix_mp = &frmbuf->format;
        pix_mp->pixelformat = frmbuf->fmtinfo->fourcc;
        pix_mp->colorspace = V4L2_COLORSPACE_SRGB;
        pix_mp->field = V4L2_FIELD_NONE;
        pix_mp->width = XVIP_FB_DEF_WIDTH;
        pix_mp->height = XVIP_FB_DEF_HEIGHT;

        __xvip_frmbuf_try_format(frmbuf, &frmbuf->format, NULL);

        /* Initialize the media entity... */
        if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
            type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                frmbuf->pad.flags = MEDIA_PAD_FL_SINK;
        else
                frmbuf->pad.flags = MEDIA_PAD_FL_SOURCE;

        ret = media_entity_pads_init(&frmbuf->video.entity, 1, &frmbuf->pad);
        if (ret < 0)
                goto error;

        /* ... and the video node... */
        frmbuf->video.fops = &xvip_frmbuf_fops;
        frmbuf->video.v4l2_dev = &xdev->v4l2_dev;
        frmbuf->video.queue = &frmbuf->queue;
        snprintf(frmbuf->video.name, sizeof(frmbuf->video.name), "%pOFn %s %u",
                 xdev->dev->of_node,
                 (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
                  type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                                        ? "output" : "input",
                 port);

        frmbuf->video.vfl_type = VFL_TYPE_VIDEO;
        if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE ||
            type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
                frmbuf->video.vfl_dir = VFL_DIR_RX;
        else
                frmbuf->video.vfl_dir = VFL_DIR_TX;

        frmbuf->video.release = video_device_release_empty;
        frmbuf->video.ioctl_ops = &xvip_frmbuf_ioctl_ops;
        frmbuf->video.lock = &frmbuf->lock;
        frmbuf->video.device_caps = V4L2_CAP_STREAMING;
        switch (type) {
        case V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE:
                frmbuf->video.device_caps |= V4L2_CAP_VIDEO_CAPTURE_MPLANE;
                break;
        case V4L2_BUF_TYPE_VIDEO_CAPTURE:
                frmbuf->video.device_caps |= V4L2_CAP_VIDEO_CAPTURE;
                break;
        case V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE:
                frmbuf->video.device_caps |= V4L2_CAP_VIDEO_OUTPUT_MPLANE;
                break;
        case V4L2_BUF_TYPE_VIDEO_OUTPUT:
                frmbuf->video.device_caps |= V4L2_CAP_VIDEO_OUTPUT;
                break;
        default:
                ret = -EINVAL;
                goto error;
        }

        video_set_drvdata(&frmbuf->video, frmbuf);

        /* ... and the buffers queue. */

        /*
         * Don't enable VB2_READ and VB2_WRITE, as using the read() and write()
         * V4L2 APIs would be inefficient. Testing on the command line with a
         * 'cat /dev/video?' thus won't be possible, but given that the driver
         * anyway requires a test tool to setup the pipeline before any video
         * stream can be started, requiring a specific V4L2 test tool as well
         * instead of 'cat' isn't really a drawback.
         */
        frmbuf->queue.type = type;
        frmbuf->queue.io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF;
        frmbuf->queue.lock = &frmbuf->lock;
        frmbuf->queue.drv_priv = frmbuf;
        frmbuf->queue.buf_struct_size = sizeof(struct xvip_fb_buffer);
        frmbuf->queue.ops = &xvip_frmbuf_queue_qops;
        frmbuf->queue.mem_ops = &vb2_dma_contig_memops;
        frmbuf->queue.timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
                                   | V4L2_BUF_FLAG_TSTAMP_SRC_EOF;
        frmbuf->queue.dev = frmbuf->xdev->dev;
        ret = vb2_queue_init(&frmbuf->queue);
        if (ret < 0) {
                dev_err(frmbuf->xdev->dev, "failed to initialize VB2 queue\n");
                goto error;
        }

        ret = video_register_device(&frmbuf->video, VFL_TYPE_VIDEO, -1);
        if (ret < 0) {
                dev_err(frmbuf->xdev->dev, "failed to register video device\n");
                goto error;
        }

        return 0;

error:
        xvip_frmbuf_cleanup(frmbuf);
        return ret;

} EXPORT_SYMBOL(xvip_frmbuf_init);

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

        dev_info(&pdev->dev, "Xilinx AXI NEW-FrameBuffer Engine Driver Pdev created..pdev= %px pdev->name=%s!!\n", pdev,pdev->name);

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
