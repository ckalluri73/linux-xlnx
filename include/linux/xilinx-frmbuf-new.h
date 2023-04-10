/*                                                                                                         
 * Xilinx Framebuffer support header file
 *
 * Copyright (C) 2017 Xilinx, Inc. All rights reserved.
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __XILINX_FRMBUF_NEW_H
#define __XILINX_FRMBUF_NEW_H

#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/xilinx-v4l2-controls.h>

#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-fh.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>

/* Register/Descriptor Offsets */
#define XILINX_FRMBUF_CTRL_OFFSET               0x00
#define XILINX_FRMBUF_GIE_OFFSET                0x04
#define XILINX_FRMBUF_IE_OFFSET                 0x08
#define XILINX_FRMBUF_ISR_OFFSET                0x0c
#define XILINX_FRMBUF_WIDTH_OFFSET              0x10
#define XILINX_FRMBUF_HEIGHT_OFFSET             0x18
#define XILINX_FRMBUF_STRIDE_OFFSET             0x20
#define XILINX_FRMBUF_FMT_OFFSET                0x28
#define XILINX_FRMBUF_ADDR_OFFSET               0x30
#define XILINX_FRMBUF_ADDR2_OFFSET              0x3c
#define XILINX_FRMBUF_FID_OFFSET                0x48
#define XILINX_FRMBUF_FID_MODE_OFFSET   0x50
#define XILINX_FRMBUF_ADDR3_OFFSET              0x54
#define XILINX_FRMBUF_FID_ERR_OFFSET    0x58
#define XILINX_FRMBUF_FID_OUT_OFFSET    0x60
#define XILINX_FRMBUF_RD_ADDR3_OFFSET           0x74

/* Control Registers */
#define XILINX_FRMBUF_CTRL_AP_START             BIT(0)
#define XILINX_FRMBUF_CTRL_AP_DONE              BIT(1)
#define XILINX_FRMBUF_CTRL_AP_IDLE              BIT(2)
#define XILINX_FRMBUF_CTRL_AP_READY             BIT(3)
#define XILINX_FRMBUF_CTRL_FLUSH                BIT(5)
#define XILINX_FRMBUF_CTRL_FLUSH_DONE           BIT(6)
#define XILINX_FRMBUF_CTRL_AUTO_RESTART         BIT(7)
#define XILINX_FRMBUF_GIE_EN                    BIT(0)

/* Interrupt Status and Control */
#define XILINX_FRMBUF_IE_AP_DONE                BIT(0)
#define XILINX_FRMBUF_IE_AP_READY               BIT(1)

#define XILINX_FRMBUF_ISR_AP_DONE_IRQ           BIT(0)
#define XILINX_FRMBUF_ISR_AP_READY_IRQ          BIT(1)

#define XILINX_FRMBUF_ISR_ALL_IRQ_MASK  \
             (XILINX_FRMBUF_ISR_AP_DONE_IRQ | \
                XILINX_FRMBUF_ISR_AP_READY_IRQ)

/* Video Format Register Settings */
#define XILINX_FRMBUF_FMT_RGBX8                 10
#define XILINX_FRMBUF_FMT_YUVX8                 11
#define XILINX_FRMBUF_FMT_YUYV8                 12
#define XILINX_FRMBUF_FMT_RGBA8                 13
#define XILINX_FRMBUF_FMT_YUVA8                 14
#define XILINX_FRMBUF_FMT_RGBX10                15
#define XILINX_FRMBUF_FMT_YUVX10                16
#define XILINX_FRMBUF_FMT_Y_UV8                 18
#define XILINX_FRMBUF_FMT_Y_UV8_420             19
#define XILINX_FRMBUF_FMT_RGB8                  20
#define XILINX_FRMBUF_FMT_YUV8                  21
#define XILINX_FRMBUF_FMT_Y_UV10                22
#define XILINX_FRMBUF_FMT_Y_UV10_420            23
#define XILINX_FRMBUF_FMT_Y8                    24
#define XILINX_FRMBUF_FMT_Y10                   25
#define XILINX_FRMBUF_FMT_BGRA8                 26
#define XILINX_FRMBUF_FMT_BGRX8                 27
#define XILINX_FRMBUF_FMT_UYVY8                 28
#define XILINX_FRMBUF_FMT_BGR8                  29
#define XILINX_FRMBUF_FMT_RGBX12                30
#define XILINX_FRMBUF_FMT_RGB16                 35
#define XILINX_FRMBUF_FMT_Y_U_V8                42
#define XILINX_FRMBUF_FMT_Y_U_V10               43

/* FID Register */
#define XILINX_FRMBUF_FID_MASK                  BIT(0)

/* FID ERR Register */
#define XILINX_FRMBUF_FID_ERR_MASK              BIT(0)
#define XILINX_FRMBUF_FID_OUT_MASK              BIT(0)

#define XILINX_FRMBUF_ALIGN_MUL                 8

#define WAIT_FOR_FLUSH_DONE                     25


/* Pixels per clock property flag */
#define XILINX_PPC_PROP                         BIT(0)
#define XILINX_FLUSH_PROP                       BIT(1)
#define XILINX_FID_PROP                         BIT(2)
#define XILINX_CLK_PROP                         BIT(3)
#define XILINX_THREE_PLANES_PROP                BIT(4)
#define XILINX_FID_ERR_DETECT_PROP              BIT(5)

#define XILINX_FRMBUF_MIN_HEIGHT                (64)
#define XILINX_FRMBUF_MIN_WIDTH                 (64)

/* Modes to enable early callback */
/* To avoid first frame delay */
#define EARLY_CALLBACK                  BIT(1)
/* Give callback at start of descriptor processing */
#define EARLY_CALLBACK_START_DESC       BIT(2)


#ifdef CONFIG_ARCH_FB_ADDR_T_64BIT
typedef u64 fb_addr_t;
#else
typedef u32 fb_addr_t;
#endif

#define XVIP_FB_DEF_FORMAT             V4L2_PIX_FMT_YUYV
#define XVIP_FB_DEF_WIDTH              1920
#define XVIP_FB_DEF_HEIGHT             1080
#define XVIP_FB_DEF_WIDTH_ALIGN        2

/* Minimum and maximum widths are expressed in bytes */
#define XVIP_FB_MIN_WIDTH              1U
#define XVIP_FB_MAX_WIDTH              65535U
#define XVIP_FB_MIN_HEIGHT             1U
#define XVIP_FB_MAX_HEIGHT             8191U

/**
* typedef fb_cookie_t - an opaque FB cookie
* if fb_cookie_t is > 0 it's a FB request cookie, < 0 its an error code
*
*/
typedef s32 fb_cookie_t;
#define FB_MIN_COOKIE 1

/**
* enum fb_transfer_direction - Framebuffer read write dir
* FB_MEM_TO_MEM
* FB_MEM_TO_DEV
* FB_DEV_TO_DEV
* FB_DEV_TO_MEM
* FB_TRANS_NONE
*/
enum fb_transfer_direction {
         FB_MEM_TO_MEM,
         FB_MEM_TO_DEV,
         FB_DEV_TO_DEV,
         FB_DEV_TO_MEM,
         FB_TRANS_NONE,
};

/**
 * enum vid_frmwork_type - Linux video framework type
 * @XDMA_V4L2: fourcc is of type V4L2
 */
enum vid_frmwork_type {
        XDMA_V4L2,
};

/**
 * enum operation_mode - FB IP control register field settings to select mode
 * @DEFAULT : Use default mode, No explicit bit field settings required.
 * @AUTO_RESTART : Use auto-restart mode by setting BIT(7) of control register.
 */
enum operation_mode {
        DEFAULT = 0x0,
        AUTO_RESTART = BIT(7),
};

/**
 * enum fid_modes - FB IP fid mode register settings to select mode
 * @FID_MODE_0: carries the fid value shared by application
 * @FID_MODE_1: sets the fid after first frame
 * @FID_MODE_2: sets the fid after second frame
 */
enum fid_modes {
        FID_MODE_0 = 0,
        FID_MODE_1 = 1,
        FID_MODE_2 = 2,
};

typedef void (*fb_tx_callback)(void *fb_tx_param);
struct xvip_composite_device;
struct xvip_video_format;

/**
* struct xilinx_frmbuf_format_desc - lookup table to match fourcc to format
* @dts_name: Device tree name for this entry.
* @id: Format ID
* @bpw: Bits of pixel data + padding in a 32-bit word (luma plane for semi-pl)
* @ppw: Number of pixels represented in a 32-bit word (luma plane for semi-pl)
* @num_planes: Expected number of plane buffers in framebuffer for this format
* @drm_fmt: DRM video framework equivalent fourcc code
* @v4l2_fmt: Video 4 Linux framework equivalent fourcc code
* @fmt_bitmask: Flag identifying this format in device-specific "enabled"
*      bitmap
*/
struct xilinx_frmbuf_format_desc {
        const char *dts_name;
        u32 id;
        u32 bpw;
        u32 ppw;
        u32 num_planes;
        u32 v4l2_fmt;
        u32 fmt_bitmask;
};

struct xilinx_frmbuf_chan;

/**
 * struct xilinx_frmbuf_desc_hw - Hardware Descriptor
 * @luma_plane_addr: Luma or packed plane buffer address
 * @chroma_plane_addr: Chroma plane buffer address
 * @vsize: Vertical Size
 * @hsize: Horizontal Size
 * @stride: Number of bytes between the first
 *          pixels of each horizontal line
 * @fmt_id: Format ID
 */
struct xilinx_frmbuf_desc_hw {
        fb_addr_t luma_plane_addr;
        fb_addr_t chroma_plane_addr[2];
        u32 vsize;
        u32 hsize;
        u32 stride;
	u32 fmt_id;
};

/**
 * struct xilinx_frmbuf_tx_descriptor - Per Transaction structure
 * @hw: Hardware descriptor
 * @node: Node in the channel descriptors list
 * @fid: Field ID of buffer
 * @earlycb: Whether the callback should be called when in staged state
 */
struct xilinx_frmbuf_tx_descriptor {
        struct xilinx_frmbuf_desc_hw hw;
        struct list_head node;
        u32 fid;
        u32 earlycb;
        fb_tx_callback callback;
        void *callback_param;
	fb_cookie_t cookie;
	struct xilinx_frmbuf_chan *chan;
};

/**
 * struct data_chunk - Element of scatter-gather list that makes a frame.
 * @size: Number of bytes to read from source.
 *        size_dst := fn(op, size_src), so doesn't mean much for destination.
 * @icg: Number of bytes to jump after last src/dst address of this
 *       chunk and before first src/dst address for next chunk.
 *       Ignored for dst(assumed 0), if dst_inc is true and dst_sgl is false.
 *       Ignored for src(assumed 0), if src_inc is true and src_sgl is false.
 * @dst_icg: Number of bytes to jump after last dst address of this
 *       chunk and before the first dst address for next chunk.
 *       Ignored if dst_inc is true and dst_sgl is false.
 * @src_icg: Number of bytes to jump after last src address of this
 *       chunk and before the first src address for next chunk.
 *       Ignored if src_inc is true and src_sgl is false.
 */    
struct data_chunk { 
        size_t size;
        size_t icg;
        size_t dst_icg;
        size_t src_icg;
};     

/**
 * struct fb_interleaved_template - Template to convey DMAC the transfer pattern
 *       and attributes.
 * @src_start: Bus address of source for the first chunk.
 * @dst_start: Bus address of destination for the first chunk.
 * @dir: Specifies the type of Source and Destination.
 * @src_inc: If the source address increments after reading from it.
 * @dst_inc: If the destination address increments after writing to it.
 * @src_sgl: If the 'icg' of sgl[] applies to Source (scattered read).
 *              Otherwise, source is read contiguously (icg ignored).
 *              Ignored if src_inc is false.
 * @dst_sgl: If the 'icg' of sgl[] applies to Destination (scattered write).
 *              Otherwise, destination is filled contiguously (icg ignored).
 *              Ignored if dst_inc is false.
 * @numf: Number of frames in this template.
 * @frame_size: Number of chunks in a frame i.e, size of sgl[].
 * @sgl: Array of {chunk,icg} pairs that make up a frame.
 */
struct fb_interleaved_template {                                                                     
        fb_addr_t src_start;
        fb_addr_t dst_start;
        enum fb_transfer_direction dir;
        bool src_inc;
        bool dst_inc;
        bool src_sgl;
        bool dst_sgl;
        size_t numf;
        size_t frame_size;
        struct data_chunk sgl[];
};

/**
 * struct xilinx_frmbuf_chan - Driver specific dma channel structure
 * @xdev: Driver specific device structure
 * @lock: Descriptor operation lock
 * @chan_node: Member of a list of framebuffer channel instances
 * @pending_list: Descriptors waiting
 * @done_list: Complete descriptors
 * @staged_desc: Next buffer to be programmed
 * @active_desc: Currently active buffer being read/written to
 * @dev: The dma device
 * @write_addr: callback that will write dma addresses to IP (32 or 64 bit)
 * @irq: Channel IRQ
 * @direction: Transfer direction
 * @idle: Channel idle state
 * @tasklet: Cleanup work after irq
 * @vid_fmt: Reference to currently assigned video format description
 * @hw_fid: FID enabled in hardware flag
 * @mode: Select operation mode
 * @fid_err_flag: Field id error detection flag
 * @fid_out_val: Field id out val
 * @fid_mode: Select fid mode
 */
struct xilinx_frmbuf_chan {
        struct xvip_frmbuf *frmbuf;
        /* Descriptor operation lock */
        spinlock_t lock;
        struct list_head chan_node;
        struct list_head pending_list;
        struct list_head done_list;
	fb_cookie_t cookie;
	fb_cookie_t completed_cookie;
	struct xilinx_frmbuf_tx_descriptor *staged_desc;
	struct xilinx_frmbuf_tx_descriptor *active_desc;
        struct device *dev;
        enum fb_transfer_direction direction;
        int irq;
        bool idle;
        void (*write_addr)(struct xilinx_frmbuf_chan *chan, u32 reg,
                           fb_addr_t value);
	struct tasklet_struct tasklet;
        const struct xilinx_frmbuf_format_desc *vid_fmt;
        bool hw_fid;
        enum operation_mode mode;
        u8 fid_err_flag;
        u8 fid_out_val;
        enum fid_modes fid_mode;
};

/**
 * struct xilinx_frmbuf - device structure
 * @dev: Device Structure
 */
struct xvip_frmbuf {
       void __iomem *regs;
       struct device *dev;
       struct xvip_composite_device *xdev;

       struct video_device video;

       struct xilinx_frmbuf_chan chan;

       struct gpio_desc *rst_gpio;

       struct v4l2_format format;
       const struct xvip_video_format *fmtinfo;

       const struct xilinx_frmbuf_feature *cfg;

       u32 enabled_vid_fmts;
       u32 *v4l2_memory_fmts;
       u32 v4l2_fmt_cnt;
       u32 *poss_v4l2_fmts;
       u32 poss_v4l2_fmt_cnt;

       u32 max_width;
       u32 max_height;
       unsigned int align;
       u32 ppc;
       struct clk *ap_clk;

       u32 prev_fid;
       u32 low_latency_cap;

};

#if IS_ENABLED(CONFIG_XILINX_FRMBUF_NEW)
/*
TODO: xvip_frmbuf_init
*/
int xilinx_frmbuf_init(struct xvip_composite_device *xdev, struct xvip_frmbuf *frmbuf, struct device_node *node);

#else

static inline void  xilinx_frmbuf_init(struct xvip_composite_device *xdev, struct xvip_frmbuf *frmbuf, struct device_node *node)

{ }

#endif

#endif /*__XILINX_FRMBUF_NEW_H*/ 
