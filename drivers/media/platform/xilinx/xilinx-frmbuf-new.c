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

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/xilinx-frmbuf-new.h>

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
