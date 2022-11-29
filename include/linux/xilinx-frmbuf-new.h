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


/* Pixels per clock property flag */
#define XILINX_PPC_PROP                         BIT(0)
#define XILINX_FLUSH_PROP                       BIT(1)
#define XILINX_FID_PROP                         BIT(2)
#define XILINX_CLK_PROP                         BIT(3)
#define XILINX_THREE_PLANES_PROP                BIT(4)
#define XILINX_FID_ERR_DETECT_PROP              BIT(5)

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
 * struct xilinx_frmbuf - device structure
 * @dev: Device Structure
 */
struct xvip_frmbuf {

        struct device *dev;

};

#endif /*__XILINX_FRMBUF_NEW_H*/ 
