#ifndef _ANT_DISPCTRL_CRTC_H_
#define _ANT_DISPCTRL_CRTC_H_

struct drm_device;
struct drm_crtc;

struct drm_crtc* axi_dispctrl_crtc_create(struct drm_device *dev);


/* shared by all Xilinx DMA engines
 */
/* Device configuration structure
 *
 * Xilinx CDMA and Xilinx DMA only use interrupt coalescing and delay counter
 * settings.
 *
 * If used to start/stop parking mode for Xilinx VDMA, vsize must be -1
 * If used to set interrupt coalescing and delay counter only for
 * Xilinx VDMA, hsize must be -1 */
struct xilinx_dma_config {
	enum dma_transfer_direction direction; /* Channel direction */
	int vsize;                         /* Vertical size */
	int hsize;                         /* Horizontal size */
	int stride;                        /* Stride */
	int frm_dly;                       /* Frame delay */
	int gen_lock;                      /* Whether in gen-lock mode */
	int master;                        /* Master that it syncs to */
	int frm_cnt_en;                    /* Enable frame count enable */
	int park;                          /* Whether wants to park */
	int park_frm;                      /* Frame to park on */
	int coalesc;                       /* Interrupt coalescing threshold */
	int delay;                         /* Delay counter */
//	int disable_intr;                  /* Whether use interrupts */
	int reset;			   /* Reset Channel */
	int ext_fsync;			   /* External Frame Sync */
};

#endif
