/*
 * drivers/media/video/sun4i_csi/csi1/sun4i_drv_csi.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

/*
 * Sun4i Camera Interface  driver
 * Author: raymonxiu
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/delay.h>

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#include <linux/freezer.h>
#endif

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-dma-contig.h>
#include <linux/moduleparam.h>

#include <plat/sys_config.h>
#include <mach/clock.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>

#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#include "sun4i_csi_reg.h"

#define CSI_MAJOR_VERSION 1
#define CSI_MINOR_VERSION 0
#define CSI_RELEASE 0
#define CSI_VERSION \
	KERNEL_VERSION(CSI_MAJOR_VERSION, CSI_MINOR_VERSION, CSI_RELEASE)
#define CSI_MODULE_NAME "sun4i_csi1"

//#define USE_DMA_CONTIG

//#define AJUST_DRAM_PRIORITY
#define REGS_pBASE					(0x01C00000)	 	      // register base addr
#define SDRAM_REGS_pBASE    (REGS_pBASE + 0x01000)    // SDRAM Controller

#define NUM_INPUTS 2
#define CSI_OUT_RATE      (24*1000*1000)
//#define CSI_ISP_RATE			(100*1000*1000)
#define CSI_MAX_FRAME_MEM (32*1024*1024)
//#define TWI_NO		 (1)

#define MIN_WIDTH  (32)
#define MIN_HEIGHT (32)
#define MAX_WIDTH  (4096)
#define MAX_HEIGHT (4096)

static unsigned video_nr = 1;
static unsigned first_flag = 0;


static bool debug;
module_param(debug, bool, 0644);
MODULE_PARM_DESC(debug, "Debug level (0-1)");

static char ccm[I2C_NAME_SIZE] = "";
static uint i2c_addr = 0xff;

static char ccm_b[I2C_NAME_SIZE] = "";
static uint i2c_addr_b = 0xff;

module_param_string(ccm, ccm, sizeof(ccm), S_IRUGO|S_IWUSR);
module_param(i2c_addr,uint, S_IRUGO|S_IWUSR);
module_param_string(ccm_b, ccm_b, sizeof(ccm_b), S_IRUGO|S_IWUSR);
module_param(i2c_addr_b,uint, S_IRUGO|S_IWUSR);

static struct csi_sensor_platform_data sensor_0_pdata = {
	.iovdd_str = "",
	.avdd_str = "",
	.dvdd_str = "",
	/* TODO: Add mclk parameter to FEX file */
	.mclk = CSI_OUT_RATE,
};

static struct csi_sensor_platform_data sensor_1_pdata = {
	.iovdd_str = "",
	.avdd_str = "",
	.dvdd_str = "",
	/* TODO: Add mclk parameter to FEX file */
	.mclk = CSI_OUT_RATE,
};

static struct i2c_board_info  dev_sensor[] =  {
	{
		//I2C_BOARD_INFO(ccm, i2c_addr),
		.platform_data	= &sensor_0_pdata,
	},
	{
		//I2C_BOARD_INFO(ccm, i2c_addr),
		.platform_data	= &sensor_1_pdata,
	},
};

//ccm support format
static struct csi_fmt formats[] = {
	{
		.name     		= "planar YUV 422",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_YUV422P,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_PLANAR_YUV422,
		.depth    		= 16,
		.planes_cnt		= 3,
	},
	{
		.name     		= "planar YUV 420",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_YUV420,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_PLANAR_YUV420,
		.depth    		= 12,
		.planes_cnt		= 3,
	},
	{
		.name     		= "planar YUV 422 UV combined",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_NV16,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_UV_CB_YUV422,
		.depth    		= 16,
		.planes_cnt		= 2,
	},
	{
		.name     		= "planar YUV 420 UV combined",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_NV12,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_UV_CB_YUV420,
		.depth    		= 12,
		.planes_cnt		= 2,
	},
	{
		.name     		= "planar YUV 422 VU combined",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_NV61,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_UV_CB_YUV422,
		.depth    		= 16,
		.planes_cnt		= 2,
	},
	{
		.name     		= "planar YUV 420 VU combined",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_NV21,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_UV_CB_YUV420,
		.depth    		= 12,
		.planes_cnt		= 2,
	},
	{
		.name     		= "MB YUV420",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_HM12,
		.input_fmt		= CSI_YUV422,
		.output_fmt		= CSI_MB_YUV420,
		.depth    		= 12,
		.planes_cnt		= 2,
	},
	{
		.name     		= "RAW Bayer",
		.ccm_fmt			= V4L2_MBUS_FMT_SBGGR8_1X8,	//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_SBGGR8,
		.input_fmt		= CSI_RAW,
		.output_fmt		= CSI_PASS_THROUTH,
		.depth    		= 8,
		.planes_cnt		= 1,
	},
//	{
//		.name     		= "planar RGB242",
//		.ccm_fmt			= V4L2_PIX_FMT_SBGGR8,
//		.fourcc   		= V4L2_PIX_FMT_RGB32,		//can't find the appropriate format in V4L2 define,use this temporarily
//		.input_fmt		= CSI_BAYER,
//		.output_fmt		= CSI_PLANAR_RGB242,
//		.depth    		= 8,
//		.planes_cnt		= 3,
//	},
	{
		.name     		= "YUV422 YUYV",
		.ccm_fmt			= V4L2_MBUS_FMT_YUYV8_2X8,//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_YUYV,
		.input_fmt		= CSI_RAW,
		.output_fmt		= CSI_PASS_THROUTH,
		.depth    		= 16,
		.planes_cnt		= 1,
	},
	{
		.name     		= "YUV422 YVYU",
		.ccm_fmt			= V4L2_MBUS_FMT_YVYU8_2X8,//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_YVYU,
		.input_fmt		= CSI_RAW,
		.output_fmt		= CSI_PASS_THROUTH,
		.depth    		= 16,
		.planes_cnt		= 1,
	},
	{
		.name     		= "YUV422 UYVY",
		.ccm_fmt			= V4L2_MBUS_FMT_UYVY8_2X8,//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_UYVY,
		.input_fmt		= CSI_RAW,
		.output_fmt		= CSI_PASS_THROUTH,
		.depth    		= 16,
		.planes_cnt		= 1,
	},
	{
		.name     		= "YUV422 VYUY",
		.ccm_fmt			= V4L2_MBUS_FMT_VYUY8_2X8,//linux-3.0
		.fourcc   		= V4L2_PIX_FMT_VYUY,
		.input_fmt		= CSI_RAW,
		.output_fmt		= CSI_PASS_THROUTH,
		.depth    		= 16,
		.planes_cnt		= 1,
	},
};

static struct csi_fmt *get_format(struct v4l2_format *f)
{
	struct csi_fmt *fmt;
	unsigned int k;

	for (k = 0; k < ARRAY_SIZE(formats); k++) {
		fmt = &formats[k];
		if (fmt->fourcc == f->fmt.pix.pixelformat) {
			break;
		}
	}

	if (k == ARRAY_SIZE(formats)) {
		return NULL;
	}

	return &formats[k];
};

static void csi_print_sensor_i2c_board_info(const struct i2c_board_info *binfo)
{
	struct csi_sensor_platform_data *pdata = binfo->platform_data;

	csi_dbg(0, "i2c_board_info:\n");
	csi_dbg(0, "i2c_board_info->addr = %x\n", binfo->addr);
	csi_dbg(0, "i2c_board_info->type = %s\n", binfo->type);
	csi_dbg(0, "platform_data:\n");
	csi_dbg(0, "csi_sensor_platform_data->iovdd_str = %s\n", pdata->iovdd_str);
	csi_dbg(0, "csi_sensor_platform_data->avdd_str = %s\n", pdata->avdd_str);
	csi_dbg(0, "csi_sensor_platform_data->dvdd_str = %s\n", pdata->dvdd_str);
	csi_dbg(0, "csi_sensor_platform_data->flash_pol = %d\n", pdata->flash_pol);
	csi_dbg(0, "csi_sensor_platform_data->interface = %d\n", pdata->interface);
	csi_dbg(0, "csi_sensor_platform_data->inv_vflip = %d\n", pdata->inv_vflip);
	csi_dbg(0, "csi_sensor_platform_data->inv_hflip = %d\n", pdata->inv_hflip);
	csi_dbg(0, "csi_sensor_platform_data->mclk = %d\n", pdata->mclk);
}

void static inline bsp_csi_int_clear_status(struct csi_dev *dev,__csi_int_t interrupt)
{
    W(dev->regs+CSI_REG_INT_STATUS, interrupt);
}

void static inline bsp_csi_set_buffer_address(struct csi_dev *dev,__csi_buf_t buf, u32 addr)
{
	//bufer0a +4 = buffer0b, bufer0a +8 = buffer1a
    W(dev->regs+CSI_REG_BUF_0_A + (buf<<2), addr);
}


static inline void csi_set_addr(struct csi_dev *dev,struct csi_buffer *buffer)
{

	struct csi_buffer *buf = buffer;
	dma_addr_t addr_org;

	csi_dbg(3,"buf ptr=%p\n",buf);

	addr_org = videobuf_to_dma_contig((struct videobuf_buffer *)buf);


	if(dev->fmt->input_fmt==CSI_RAW){
		dev->csi_buf_addr.y  = addr_org;
		dev->csi_buf_addr.cb = addr_org;
		dev->csi_buf_addr.cr = addr_org;

	}else if(dev->fmt->input_fmt==CSI_BAYER){
		//really rare here
		dev->csi_buf_addr.cb = addr_org;//for G channel
		dev->csi_buf_addr.y  = addr_org + dev->width*dev->height*1/2;//for B channel
		dev->csi_buf_addr.cr = addr_org + dev->width*dev->height*3/4;//for R channel

	}else if(dev->fmt->input_fmt==CSI_CCIR656){
	//TODO:

	}else if(dev->fmt->input_fmt==CSI_YUV422){

		switch (dev->fmt->output_fmt) {
			case CSI_PLANAR_YUV422:
				dev->csi_buf_addr.y  = addr_org;
				dev->csi_buf_addr.cb = addr_org + dev->width*dev->height;
				dev->csi_buf_addr.cr = addr_org + dev->width*dev->height*3/2;
				break;

			case CSI_PLANAR_YUV420:
				dev->csi_buf_addr.y  = addr_org;
				dev->csi_buf_addr.cb = addr_org + dev->width*dev->height;
				dev->csi_buf_addr.cr = addr_org + dev->width*dev->height*5/4;
				break;

			case CSI_UV_CB_YUV422:
			case CSI_UV_CB_YUV420:
			case CSI_MB_YUV422:
			case CSI_MB_YUV420:
				dev->csi_buf_addr.y  = addr_org;
				dev->csi_buf_addr.cb = addr_org + dev->width*dev->height;
				dev->csi_buf_addr.cr = addr_org + dev->width*dev->height;
				break;

			default:
				break;
		}
	}

	bsp_csi_set_buffer_address(dev, CSI_BUF_0_A, dev->csi_buf_addr.y);
	bsp_csi_set_buffer_address(dev, CSI_BUF_0_B, dev->csi_buf_addr.y);
	bsp_csi_set_buffer_address(dev, CSI_BUF_1_A, dev->csi_buf_addr.cb);
	bsp_csi_set_buffer_address(dev, CSI_BUF_1_B, dev->csi_buf_addr.cb);
	bsp_csi_set_buffer_address(dev, CSI_BUF_2_A, dev->csi_buf_addr.cr);
	bsp_csi_set_buffer_address(dev, CSI_BUF_2_B, dev->csi_buf_addr.cr);

	csi_dbg(3,"csi_buf_addr_y=%x\n",  dev->csi_buf_addr.y);
	csi_dbg(3,"csi_buf_addr_cb=%x\n", dev->csi_buf_addr.cb);
	csi_dbg(3,"csi_buf_addr_cr=%x\n", dev->csi_buf_addr.cr);

}

static int csi_clk_get(struct csi_dev *dev)
{
	int ret;

	dev->csi_ahb_clk=clk_get(NULL, "ahb_csi1");
	if (dev->csi_ahb_clk == NULL) {
		csi_err("get csi1 ahb clk error!\n");
		return -1;
	}

	if(dev->mclk==24000000 || dev->mclk==12000000) {
		dev->csi_clk_src=clk_get(NULL,"hosc");
		if (dev->csi_clk_src == NULL) {
			csi_err("get csi1 hosc source clk error!\n");
			return -1;
		}
	}
	else
	{
		dev->csi_clk_src=clk_get(NULL,"video_pll1");
		if (dev->csi_clk_src == NULL) {
			csi_err("get csi1 video pll1 source clk error!\n");
			return -1;
		}
	}

	dev->csi_module_clk=clk_get(NULL,"csi1");
	if(dev->csi_module_clk == NULL) {
		csi_err("get csi1 module clk error!\n");
		return -1;
	}

	ret = clk_set_parent(dev->csi_module_clk, dev->csi_clk_src);
	if (ret == -1) {
		csi_err(" csi set parent failed \n");
		return -1;
	}

	clk_put(dev->csi_clk_src);

	ret = clk_set_rate(dev->csi_module_clk, dev->mclk);
	if (ret == -1) {
		csi_err("set csi1 module clock error\n");
		return -1;
	}

//	dev->csi_isp_src_clk=clk_get(NULL,"video_pll0");
//	if (dev->csi_isp_src_clk == NULL) {
//       	csi_err("get csi_isp source clk error!\n");
//		return -1;
//    }
//
//  dev->csi_isp_clk=clk_get(NULL,"csi_isp");
//	if(dev->csi_isp_clk == NULL) {
//       	csi_err("get csi_isp clk error!\n");
//		return -1;
//    }
//
//	ret = clk_set_parent(dev->csi_isp_clk, dev->csi_isp_src_clk);
//	if (ret == -1) {
//        csi_err(" csi_isp set parent failed \n");
//	    return -1;
//    }
//
//	clk_put(dev->csi_isp_src_clk);
//
//  ret = clk_set_rate(dev->csi_isp_clk, CSI_ISP_RATE);
//	if (ret == -1) {
//        csi_err("set csi_isp clock error\n");
//		return -1;
//   	}

	dev->csi_dram_clk = clk_get(NULL, "sdram_csi1");
	if (dev->csi_dram_clk == NULL) {
		csi_err("get csi1 dram clk error!\n");
		return -1;
	}

	return 0;
}

static int csi_clk_out_set(struct csi_dev *dev)
{
	int ret;
	ret = clk_set_rate(dev->csi_module_clk, dev->mclk);
	if (ret == -1) {
		csi_err("set csi1 module clock error\n");
		return -1;
	}

	return 0;
}

static void csi_reset_enable(struct csi_dev *dev)
{
	clk_reset(dev->csi_module_clk, 1);
}

static void csi_reset_disable(struct csi_dev *dev)
{
	clk_reset(dev->csi_module_clk, 0);
}

static int csi_clk_enable(struct csi_dev *dev)
{
	clk_enable(dev->csi_ahb_clk);
//	clk_enable(dev->csi_module_clk);
	clk_enable(dev->csi_isp_clk);
	clk_enable(dev->csi_dram_clk);

	return 0;
}

static int csi_clk_disable(struct csi_dev *dev)
{
	clk_disable(dev->csi_ahb_clk);
//	clk_disable(dev->csi_module_clk);
	clk_disable(dev->csi_isp_clk);
	clk_disable(dev->csi_dram_clk);

	return 0;
}

static int csi_clk_release(struct csi_dev *dev)
{
	clk_put(dev->csi_ahb_clk);
    dev->csi_ahb_clk = NULL;

	clk_put(dev->csi_module_clk);
    dev->csi_module_clk = NULL;

	clk_put(dev->csi_dram_clk);
    dev->csi_dram_clk = NULL;

	return 0;
}

static int inline csi_is_generating(struct csi_dev *dev)
{
	return test_bit(0, &dev->generating);
}

static void inline csi_start_generating(struct csi_dev *dev)
{
	 set_bit(0, &dev->generating);
	 return;
}

static void inline csi_stop_generating(struct csi_dev *dev)
{
	 first_flag = 0;
	 clear_bit(0, &dev->generating);
	 return;
}

static void update_ccm_info(struct csi_dev *dev , struct ccm_config *ccm_cfg)
{
   dev->sd = ccm_cfg->sd;
	dev->mclk = ccm_cfg->mclk;
}

static irqreturn_t csi_isr(int irq, void *priv)
{
	struct csi_buffer *buf;
	struct csi_dev *dev = (struct csi_dev *)priv;
	struct csi_dmaqueue *dma_q = &dev->vidq;
//	__csi_int_status_t * status;

	csi_dbg(3,"csi_isr\n");

	bsp_csi_int_disable(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE

	spin_lock(&dev->slock);

	if (first_flag == 0) {
		first_flag=1;
		goto set_next_addr;
	}

	if (list_empty(&dma_q->active)) {
		csi_err("No active queue to serve\n");
		goto unlock;
	}

	buf = list_entry(dma_q->active.next,struct csi_buffer, vb.queue);
	csi_dbg(3,"buf ptr=%p\n",buf);

	/* Nobody is waiting on this buffer*/

	if (!waitqueue_active(&buf->vb.done)) {
		csi_dbg(1," Nobody is waiting on this buffer,buf = 0x%p\n",buf);
	}

	list_del(&buf->vb.queue);

	do_gettimeofday(&buf->vb.ts);
	buf->vb.field_count++;

	dev->ms += jiffies_to_msecs(jiffies - dev->jiffies);
	dev->jiffies = jiffies;

	buf->vb.state = VIDEOBUF_DONE;
	wake_up(&buf->vb.done);

	//judge if the frame queue has been written to the last
	if (list_empty(&dma_q->active)) {
		csi_dbg(1,"No more free frame\n");
		goto unlock;
	}

	if ((&dma_q->active) == dma_q->active.next->next) {
		csi_dbg(1,"No more free frame on next time\n");
		goto unlock;
	}


set_next_addr:
	buf = list_entry(dma_q->active.next->next,struct csi_buffer, vb.queue);
	csi_set_addr(dev,buf);

unlock:
	spin_unlock(&dev->slock);

//	bsp_csi_int_get_status(dev, status);
//	if((status->buf_0_overflow) || (status->buf_1_overflow) || (status->buf_2_overflow))
//	{
//		bsp_csi_int_clear_status(dev,CSI_INT_BUF_0_OVERFLOW);
//		bsp_csi_int_clear_status(dev,CSI_INT_BUF_1_OVERFLOW);
//		bsp_csi_int_clear_status(dev,CSI_INT_BUF_2_OVERFLOW);
//		csi_err("fifo overflow\n");
//	}
//
//	if((status->hblank_overflow))
//	{
//		bsp_csi_int_clear_status(dev,CSI_INT_HBLANK_OVERFLOW);
//		csi_err("hblank overflow\n");
//	}
	bsp_csi_int_clear_status(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	bsp_csi_int_enable(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE

	return IRQ_HANDLED;
}

/*
 * Videobuf operations
 */
static int buffer_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
	struct csi_dev *dev = vq->priv_data;

	csi_dbg(1,"buffer_setup\n");

	if(dev->fmt->input_fmt == CSI_RAW)
	{
		switch(dev->fmt->fourcc) {
			case 	V4L2_PIX_FMT_YUYV:
			case	V4L2_PIX_FMT_YVYU:
			case	V4L2_PIX_FMT_UYVY:
			case	V4L2_PIX_FMT_VYUY:
				*size = dev->width * dev->height * 2;
				break;
			default:
				*size = dev->width * dev->height;
				break;
		}
	}
	else if(dev->fmt->input_fmt == CSI_BAYER)
	{
		*size = dev->width * dev->height;
	}
	else if(dev->fmt->input_fmt == CSI_CCIR656)
	{
		//TODO
	}
	else if(dev->fmt->input_fmt == CSI_YUV422)
	{
		switch (dev->fmt->output_fmt) {
			case 	CSI_PLANAR_YUV422:
			case	CSI_UV_CB_YUV422:
			case 	CSI_MB_YUV422:
				*size = dev->width * dev->height * 2;
				break;

			case CSI_PLANAR_YUV420:
			case CSI_UV_CB_YUV420:
			case CSI_MB_YUV420:
				*size = dev->width * dev->height * 3/2;
				break;

			default:
				*size = dev->width * dev->height * 2;
				break;
		}
	}
	else
	{
		*size = dev->width * dev->height * 2;
	}

	dev->frame_size = *size;

	if (*count < 3) {
		*count = 3;
		csi_err("buffer count is invalid, set to 3\n");
	} else if(*count > 5) {
		*count = 5;
		csi_err("buffer count is invalid, set to 5\n");
	}

	while (*size * *count > CSI_MAX_FRAME_MEM) {
		(*count)--;
	}
	csi_print("%s, buffer count=%d, size=%d\n", __func__,*count, *size);

	return 0;
}

static void free_buffer(struct videobuf_queue *vq, struct csi_buffer *buf)
{
	csi_dbg(1,"%s, state: %i\n", __func__, buf->vb.state);

#ifdef USE_DMA_CONTIG
	videobuf_dma_contig_free(vq, &buf->vb);
#endif

	csi_dbg(1,"free_buffer: freed\n");

	buf->vb.state = VIDEOBUF_NEEDS_INIT;
}

static int buffer_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb,
						  enum v4l2_field field)
{
	struct csi_dev *dev = vq->priv_data;
	struct csi_buffer *buf = container_of(vb, struct csi_buffer, vb);
	int rc;

	csi_dbg(1,"buffer_prepare\n");

	BUG_ON(NULL == dev->fmt);

	if (dev->width  < MIN_WIDTH || dev->width  > MAX_WIDTH ||
	    dev->height < MIN_HEIGHT || dev->height > MAX_HEIGHT) {
		return -EINVAL;
	}

	buf->vb.size = dev->frame_size;

	if (0 != buf->vb.baddr && buf->vb.bsize < buf->vb.size) {
		return -EINVAL;
	}

	/* These properties only change when queue is idle, see s_fmt */
	buf->fmt       = dev->fmt;
	buf->vb.width  = dev->width;
	buf->vb.height = dev->height;
	buf->vb.field  = field;

	if (VIDEOBUF_NEEDS_INIT == buf->vb.state) {
		rc = videobuf_iolock(vq, &buf->vb, NULL);
		if (rc < 0) {
			goto fail;
		}
	}

	vb->boff= videobuf_to_dma_contig(vb);
	buf->vb.state = VIDEOBUF_PREPARED;

	return 0;

fail:
	free_buffer(vq, buf);
	return rc;
}

static void buffer_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
	struct csi_dev *dev = vq->priv_data;
	struct csi_buffer *buf = container_of(vb, struct csi_buffer, vb);
	struct csi_dmaqueue *vidq = &dev->vidq;

	csi_dbg(1,"buffer_queue\n");
	buf->vb.state = VIDEOBUF_QUEUED;
	list_add_tail(&buf->vb.queue, &vidq->active);
}

static void buffer_release(struct videobuf_queue *vq,
			   struct videobuf_buffer *vb)
{
	struct csi_buffer *buf  = container_of(vb, struct csi_buffer, vb);

	csi_dbg(1,"buffer_release\n");

	free_buffer(vq, buf);
}

static struct videobuf_queue_ops csi_video_qops = {
	.buf_setup    = buffer_setup,
	.buf_prepare  = buffer_prepare,
	.buf_queue    = buffer_queue,
	.buf_release  = buffer_release,
};

/*
 * IOCTL vidioc handling
 */
static int vidioc_querycap(struct file *file, void  *priv,
					struct v4l2_capability *cap)
{
	struct csi_dev *dev = video_drvdata(file);

	strcpy(cap->driver, "sun4i_csi");
	strcpy(cap->card, "sun4i_csi");
	strlcpy(cap->bus_info, dev->v4l2_dev.name, sizeof(cap->bus_info));

	cap->version = CSI_VERSION;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING | \
			    V4L2_CAP_READWRITE;
	return 0;
}

static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,
					struct v4l2_fmtdesc *f)
{
	struct csi_fmt *fmt;

	csi_dbg(0,"vidioc_enum_fmt_vid_cap\n");

	if (f->index > ARRAY_SIZE(formats)-1) {
		return -EINVAL;
	}
	fmt = &formats[f->index];

	strlcpy(f->description, fmt->name, sizeof(f->description));
	f->pixelformat = fmt->fourcc;
	return 0;
}

static int vidioc_g_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct csi_dev *dev = video_drvdata(file);

	f->fmt.pix.width        = dev->width;
	f->fmt.pix.height       = dev->height;
	f->fmt.pix.field        = dev->vb_vidq.field;
	f->fmt.pix.pixelformat  = dev->fmt->fourcc;
	f->fmt.pix.bytesperline = (f->fmt.pix.width * dev->fmt->depth) >> 3;
	f->fmt.pix.sizeimage    = f->fmt.pix.height * f->fmt.pix.bytesperline;

	return 0;
}

static int vidioc_try_fmt_vid_cap(struct file *file, void *priv,
			struct v4l2_format *f)
{
	struct csi_dev *dev = video_drvdata(file);
	struct csi_fmt *csi_fmt;
	struct v4l2_mbus_framefmt ccm_fmt;//linux-3.0
	int ret = 0;

	csi_dbg(0,"vidioc_try_fmt_vid_cap\n");

	/*judge the resolution*/
	if(f->fmt.pix.width > MAX_WIDTH || f->fmt.pix.height > MAX_HEIGHT) {
		csi_err("size is too large,automatically set to maximum!\n");
		f->fmt.pix.width = MAX_WIDTH;
		f->fmt.pix.height = MAX_HEIGHT;
	}

	csi_fmt = get_format(f);
	if (!csi_fmt) {
		csi_err("Fourcc format (0x%08x) invalid.\n",f->fmt.pix.pixelformat);
		return -EINVAL;
	}

	ccm_fmt.code = csi_fmt->ccm_fmt;//linux-3.0
	ccm_fmt.width = f->fmt.pix.width;//linux-3.0
	ccm_fmt.height = f->fmt.pix.height;//linux-3.0

	ret = v4l2_subdev_call(dev->sd,video,try_mbus_fmt,&ccm_fmt);//linux-3.0
	if (ret < 0) {
		csi_err("v4l2 sub device try_fmt error!\n");
		return ret;
	}

	//info got from module
	f->fmt.pix.width = ccm_fmt.width;//linux-3.0
	f->fmt.pix.height = ccm_fmt.height;//linux-3.0
//	f->fmt.pix.bytesperline = ccm_fmt.fmt.pix.bytesperline;//linux-3.0
//	f->fmt.pix.sizeimage = ccm_fmt.fmt.pix.sizeimage;//linux-3.0

	csi_dbg(0,"pix->width=%d\n",f->fmt.pix.width);
	csi_dbg(0,"pix->height=%d\n",f->fmt.pix.height);

	return 0;
}

static int vidioc_s_fmt_vid_cap(struct file *file, void *priv,
					struct v4l2_format *f)
{
	struct csi_dev *dev = video_drvdata(file);
	struct videobuf_queue *q = &dev->vb_vidq;
	int ret,width_buf,height_buf,width_len;
	struct v4l2_mbus_framefmt ccm_fmt;//linux-3.0
	struct csi_fmt *csi_fmt;
	struct v4l2_mbus_config cfg;

	csi_dbg(0,"vidioc_s_fmt_vid_cap\n");

	if (csi_is_generating(dev)) {
		csi_err("%s device busy\n", __func__);
		return -EBUSY;
	}

	mutex_lock(&q->vb_lock);

	ret = v4l2_subdev_call(dev->sd, video, g_mbus_config, &cfg);
	if (ret) {
		v4l2_err(dev->sd, "v4l2 sub device g_mbus_config error!\n");
		goto out;
	}

	if (cfg.flags & (V4L2_MBUS_HSYNC_ACTIVE_HIGH |
			V4L2_MBUS_HSYNC_ACTIVE_LOW))
	{
		dev->csi_mode.href = cfg.flags & V4L2_MBUS_HSYNC_ACTIVE_HIGH ? CSI_HIGH : CSI_LOW;
	} else {
		ret = -EINVAL;
	}

	if (cfg.flags & (V4L2_MBUS_VSYNC_ACTIVE_HIGH |
			V4L2_MBUS_VSYNC_ACTIVE_LOW))
	{
		dev->csi_mode.vref = cfg.flags & V4L2_MBUS_VSYNC_ACTIVE_HIGH ? CSI_HIGH : CSI_LOW;
	} else {
		ret = -EINVAL;
	}

	if (cfg.flags & (V4L2_MBUS_PCLK_SAMPLE_RISING |
			V4L2_MBUS_PCLK_SAMPLE_FALLING))
	{
		dev->csi_mode.clock = cfg.flags & V4L2_MBUS_PCLK_SAMPLE_RISING ? CSI_RISING : CSI_RISING;
	} else {
		ret = -EINVAL;
	}

	if (!ret) {
		v4l2_dbg(1, debug, &dev->v4l2_dev, "csi_mode.vref=%x\n", dev->csi_mode.vref);
		v4l2_dbg(1, debug, &dev->v4l2_dev, "csi_mode.href=%x\n", dev->csi_mode.href);
		v4l2_dbg(1, debug, &dev->v4l2_dev, "csi_mode.clock=%x\n", dev->csi_mode.clock);
	} else {
		v4l2_err(dev->sd, "g_mbus_config returns incorrect info\n");
		goto out;
	}

	ret = vidioc_try_fmt_vid_cap(file, priv, f);
	if (ret < 0) {
		csi_err("try format failed!\n");
		goto out;
	}

	csi_fmt = get_format(f);
	if (!csi_fmt) {
		csi_err("Fourcc format (0x%08x) invalid.\n",f->fmt.pix.pixelformat);
		ret	= -EINVAL;
		goto out;
	}

	ccm_fmt.code = csi_fmt->ccm_fmt;//linux-3.0
	ccm_fmt.width = f->fmt.pix.width;//linux-3.0
	ccm_fmt.height = f->fmt.pix.width;//linux-3.0

	ret = v4l2_subdev_call(dev->sd,video,s_mbus_fmt,&ccm_fmt);//linux-3.0
	if (ret < 0) {
		csi_err("v4l2 sub device s_fmt error!\n");
		goto out;
	}

	//save the current format info
	dev->fmt = csi_fmt;
	dev->vb_vidq.field = f->fmt.pix.field;
	dev->width  = f->fmt.pix.width;
	dev->height = f->fmt.pix.height;

	//set format
	dev->csi_mode.output_fmt = dev->fmt->output_fmt;
	dev->csi_mode.input_fmt = dev->fmt->input_fmt;

	switch(dev->fmt->ccm_fmt) {
	case V4L2_MBUS_FMT_YUYV8_2X8://linux-3.0
		if ((dev->fmt->fourcc == V4L2_PIX_FMT_NV61) || (dev->fmt->fourcc == V4L2_PIX_FMT_NV21))
			dev->csi_mode.seq = CSI_YVYU;
		else
			dev->csi_mode.seq = CSI_YUYV;
		break;
	case V4L2_MBUS_FMT_YVYU8_2X8://linux-3.0
		dev->csi_mode.seq = CSI_YVYU;
		break;
	case V4L2_MBUS_FMT_UYVY8_2X8://linux-3.0
		dev->csi_mode.seq = CSI_UYVY;
		break;
	case V4L2_MBUS_FMT_VYUY8_2X8://linux-3.0
		dev->csi_mode.seq = CSI_VYUY;
		break;
	default:
		dev->csi_mode.seq = CSI_YUYV;
		break;
	}

	switch(dev->fmt->input_fmt){
	case CSI_RAW:
		if ( (dev->fmt->fourcc == V4L2_PIX_FMT_YUYV) || (dev->fmt->fourcc == V4L2_PIX_FMT_YVYU) || \
				 (dev->fmt->fourcc == V4L2_PIX_FMT_UYVY) || (dev->fmt->fourcc == V4L2_PIX_FMT_VYUY)) {

			width_len  = dev->width*2;
			width_buf = dev->width*2;
			height_buf = dev->height;

		} else {
			width_len  = dev->width;
			width_buf = dev->width;
			height_buf = dev->height;
		}
		break;
	case CSI_BAYER:
		width_len  = dev->width;
		width_buf = dev->width;
		height_buf = dev->height;
		break;
	case CSI_CCIR656://TODO
	case CSI_YUV422:
		width_len  = dev->width;
		width_buf = dev->width*2;
		height_buf = dev->height;
		break;
	default:
		width_len  = dev->width;
		width_buf = dev->width*2;
		height_buf = dev->height;
		break;
	}

	bsp_csi_configure(dev,&dev->csi_mode);
	//horizontal and vertical offset are constant zero
	bsp_csi_set_size(dev,width_buf,height_buf,width_len);

	ret = 0;
out:
	mutex_unlock(&q->vb_lock);
	return ret;
}

static int vidioc_reqbufs(struct file *file, void *priv,
			  struct v4l2_requestbuffers *p)
{
	struct csi_dev *dev = video_drvdata(file);

	csi_dbg(0,"vidioc_reqbufs\n");

	return videobuf_reqbufs(&dev->vb_vidq, p);
}

static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct csi_dev *dev = video_drvdata(file);

	return videobuf_querybuf(&dev->vb_vidq, p);
}

static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct csi_dev *dev = video_drvdata(file);

	return videobuf_qbuf(&dev->vb_vidq, p);
}

static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
	struct csi_dev *dev = video_drvdata(file);

	return videobuf_dqbuf(&dev->vb_vidq, p, file->f_flags & O_NONBLOCK);
}

#ifdef CONFIG_VIDEO_V4L1_COMPAT
static int vidiocgmbuf(struct file *file, void *priv, struct video_mbuf *mbuf)
{
	struct csi_dev *dev = video_drvdata(file);

	return videobuf_cgmbuf(&dev->vb_vidq, mbuf, 8);
}
#endif


static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct csi_dev *dev = video_drvdata(file);
	struct csi_dmaqueue *dma_q = &dev->vidq;
	struct csi_buffer *buf;

	int ret;

	csi_dbg(0,"video stream on\n");
	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}

	if (csi_is_generating(dev)) {
		csi_err("stream has been already on\n");
		return 0;
	}

	/* Resets frame counters */
	dev->ms = 0;
	dev->jiffies = jiffies;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;

	ret = videobuf_streamon(&dev->vb_vidq);
	if (ret) {
		return ret;
	}

	buf = list_entry(dma_q->active.next,struct csi_buffer, vb.queue);
	csi_set_addr(dev,buf);

	bsp_csi_int_clear_status(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	bsp_csi_int_enable(dev, CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	bsp_csi_capture_video_start(dev);
	csi_start_generating(dev);

	return 0;
}


static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
	struct csi_dev *dev = video_drvdata(file);
	struct csi_dmaqueue *dma_q = &dev->vidq;
	int ret;

	csi_dbg(0,"video stream off\n");

	if (!csi_is_generating(dev)) {
		csi_err("stream has been already off\n");
		return 0;
	}

	csi_stop_generating(dev);

	/* Resets frame counters */
	dev->ms = 0;
	dev->jiffies = jiffies;

	dma_q->frame = 0;
	dma_q->ini_jiffies = jiffies;

	bsp_csi_int_disable(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	bsp_csi_int_clear_status(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	bsp_csi_capture_video_stop(dev);

	if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		return -EINVAL;
	}

	ret = videobuf_streamoff(&dev->vb_vidq);
	if (ret!=0) {
		csi_err("videobu_streamoff error!\n");
		return ret;
	}

	if (ret!=0) {
		csi_err("videobuf_mmap_free error!\n");
		return ret;
	}

	return 0;
}


static int vidioc_enum_input(struct file *file, void *priv,
				struct v4l2_input *inp)
{
	struct csi_dev *dev = video_drvdata(file);

	if (inp->index > dev->dev_qty-1) {
		csi_err("input index invalid!\n");
		return -EINVAL;
	}

	inp->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
	struct csi_dev *dev = video_drvdata(file);

	*i = dev->input;
	return 0;
}

static int internal_s_input(struct csi_dev *dev, unsigned int i)
{
	int ret;

	if (i > dev->dev_qty-1) {
		csi_err("set input error!\n");
		return -EINVAL;
	}

	if (i == dev->input)
		return 0;

	csi_dbg(0,"input_num = %d\n",i);

//	spin_lock(&dev->slock);

	/*Power down current device*/
	ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_ON);
	if(ret < 0)
		goto altend;

	/* Alternate the device info and select target device*/
	update_ccm_info(dev, &dev->ccm_cfg[i]);

	/* change the csi setting */
	v4l2_dbg(1, debug, &dev->v4l2_dev, "dev->mclk = %d\n", dev->mclk);

//  bsp_csi_configure(dev,&dev->csi_mode);
	csi_clk_out_set(dev);

	/* Initial target device */
	ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_OFF);
	if (ret!=0) {
	  csi_err("sensor standby off error when selecting target device!\n");
	  goto recover;
	}
	ret = v4l2_subdev_call(dev->sd,core, init, 0);
	if (ret!=0) {
		csi_err("sensor initial error when selecting target device!\n");
		goto recover;
	}

	dev->input = i;
  ret = 0;

altend:
//  spin_unlock(&dev->slock);
	return ret;

recover:
	/*Power down target device*/
	ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_ON);
	if(ret < 0)
		goto altend;

	/* Alternate the device info and select the current device*/
	update_ccm_info(dev, &dev->ccm_cfg[dev->input]);

	/*Re Initial current device*/
	ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_OFF);
	if (ret!=0) {
	  csi_err("sensor standby off error when selecting back current device!\n");
	  goto recover;
	}

	ret = v4l2_subdev_call(dev->sd,core, init, 0);
	if (ret!=0) {
		csi_err("sensor recovering error when selecting back current device!\n");
	}
	ret = 0;
	goto altend;
}

static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
	struct csi_dev *dev = video_drvdata(file);

	return internal_s_input(dev , i);
}


static int vidioc_queryctrl(struct file *file, void *priv,
			    struct v4l2_queryctrl *qc)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(dev->sd,core,queryctrl,qc);
	if (ret < 0)
		csi_err("v4l2 sub device queryctrl error!\n");

	return ret;
}

static int vidioc_g_ctrl(struct file *file, void *priv,
			 struct v4l2_control *ctrl)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(dev->sd,core,g_ctrl,ctrl);
	if (ret < 0)
		csi_err("v4l2 sub device g_ctrl error!\n");

	return ret;
}


static int vidioc_s_ctrl(struct file *file, void *priv,
				struct v4l2_control *ctrl)
{
	struct csi_dev *dev = video_drvdata(file);
	struct v4l2_queryctrl qc;
	int ret;

	qc.id = ctrl->id;
	ret = vidioc_queryctrl(file, priv, &qc);
	if (ret < 0) {
		return ret;
	}

	if (ctrl->value < qc.minimum || ctrl->value > qc.maximum) {
		return -ERANGE;
	}

	ret = v4l2_subdev_call(dev->sd,core,s_ctrl,ctrl);
	if (ret < 0)
		csi_err("v4l2 sub device s_ctrl error!\n");

	return ret;
}

static int vidioc_g_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *parms)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(dev->sd,video,g_parm,parms);
	if (ret < 0)
		csi_err("v4l2 sub device g_parm error!\n");

	return ret;
}

static int vidioc_s_parm(struct file *file, void *priv,
			 struct v4l2_streamparm *parms)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret;

	ret = v4l2_subdev_call(dev->sd,video,s_parm,parms);
	if (ret < 0)
		csi_err("v4l2 sub device s_parm error!\n");

	return ret;
}


static ssize_t csi_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
	struct csi_dev *dev = video_drvdata(file);

//	csi_start_generating(dev);
	if(csi_is_generating(dev)) {
		return videobuf_read_stream(&dev->vb_vidq, data, count, ppos, 0,
					file->f_flags & O_NONBLOCK);
	} else {
		csi_err("csi is not generating!\n");
		return -EINVAL;
	}
}

static unsigned int csi_poll(struct file *file, struct poll_table_struct *wait)
{
	struct csi_dev *dev = video_drvdata(file);
	struct videobuf_queue *q = &dev->vb_vidq;

//	csi_start_generating(dev);
	if(csi_is_generating(dev)) {
		return videobuf_poll_stream(file, q, wait);
	} else {
		csi_err("csi is not generating!\n");
		return -EINVAL;
	}
}

static int csi_open(struct file *file)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret,input_num;

	csi_dbg(0,"csi_open\n");

	if (dev->opened == 1) {
		csi_err("device open busy\n");
		return -EBUSY;
	}

	csi_clk_enable(dev);
	csi_reset_disable(dev);
	//open all the device power and set it to standby on
	for (input_num=dev->dev_qty-1; input_num>=0; input_num--) {
		/* update target device info and select it*/
		update_ccm_info(dev, &dev->ccm_cfg[input_num]);

		csi_clk_out_set(dev);

		ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_PWR_ON);
	  if (ret!=0) {
	  	csi_err("sensor CSI_SUBDEV_PWR_ON error at device number %d when csi open!\n",input_num);
	  }

		ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_ON);
		if (ret!=0) {
	  	csi_err("sensor CSI_SUBDEV_STBY_ON error at device number %d when csi open!\n",input_num);
	  }
	}

	dev->input=0;//default input

	bsp_csi_open(dev);
	bsp_csi_set_offset(dev,0,0);//h and v offset is initialed to zero

	ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_OFF);
	if (ret!=0) {
	  csi_err("sensor standby off error when csi open!\n");
	  return ret;
	}
	ret = v4l2_subdev_call(dev->sd,core, init, 0);
	if (ret!=0) {
		csi_err("sensor initial error when csi open!\n");
		return ret;
	} else {
		csi_print("sensor initial success when csi open!\n");
	}

	dev->opened=1;
	dev->fmt = &formats[5]; //default format
	return 0;
}

static int csi_close(struct file *file)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret,input_num;

	csi_dbg(0,"csi_close\n");

	bsp_csi_int_disable(dev,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	//bsp_csi_int_clear_status(dev,CSI_INT_FRAME_DONE);

	bsp_csi_capture_video_stop(dev);
	bsp_csi_close(dev);

	csi_clk_disable(dev);
	csi_reset_enable(dev);


	videobuf_stop(&dev->vb_vidq);
	videobuf_mmap_free(&dev->vb_vidq);

	dev->opened=0;
	csi_stop_generating(dev);

	if(dev->stby_mode == 0) {
		return v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_ON);
	} else {

		//close all the device power
		for (input_num=0; input_num<dev->dev_qty; input_num++) {
      /* update target device info and select it */
			update_ccm_info(dev, &dev->ccm_cfg[input_num]);
			ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_PWR_OFF);
		  if (ret!=0) {
		  	csi_err("sensor power off error at device number %d when csi open!\n",input_num);
		  	return ret;
		  }
		}
	}

	return 0;
}

static int csi_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct csi_dev *dev = video_drvdata(file);
	int ret;

	csi_dbg(0,"mmap called, vma=0x%08lx\n", (unsigned long)vma);

	ret = videobuf_mmap_mapper(&dev->vb_vidq, vma);

	csi_dbg(0,"vma start=0x%08lx, size=%ld, ret=%d\n",
		(unsigned long)vma->vm_start,
		(unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
		ret);
	return ret;
}

static const struct v4l2_file_operations csi_fops = {
	.owner	  = THIS_MODULE,
	.open	  = csi_open,
	.release  = csi_close,
	.read     = csi_read,
	.poll	  = csi_poll,
	.ioctl    = video_ioctl2,
	.mmap     = csi_mmap,
};

static const struct v4l2_ioctl_ops csi_ioctl_ops = {
	.vidioc_querycap          = vidioc_querycap,
	.vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,
	.vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,
	.vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,
	.vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,
	.vidioc_reqbufs           = vidioc_reqbufs,
	.vidioc_querybuf          = vidioc_querybuf,
	.vidioc_qbuf              = vidioc_qbuf,
	.vidioc_dqbuf             = vidioc_dqbuf,
	.vidioc_enum_input        = vidioc_enum_input,
	.vidioc_g_input           = vidioc_g_input,
	.vidioc_s_input           = vidioc_s_input,
	.vidioc_streamon          = vidioc_streamon,
	.vidioc_streamoff         = vidioc_streamoff,
	.vidioc_queryctrl         = vidioc_queryctrl,
	.vidioc_g_ctrl            = vidioc_g_ctrl,
	.vidioc_s_ctrl            = vidioc_s_ctrl,
	.vidioc_g_parm		 			  = vidioc_g_parm,
	.vidioc_s_parm		  			= vidioc_s_parm,

#ifdef CONFIG_VIDEO_V4L1_COMPAT
	.vidiocgmbuf              = vidiocgmbuf,
#endif
};

static struct video_device csi_template = {
	.name		= "csi",
	.fops       = &csi_fops,
	.ioctl_ops 	= &csi_ioctl_ops,
	.release	= video_device_release,
};

static int fetch_sensor_config(struct i2c_board_info *binfo)
{
	struct csi_sensor_platform_data *sensor_pdata = binfo->platform_data;
	int ret;

	/* fetch i2c address and module name */
	if((i2c_addr == 0xff) && (strlen(ccm) == 0))	/* when insmod without parm */
	{
		int tmp_addr;

		ret = script_parser_fetch("csi1_para", "csi_twi_addr", &tmp_addr, sizeof(int));
		binfo->addr = tmp_addr >> 1;
		if (ret) {
			csi_err("fetch csi_twi_addr from sys_config failed\n");
			return ret;
		}

		ret = script_parser_fetch("csi1_para", "csi_mname", (int *)&binfo->type, sizeof(binfo->type));
		if (ret) {
			csi_err("fetch csi_mname from sys_config failed\n");
			return ret;
		}
	} else {
		binfo->addr = i2c_addr >> 1;
		strlcpy(binfo->type, ccm, sizeof(binfo->type));
	}

	/* fetch power regulators configuration */
	ret = script_parser_fetch("csi1_para","csi_iovdd", (int *)&sensor_pdata->iovdd_str,
					sizeof(sensor_pdata->iovdd_str));
	if (ret) {
		csi_err("fetch csi_iovdd from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_avdd", (int *)&sensor_pdata->avdd_str,
					sizeof(sensor_pdata->avdd_str));
	if (ret) {
		csi_err("fetch csi_avdd from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_dvdd", (int *)&sensor_pdata->dvdd_str,
					sizeof(sensor_pdata->dvdd_str));
	if (ret) {
		csi_err("fetch csi_dvdd from sys_config failed\n");
		return ret;
	}

	/* fetch camera io configuration */
	ret = script_parser_fetch("csi1_para","csi_reset", (int *)&sensor_pdata->reset , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_reset from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_stby", (int *)&sensor_pdata->stby , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_stby from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_power_en", (int *)&sensor_pdata->power , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_power_en from sys_config failed\n");
		return ret;
	}

	/* fetch auto focus io configuration */
/*
	ret = script_parser_fetch("csi1_para","csi_af_en", (int *)&sensor_pdata->af_power , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_af_en from sys_config failed\n");
		return ret;
	}
*/
	/* fetch flash io configuration */
	ret = script_parser_fetch("csi1_para","csi_flash", (int *)&sensor_pdata->flash , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_flash from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_flash_pol", (int *)&sensor_pdata->flash_pol , sizeof(int));
	if (ret) {
		csi_err("fetch csi1 csi_flash_pol from sys_config failed\n");
		return ret;
	}

	/* fetch interface issue*/
	ret = script_parser_fetch("csi1_para","csi_if", (int *)&sensor_pdata->interface , sizeof(int));
	if (ret) {
		csi_err("fetch csi_if from sys_config failed\n");
		return ret;
	}

	/* fetch flip issue */
	ret = script_parser_fetch("csi1_para", "csi_vflip", &sensor_pdata->inv_vflip, sizeof(int));
	if (ret) {
		csi_err("fetch csi1 vflip from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para", "csi_hflip", &sensor_pdata->inv_hflip, sizeof(int));
	if (ret) {
		csi_err("fetch csi1 hflip from sys_config failed\n");
		return ret;
	}

	/* TODO: Add mclk parameter to FEX file */
	sensor_pdata->mclk = 27 * 1000 * 1000; /* ov7670 MCLK */

	csi_dbg(0, "Sensor 0:\n");
	csi_print_sensor_i2c_board_info(binfo);

	return 0;
}

static int fetch_sensor_b_config(struct i2c_board_info *binfo)
{
	struct csi_sensor_platform_data *sensor_pdata = binfo->platform_data;
	int ret;

	/* fetch i2c address and module name */
	if((i2c_addr_b == 0xff) && (strlen(ccm_b) == 0))	/* when insmod without parm */
	{
		int tmp_addr;

		ret = script_parser_fetch("csi1_para", "csi_twi_addr_b", &tmp_addr, sizeof(int));
		binfo->addr = tmp_addr >> 1;
		if (ret) {
			csi_err("fetch csi_twi_addr_b from sys_config failed\n");
			return ret;
		}

		ret = script_parser_fetch("csi1_para", "csi_mname_b", (int *)&binfo->type, sizeof(binfo->type));
		if (ret) {
			csi_err("fetch csi_mname_b from sys_config failed\n");
			return ret;
		}
	} else {
		binfo->addr = i2c_addr >> 1;
		strlcpy(binfo->type, ccm, sizeof(binfo->type));
	}

	/* fetch power regulators configuration */
	ret = script_parser_fetch("csi1_para","csi_iovdd_b", (int *)&sensor_pdata->iovdd_str,
					sizeof(sensor_pdata->iovdd_str));
	if (ret) {
		csi_err("fetch csi_iovdd_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_avdd_b", (int *)&sensor_pdata->avdd_str,
					sizeof(sensor_pdata->avdd_str));
	if (ret) {
		csi_err("fetch csi_avdd_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_dvdd_b", (int *)&sensor_pdata->dvdd_str,
					sizeof(sensor_pdata->dvdd_str));
	if (ret) {
		csi_err("fetch csi_dvdd_b from sys_config failed\n");
		return ret;
	}

	/* fetch camera io configuration */
	ret = script_parser_fetch("csi1_para","csi_reset_b", (int *)&sensor_pdata->reset , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_reset_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_stby_b", (int *)&sensor_pdata->stby , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_stby_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_power_en_b", (int *)&sensor_pdata->power , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_power_en_b from sys_config failed\n");
		return ret;
	}

	/* fetch auto focus io configuration */
/*
	ret = script_parser_fetch("csi1_para","csi_af_en_b", (int *)&sensor_pdata->af_power , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_af_en_b from sys_config failed\n");
		return ret;
	}
*/
	/* fetch flash io configuration */
	ret = script_parser_fetch("csi1_para","csi_flash_b", (int *)&sensor_pdata->flash , sizeof(user_gpio_set_t)/sizeof(int));
	if (ret) {
		csi_err("fetch csi_flash_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para","csi_flash_pol_b", (int *)&sensor_pdata->flash_pol , sizeof(int));
	if (ret) {
		csi_err("fetch csi_flash_pol_b from sys_config failed\n");
		return ret;
	}

	/* fetch interface issue*/
	ret = script_parser_fetch("csi1_para","csi_if_b", (int *)&sensor_pdata->interface , sizeof(int));
	if (ret) {
		csi_err("fetch csi_if_b from sys_config failed\n");
		return ret;
	}

	/* fetch flip issue */
	ret = script_parser_fetch("csi1_para", "csi_vflip_b", &sensor_pdata->inv_vflip, sizeof(int));
	if (ret) {
		csi_err("fetch csi1 vflip_b from sys_config failed\n");
		return ret;
	}

	ret = script_parser_fetch("csi1_para", "csi_hflip_b", &sensor_pdata->inv_hflip, sizeof(int));
	if (ret) {
		csi_err("fetch csi1 hflip_b from sys_config failed\n");
		return ret;
	}

	/* TODO: Add mclk parameter to FEX file */
	sensor_pdata->mclk = 27 * 1000 * 1000; /* ov7670 MCLK */

	csi_dbg(0, "Sensor 1:\n");
	csi_print_sensor_i2c_board_info(binfo);

	return 0;
}

static int csi_probe(struct platform_device *pdev)
{
	struct csi_platform_data *csi_pdata = pdev->dev.platform_data;
	struct csi_sensor_platform_data *sensor_pdata;
	struct csi_dev *dev;
	struct resource *res;
	int irq;
	struct video_device *vfd;
	struct i2c_adapter *i2c_adap;
	int ret = 0;
	int input_num;

	csi_dbg(0,"csi_probe\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	irq = platform_get_irq(pdev, 0);
	if (!res || irq <= 0) {
		dev_err(&pdev->dev, "Not enough CSI platform resources.\n");
		return -ENODEV;
	}

	dev = devm_kzalloc(&pdev->dev, sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Could not allocate dev\n");
		return -ENOMEM;
	}

	dev->pdev = pdev;

	spin_lock_init(&dev->slock);

	dev->regs = devm_request_and_ioremap(&pdev->dev, res);
	if (!dev->regs)
		return -EADDRNOTAVAIL;

	ret = devm_request_irq(&pdev->dev, irq, csi_isr, 0,
				dev_name(&pdev->dev), dev);
	if (ret) {
		dev_err(&pdev->dev, "cannot claim IRQ %d\n", ret);
		return ret;
	}

    /*pin resource*/
	dev->csi_pin_hd = gpio_request_ex("csi1_para",NULL);
	if (dev->csi_pin_hd==-1) {
		csi_err("csi1 pin request error!\n");
		ret = -ENXIO;
		goto err_irq;
	}

    /* v4l2 device register */
	ret = v4l2_device_register(&pdev->dev, &dev->v4l2_dev);
	if (ret) {
		csi_err("Error registering v4l2 device\n");
		goto err_irq;

	}

	dev_set_drvdata(&(pdev)->dev, (dev));

	dev->dev_qty = csi_pdata->dev_qty;
	dev->stby_mode = csi_pdata->stby_mode;

  /* v4l2 subdev register	*/
	for(input_num=0; input_num<dev->dev_qty; input_num++)
	{
		sensor_pdata = dev_sensor[input_num].platform_data;

		i2c_adap = i2c_get_adapter(csi_pdata->i2c_adapter_id[input_num]);

		if (i2c_adap == NULL) {
			csi_err("request i2c adapter failed,input_num = %d\n",input_num);
			ret = -EINVAL;
			goto free_dev;//linux-3.0
		}

		dev->ccm_cfg[input_num].sd = v4l2_i2c_new_subdev_board(&dev->v4l2_dev, i2c_adap,
									&dev_sensor[input_num], NULL);
		if (!dev->ccm_cfg[input_num].sd) {
			csi_err("Error registering v4l2 subdevice,input_num = %d\n",input_num);
			goto free_dev;
		} else{
			csi_print("registered sub device,input_num = %d\n",input_num);
		}

		dev->ccm_cfg[input_num].mclk =  sensor_pdata->mclk;

		/*power issue*/

		if(dev->stby_mode == 1) {
			csi_print("power on and power off camera!\n");
			update_ccm_info(dev, &dev->ccm_cfg[input_num]);
			v4l2_subdev_call(dev->ccm_cfg[input_num].sd,core, s_power, CSI_SUBDEV_PWR_ON);
			v4l2_subdev_call(dev->ccm_cfg[input_num].sd,core, s_power, CSI_SUBDEV_PWR_OFF);
		}
	}

	for(input_num=0; input_num<dev->dev_qty; input_num++)
	{
		v4l2_dbg(1, debug, &dev->v4l2_dev, "dev->ccm_cfg[%d].sd = %p\n", input_num, dev->ccm_cfg[input_num].sd);
		v4l2_dbg(1, debug, &dev->v4l2_dev, "dev->ccm_cfg[%d].mclk = %d\n", input_num, dev->ccm_cfg[input_num].mclk);
	}

	update_ccm_info(dev, &dev->ccm_cfg[0]);

	/*clock resource*/
	if (csi_clk_get(dev)) {
		csi_err("csi clock get failed!\n");
		ret = -ENXIO;
		goto unreg_dev;
	}

	/*video device register	*/
	ret = -ENOMEM;
	vfd = video_device_alloc();
	if (!vfd) {
		goto err_clk;
	}

	*vfd = csi_template;
	vfd->v4l2_dev = &dev->v4l2_dev;

	dev_set_name(&vfd->dev, "csi-1");
	ret = video_register_device(vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		goto rel_vdev;
	}
	video_set_drvdata(vfd, dev);

	/*add device list*/
	/* Now that everything is fine, let's add it to device list */
	list_add_tail(&dev->csi_devlist, &csi_devlist);

	if (video_nr != -1) {
		video_nr++;
	}
	dev->vfd = vfd;

	csi_print("V4L2 device registered as %s\n",video_device_node_name(vfd));

	/*initial video buffer queue*/
	videobuf_queue_dma_contig_init(&dev->vb_vidq, &csi_video_qops,
			NULL, &dev->slock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			V4L2_FIELD_NONE,
			sizeof(struct csi_buffer), dev,NULL);//linux-3.0

	/* init video dma queues */
	INIT_LIST_HEAD(&dev->vidq.active);
	//init_waitqueue_head(&dev->vidq.wq);

	return 0;

rel_vdev:
	video_device_release(vfd);
err_clk:
	csi_clk_release(dev);
unreg_dev:
	v4l2_device_unregister(&dev->v4l2_dev);
free_dev:
err_irq:

	return ret;
}
void csi_dev_release(struct device *dev)
{
}

static int csi_release(void)
{
	struct csi_dev *dev;
	struct list_head *list;

	csi_dbg(0,"csi_release\n");
	while (!list_empty(&csi_devlist))
	{
		list = csi_devlist.next;
		list_del(list);
		dev = list_entry(list, struct csi_dev, csi_devlist);

		v4l2_info(&dev->v4l2_dev, "unregistering %s\n", video_device_node_name(dev->vfd));
		video_unregister_device(dev->vfd);
		csi_clk_release(dev);
		v4l2_device_unregister(&dev->v4l2_dev);
	}

	csi_print("csi_release ok!\n");
	return 0;
}

static int __devexit csi_remove(struct platform_device *pdev)
{
	struct csi_dev *dev;
	dev=(struct csi_dev *)dev_get_drvdata(&(pdev)->dev);

	csi_dbg(0,"csi_remove\n");

//	video_device_release(vfd);
//	csi_clk_release(dev);

	csi_print("csi_remove ok!\n");
	return 0;
}

static int csi_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(&(pdev)->dev);
	int ret,input_num;

	csi_print("csi_suspend\n");

	if (dev->opened==1) {
		csi_clk_disable(dev);

		if(dev->stby_mode == 0) {
			csi_print("set camera to standby!\n");
			return v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_STBY_ON);
		} else {
			csi_print("set camera to power off!\n");
			//close all the device power
			for (input_num=0; input_num<dev->dev_qty; input_num++) {
        /* update target device info and select it */
				update_ccm_info(dev, &dev->ccm_cfg[input_num]);
				ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_PWR_OFF);
			  if (ret!=0) {
			  	csi_err("sensor power off error at device number %d when csi_suspend!\n",input_num);
			  	return ret;
			  }
			}
		}
	}
	return 0;
}

static int csi_resume(struct platform_device *pdev)
{
	int ret,input_num;
	struct csi_dev *dev=(struct csi_dev *)dev_get_drvdata(&(pdev)->dev);

	csi_print("csi_resume\n");

	if (dev->opened==1) {
		csi_clk_out_set(dev);
//		csi_clk_enable(dev);
		if(dev->stby_mode == 0) {
			ret = v4l2_subdev_call(dev->sd,core, s_power,CSI_SUBDEV_STBY_OFF);
			if(ret < 0)
				return ret;
			ret = v4l2_subdev_call(dev->sd,core, init, 0);
			if (ret!=0) {
				csi_err("sensor initial error when resume from suspend!\n");
				return ret;
			} else {
				csi_print("sensor initial success when resume from suspend!\n");
			}
		} else {
			//open all the device power
			for (input_num=0; input_num<dev->dev_qty; input_num++) {
        /* update target device info and select it */
				update_ccm_info(dev, &dev->ccm_cfg[input_num]);
				ret = v4l2_subdev_call(dev->sd,core, s_power, CSI_SUBDEV_PWR_ON);
			  if (ret!=0) {
			  	csi_err("sensor power on error at device number %d when csi_resume!\n",input_num);
			  }
			}

			/* update target device info and select it */
			update_ccm_info(dev, &dev->ccm_cfg[0]);
			ret = v4l2_subdev_call(dev->sd,core, init,0);
			if (ret!=0) {
				csi_err("sensor full initial error when resume from suspend!\n");
				return ret;
			} else {
				csi_print("sensor full initial success when resume from suspend!\n");
			}
		}
	}

	return 0;
}


static struct platform_driver csi_driver = {
	.probe		= csi_probe,
	.remove		= __devexit_p(csi_remove),
	.suspend	= csi_suspend,
	.resume		= csi_resume,
	//.id_table	= csi_driver_ids,
	.driver = {
		.name	= "sun4i_csi1",
		.owner	= THIS_MODULE,
	}
};

static struct resource csi1_resource[] = {
	[0] = {
		.start	= CSI1_REGS_BASE,
		.end	= CSI1_REGS_BASE + CSI1_REG_SIZE - 1,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= SW_INTC_IRQNO_CSI1,
		.end	= SW_INTC_IRQNO_CSI1,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device csi_device[] = {
	[0] = {
	.name           	= "sun4i_csi1",
    	.id             	= 0,
	.num_resources		= ARRAY_SIZE(csi1_resource),
    	.resource       	= csi1_resource,
	.dev.release      = csi_dev_release,
	}
};

static int fetch_csi_config(struct csi_platform_data *pdata)
{
	int ret;

	/* fetch device quatity issue */
	ret = script_parser_fetch("csi1_para", "csi_dev_qty", &pdata->dev_qty, sizeof(int));
	if (ret) {
		csi_err("fetch csi_dev_qty from sys_config failed\n");
		return ret;
	}

	/* fetch standby mode */
	ret = script_parser_fetch("csi1_para", "csi_stby_mode", &pdata->stby_mode, sizeof(int));
	if (ret) {
		csi_err("fetch csi_stby_mode from sys_config failed\n");
		return ret;
	}

	pdata->i2c_adapter_id[0] = 0;
	pdata->i2c_adapter_id[1] = 0;

	if (pdata->dev_qty > 0) {
		ret = script_parser_fetch("csi1_para", "csi_twi_id", &pdata->i2c_adapter_id[0], sizeof(int));
		if (ret) {
			csi_err("fetch csi_twi_id from sys_config failed\n");
			return ret;
		}
	}

	if (pdata->dev_qty > 1) {
		ret = script_parser_fetch("csi1_para", "csi_twi_id_b", &pdata->i2c_adapter_id[1], sizeof(int));
		if (ret) {
			csi_err("fetch csi_twi_id_b from sys_config failed\n");
			return ret;
		}
	}

	/* Dump csi_platform_data */
	csi_dbg(0, "csi_platform_data:\n");
	csi_dbg(0, "csi_platform_data->dev_qty = %d\n", pdata->dev_qty);
	csi_dbg(0, "csi_platform_data->stby_mode = %d\n", pdata->stby_mode);
	csi_dbg(0, "csi_platform_data->i2c_adapter_id[0] = %d\n", pdata->i2c_adapter_id[0]);
	csi_dbg(0, "csi_platform_data->i2c_adapter_id[1] = %d\n", pdata->i2c_adapter_id[1]);

	return 0;
}

static int __init csi_init(void)
{
	u32 ret;
	int csi_used;
	struct csi_platform_data pdata;

	csi_print("Welcome to CSI driver\n");
	csi_print("csi_init\n");

	ret = script_parser_fetch("csi1_para","csi_used", &csi_used , sizeof(int));
	if (ret) {
		csi_err("fetch csi_used from sys_config failed\n");
		return -1;
	}

	if(!csi_used)
	{
		csi_err("csi_used=0,csi driver is not enabled!\n");
		return 0;
	}

	ret = fetch_csi_config(&pdata);
	if (ret) {
		return -1;
	}

	if (pdata.dev_qty == 0) {
		csi_err("dev_qty=0, csi camera sub-driver is not enabled!\n");
		return -1;
	}

	ret = fetch_sensor_config (&dev_sensor[0]);
	if (ret) {
		return -1;
	}

	if (pdata.dev_qty > 1) {
		ret = fetch_sensor_b_config (&dev_sensor[1]);
		if (ret) {
			return -1;
		}
	}

	ret = platform_driver_register(&csi_driver);
	if (ret) {
		csi_err("platform driver register failed\n");
		return -1;
	}

	ret = platform_device_add_data(&csi_device[0], &pdata, sizeof(pdata));
	if (ret) {
		csi_err("platform device add data failed\n");
		return -1;
	}

	ret = platform_device_register(&csi_device[0]);
	if (ret) {
		csi_err("platform device register failed\n");
		return -1;
	}
	return 0;
}

static void __exit csi_exit(void)
{
	int csi_used,ret;

	csi_print("csi_exit\n");

	ret = script_parser_fetch("csi1_para","csi_used", &csi_used , sizeof(int));
	if (ret) {
		csi_err("fetch csi_used from sys_config failed\n");
		return;
	}

	if(csi_used)
	{
		csi_release();
		platform_device_unregister(&csi_device[0]);
		platform_driver_unregister(&csi_driver);
	}
}

module_init(csi_init);
module_exit(csi_exit);

MODULE_AUTHOR("raymonxiu");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("CSI driver for sun4i");
