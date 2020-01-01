/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_driver.c $
 * $Revision: #22 $
 * $Date: 2008/05/15 $
 * $Change: 1033974 $
 *
 * Synopsys HS OTG Linux Software Driver and documentation (hereinafter,
 * "Software") is an Unsupported proprietary work of Synopsys, Inc. unless
 * otherwise expressly agreed to in writing between Synopsys and you.
 *
 * The Software IS NOT an item of Licensed Software or Licensed Product under
 * any End User Software License Agreement or Agreement for Licensed Product
 * with Synopsys or any supplement thereto. You are permitted to use and
 * redistribute this Software in source and binary forms, with or without
 * modification, provided that redistributions of source code must retain this
 * notice. You may not view, use, disclose, copy or distribute this file or
 * any information contained herein except pursuant to this license grant from
 * Synopsys. If you do not agree with this notice, including the disclaimer
 * below, then you are not authorized to use the Software.
 *
 * THIS SOFTWARE IS BEING DISTRIBUTED BY SYNOPSYS SOLELY ON AN "AS IS" BASIS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE HEREBY DISCLAIMED. IN NO EVENT SHALL SYNOPSYS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 * ========================================================================== */

/** @file
 * The dwc_otg_driver module provides the initialization and cleanup entry
 * points for the DWC_otg driver. This module will be dynamically installed
 * after Linux is booted using the insmod command. When the module is
 * installed, the dwc_otg_driver_init function is called. When the module is
 * removed (using rmmod), the dwc_otg_driver_cleanup function is called.
 *
 * This module also defines a data structure for the dwc_otg_driver, which is
 * used in conjunction with the standard ARM platform_device structure. These
 * structures allow the OTG driver to comply with the standard Linux driver
 * model in which devices and drivers are registered with a bus driver. This
 * has the benefit that Linux can expose attributes of the driver and device
 * in its special sysfs file system. Users can then read or write files in
 * this file system to perform diagnostics on the driver components or the
 * device.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/stat.h>	 /* permission constants */
#include <linux/version.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/clk.h>

#include <asm/fiq.h>
#include <asm/io.h>
#include <asm/sizes.h>
#include <mach/gpio.h>
#include <mach/usb.h>

#include "linux/dwc_otg_plat.h"
#include "dwc_otg_attr.h"
#include "dwc_otg_driver.h"
#include "dwc_otg_cil.h"
#include "dwc_otg_pcd.h"
#include "dwc_otg_hcd.h"

#define DWC_DRIVER_VERSION	"2.70a-parrot 22/03/2009"
#define DWC_DRIVER_DESC		"HS OTG USB controller driver for PARROT asics"

static const char dwc_driver_name[] = "dwc_otg";

static struct fiq_handler fh = {
	.name	= "dwc_otg"
};

/* module parameters */
static int usb1_disable_fiq;
module_param(usb1_disable_fiq, int, 0444);

static dwc_otg_core_params_t dwc_otg_module_params[CTRL_NUM] =
{
	/* default parameters for controller0 */
	{
		.opt = 0,
		.otg_cap = DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE,
		.dma_enable = 1, /* default */
		.dma_desc_enable = 0,
		.dma_burst_size = -1, /* not used */
		.speed = DWC_SPEED_PARAM_HIGH, /* default */
		.enable_dynamic_fifo = 1,
		.dev_rx_fifo_size = 0x610,
		.dev_nperio_tx_fifo_size = 0xD0,
		.dev_perio_tx_fifo_size = { /* not used */
			-1, -1, -1, -1,	-1,
			-1, -1, -1, -1,	-1,
			-1, -1,	-1, -1,	-1
		},
		.host_rx_fifo_size = 0x6A0, /* default */
		.host_nperio_tx_fifo_size = 0x6A0, /* default */
		.host_perio_tx_fifo_size = 0x6A0, /* default */
		.max_transfer_size = 524288,
		.max_packet_count = 1024,
		.host_channels = 16,
		.dev_endpoints = 15,
		.phy_type = DWC_PHY_TYPE_PARAM_ULPI,
		.phy_utmi_width = -1, /* not used */
		.phy_ulpi_ddr = 0,
		.phy_ulpi_ext_vbus = DWC_PHY_ULPI_EXTERNAL_VBUS,
		.i2c_enable = 0,
		.ulpi_fs_ls = 0,
		.ts_dline = 0,
		.en_multiple_tx_fifo = 1,
		.dev_tx_fifo_size = {
			0xD0, 0xD0, 0xD0, 0xD0, 0xD0,
			0xD0, 0xD0, 0xD0, 0xD0, 0xD0,
			-1, -1,	-1, -1,	-1
		},
		.thr_ctl = 0,
		.tx_thr_length = 64,
		.rx_thr_length = 64,
		.reset_pin = -1, /* default */
		.ctrl_mode = DWC_OTG_HOST_ONLY, /* default */
		.sof_filter = 0, /* default */
		.vbus_detection = 0, /* default */
		.overcurrent_pin = 89, /* default */
		.xfer_debug = 0, /* default */
		.fiq_enable = 0 /* default */
	},

	/* default parameters for controller1 */
	{
		.opt = 0,
		.otg_cap = DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE,
		.dma_enable = 1, /* default */
		.dma_desc_enable = 0,
		.dma_burst_size = -1, /* not used */
		.speed = DWC_SPEED_PARAM_FULL, /* default */
		.enable_dynamic_fifo = 1,
		.dev_rx_fifo_size = -1, /* not used */
		.dev_nperio_tx_fifo_size = -1, /* not used */
		.dev_perio_tx_fifo_size = { /* not used */
			-1, -1, -1, -1,	-1,
			-1, -1, -1, -1,	-1,
			-1, -1,	-1, -1,	-1
		},
		.host_rx_fifo_size = 0x400, /* default */
		.host_nperio_tx_fifo_size = 0x400, /* default */
		.host_perio_tx_fifo_size = 0x400, /* default */
		.max_transfer_size = 524288,
		.max_packet_count = 1024,
		.host_channels = 8,
		.dev_endpoints = -1, /* not used */
		.phy_type = DWC_PHY_TYPE_PARAM_ULPI,
		.phy_utmi_width = -1, /* not used */
		.phy_ulpi_ddr = 0,
		.phy_ulpi_ext_vbus = DWC_PHY_ULPI_EXTERNAL_VBUS,
		.i2c_enable = 0,
		.ulpi_fs_ls = 0,
		.ts_dline = 0,
		.en_multiple_tx_fifo = -1, /* not used */
		.dev_tx_fifo_size = { /* not used */
			-1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1,
			-1, -1, -1, -1, -1
		},
		.thr_ctl = -1, /* not used */
		.tx_thr_length = -1, /* not used */
		.rx_thr_length = -1, /* not used */
		.reset_pin = -1, /* default */
		.ctrl_mode = DWC_OTG_HOST_ONLY, /* default */
		.sof_filter = 0, /* default */
		.vbus_detection = 0, /* default */
		.overcurrent_pin = 90, /* default */
		.xfer_debug = 0, /* default */
		.fiq_enable = 0 /* default */
	},
};


/**
 * This function shows the Driver Version.
 */
static ssize_t version_show(struct device_driver *dev, char *buf)
{
	return snprintf(buf, sizeof(DWC_DRIVER_VERSION)+2,"%s\n",
		DWC_DRIVER_VERSION);
}
static DRIVER_ATTR(version, S_IRUGO, version_show, NULL);

/**
 * Global Debug Level Mask.
 */
uint32_t g_dbg_lvl = 0; /* OFF */

/**
 * This function shows the driver Debug Level.
 */
static ssize_t dbg_level_show(struct device_driver *drv, char *buf)
{
	return sprintf(buf, "0x%0x\n", g_dbg_lvl);
}

/**
 * This function stores the driver Debug Level.
 */
static ssize_t dbg_level_store(struct device_driver *drv, const char *buf,
			       size_t count)
{
	g_dbg_lvl = simple_strtoul(buf, NULL, 16);
	return count;
}
static DRIVER_ATTR(debuglevel, S_IRUGO|S_IWUSR, dbg_level_show, dbg_level_store);

/**
 * This function is called to set specific configuration parameters
 */
static void dwc_otg_set_specific_param(dwc_otg_info_t *info, dwc_otg_core_params_t *params, int port)
{
	params->sof_filter = info->sof_filter;
	params->reset_pin = info->reset_pin;
	params->speed = info->speed;
	if (port == 0) {
		params->ctrl_mode = info->ctrl_mode;
		params->vbus_detection = info->vbus_detection;
	}
	if (port == 1 && !usb1_disable_fiq) {
		params->fiq_enable = info->fiq_enable;
	}
}

/**
 * This function is called to enable controller
 */
static int dwc_otg_driver_enable_ctrl(int id)
{
	int ret = 0;
	static const char *clk_name[] = {"usb0", "usb1"};
	struct clk *usb_clk;

	/* reset PHY */
	if (dwc_otg_module_params[id].reset_pin >= 0) {
		DWC_PRINT("enabling phy usb%d with gpio %d\n",
				id, dwc_otg_module_params[id].reset_pin);
#if 0
		/* resting phy produce some strange behaviour :
		   CPEN isn't enabled even if the phy reg seems
		   correct
		 */
		gpio_direction_output(dwc_otg_module_params[id].reset_pin, 0);
		msleep(3);
		gpio_set_value(dwc_otg_module_params[id].reset_pin, 1);
#else
		/* we expect the phy in reset mode : arch init or unload */
		gpio_direction_output(dwc_otg_module_params[id].reset_pin, 1);
#endif
		msleep(3);
	}

	/* enable clock */
	usb_clk = clk_get(NULL, clk_name[id]);
	if (IS_ERR(usb_clk)) {
		ret = -EIO;
		goto out;
	}
	clk_enable(usb_clk);

out:
	return ret;
}


/**
 * This function is called to disable controller
 */
static int dwc_otg_driver_disable_ctrl(int id)
{
	int ret = 0;
	static const char *clk_name[] = {"usb0", "usb1"};
	struct clk *usb_clk;

	/* disable PHY */
	if (dwc_otg_module_params[id].reset_pin >= 0) {
		gpio_set_value(dwc_otg_module_params[id].reset_pin, 0);
	}

	/* disable clock */
	usb_clk = clk_get(NULL, clk_name[id]);
	if (IS_ERR(usb_clk)) {
		ret = -EIO;
		goto out;
	}
	clk_disable(usb_clk);
out:
	return ret;
}


/**
 * This function is the top level interrupt handler for the Common
 * (Device and host modes) interrupts.
 */
static irqreturn_t dwc_otg_common_irq(int irq, void *dev)
{
	dwc_otg_device_t *otg_dev = dev;
	int32_t retval = IRQ_NONE;

	retval = dwc_otg_handle_common_intr(otg_dev->core_if);
	return IRQ_RETVAL(retval);
}


/**
 * This function is called when a platform_device is unregistered with the
 * dwc_otg_driver. This happens, for example, when the rmmod command is
 * executed. The device may or may not be electrically present. If it is
 * present, the driver stops device processing. Any resources used on behalf
 * of this device are freed.
 *
 * @param[in] pdev
 */
static int dwc_otg_driver_remove(struct platform_device *pdev)
{
	int retval;
	dwc_otg_device_t *otg_dev = platform_get_drvdata(pdev);

	DWC_DEBUGPL(DBG_ANY, "%s(%p)\n", __func__, pdev);

	if (otg_dev->core_if->core_params->fiq_enable) {
		/* free fiq */
		disable_fiq(platform_get_irq(pdev, 0));
		release_fiq(&fh);
	}
	/* Free IRQ */
	else if (otg_dev->common_irq_installed) {
		free_irq(platform_get_irq(pdev, 0), otg_dev);
	}


	/* Free HCD */
	if (otg_dev->core_if->core_params->ctrl_mode != DWC_OTG_DEVICE_ONLY) {
		if (otg_dev->hcd) {
			dwc_otg_hcd_remove(pdev);
		}
	}

	/* Free PCD */
	if (otg_dev->core_if->core_params->ctrl_mode != DWC_OTG_HOST_ONLY) {
		if (otg_dev->pcd) {
			dwc_otg_pcd_remove(pdev);
		}
	}

	if (otg_dev->core_if) {
		/* Free CIL */
		dwc_otg_cil_remove(otg_dev->core_if);
	}

	/* Remove the device attributes */
	dwc_otg_attr_remove(pdev);

	/* Disable controller */
	retval = dwc_otg_driver_disable_ctrl(pdev->id);
	if (retval) {
		dev_err(&pdev->dev, "cannot disable controller\n");
	}

	/* Return the memory */
	if (otg_dev->base != NULL) {
		iounmap(otg_dev->base);
		release_resource(otg_dev->mem);
	}
	kfree(otg_dev);

	/* Clear the drvdata pointer */
	platform_set_drvdata(pdev, NULL);

	return 0;
}


/**
 * This function is called when an platform_device is bound to a
 * dwc_otg_driver. It creates the driver components required to
 * control the device (CIL, HCD, and PCD) and it initializes the
 * device. The driver components are stored in a dwc_otg_device
 * structure. A reference to the dwc_otg_device is saved in the
 * platform_device. This allows the driver to access the dwc_otg_device
 * structure on subsequent calls to driver methods for this device.
 *
 * @param[in] pdev  platform_device definition
 */
static int dwc_otg_driver_probe(struct platform_device *pdev)
{
	int retval = 0;
	dwc_otg_device_t *dwc_otg_device;
	struct resource *res;
	int32_t snpsid;
	extern unsigned char dwc_otg_fiq_handler_start, dwc_otg_fiq_handler_end;
	struct pt_regs regs;

	/* Set debug level to diplay diagnostic here */
	//SET_DEBUG_LEVEL(DBG_CIL | DBG_HCD | DBG_PCD);

	dev_dbg(&pdev->dev, "dwc_otg_driver_probe(%p)\n", pdev);

	/* Init the global DWC_otg Device structure */
	dwc_otg_device = kmalloc(sizeof(dwc_otg_device_t), GFP_KERNEL);
	if (dwc_otg_device == 0) {
		dev_err(&pdev->dev, "kmalloc of dwc_otg_device failed\n");
		retval = -ENOMEM;
		goto no_dev;
	}
	memset(dwc_otg_device, 0, sizeof(dwc_otg_device_t));

	dwc_otg_device->reg_offset = 0xFFFFFFFF;
	dwc_otg_device->dev = &pdev->dev;

	/* Map the DWC_otg Core memory into virtual address space */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res == NULL) {
		dev_err(&pdev->dev, "cannot get IORESOURCE_MEM\n");
		retval = -ENOENT;
		goto no_regs;
	}
	dev_dbg(&pdev->dev, "start=0x%08x\n", (unsigned) res->start);

	dwc_otg_device->mem = request_mem_region(res->start,
						 res->end - res->start + 1,
						 pdev->name);
	if (dwc_otg_device->mem == NULL) {
		dev_err(&pdev->dev, "cannot reserve IO region\n");
		retval = -ENOENT;
		goto no_res;
	}

	dwc_otg_device->base = ioremap(res->start, res->end - res->start + 1);
	if (dwc_otg_device->base == NULL) {
		dev_err(&pdev->dev, "cannot map IO\n");
		retval = -EINVAL;
		goto no_map;
	}
	dev_dbg(&pdev->dev, "base=0x%08x\n", (unsigned) dwc_otg_device->base);

	/* Verify platform data for specific configuration */
	if (pdev->dev.platform_data) {
		dev_dbg(&pdev->dev, "specific configuration\n");
		dwc_otg_device->pdata = pdev->dev.platform_data;
		dwc_otg_set_specific_param(dwc_otg_device->pdata,
					   &dwc_otg_module_params[pdev->id],
					   pdev->id);
	}

	/* Enable controller */
	retval = dwc_otg_driver_enable_ctrl(pdev->id);
	if (retval) {
		dev_err(&pdev->dev, "cannot enable controller\n");
		goto no_ctrl;
	}

	/*
	 * Attempt to ensure this device is really a DWC_otg Controller.
	 * Read and verify the SNPSID register contents. The value should be
	 * 0x45F42XXX, which corresponds to "OT2", as in "OTG version 2.XX".
	 */
	snpsid = dwc_read_reg32((uint32_t *) ((uint8_t *) dwc_otg_device->base + 0x40));
	if ((snpsid & 0xFFFFF000) != 0x4F542000) {
		dev_err(&pdev->dev, "Bad value for SNPSID: 0x%08x\n", snpsid);
		retval = -EINVAL;
		goto no_dwc;
	}
	/*
	 * Initialize driver data to point to the global DWC_otg
	 * Device structure.
	 */
	platform_set_drvdata(pdev, dwc_otg_device);
	dev_dbg(&pdev->dev, "dwc_otg_device=0x%p\n", dwc_otg_device);

	dwc_otg_device->core_if = dwc_otg_cil_init(dwc_otg_device->base,
						   &dwc_otg_module_params[pdev->id]);
	if (dwc_otg_device->core_if == 0) {
		dev_err(&pdev->dev, "CIL initialization failed\n");
		retval = -ENOMEM;
		goto err_init_cil;
	}

 	/* Create Device Attributes in sysfs */
	dwc_otg_attr_create(pdev);

	/*
	 * Disable the global interrupt until all the interrupt
	 * handlers are installed.
	 */
	dwc_otg_disable_global_interrupts(dwc_otg_device->core_if);

	/*
	 * Install the interrupt handler for the common interrupts before
	 * enabling common interrupts in core_init below.
	 */
	DWC_DEBUGPL(DBG_CIL, "registering (common) handler for irq%d\n",
		    platform_get_irq(pdev, 0));

	if (dwc_otg_device->core_if->core_params->fiq_enable) {
		/* fiq mode */
		if (claim_fiq(&fh)) {
			DWC_ERROR("couldn't claim fiq\n");
			retval = -EBUSY;
			goto no_irq;
		}

		regs.ARM_sp = 0;
		set_fiq_handler(&dwc_otg_fiq_handler_start,
				(&dwc_otg_fiq_handler_end - &dwc_otg_fiq_handler_start));
		set_fiq_regs(&regs);
		writel(0x40000000, PARROT6_VA_VIC + VIC_INT_SELECT);
		enable_fiq(platform_get_irq(pdev, 0));
	} else {
		retval = request_irq(platform_get_irq(pdev, 0), dwc_otg_common_irq,
				     IRQF_SHARED, pdev->name, dwc_otg_device);
	}

	if (retval != 0) {
		DWC_ERROR("request of irq%d failed\n", platform_get_irq(pdev, 0));
		retval = -EBUSY;
		goto no_irq;
	}
	else {
		dwc_otg_device->common_irq_installed = 1;
	}

	/* Initialize Core */
	dwc_otg_core_init(dwc_otg_device->core_if);

	/* Initialize PCD */
	if (dwc_otg_device->core_if->core_params->ctrl_mode != DWC_OTG_HOST_ONLY) {
		retval = dwc_otg_pcd_init(pdev);
		if (retval != 0) {
			DWC_ERROR("dwc_otg_pcd_init failed\n");
			dwc_otg_device->pcd = NULL;
			goto err_init_pcd;
		}
	}

	/* Initialize HCD */
	if (dwc_otg_device->core_if->core_params->ctrl_mode != DWC_OTG_DEVICE_ONLY) {
		retval = dwc_otg_hcd_init(pdev);
		if (retval != 0) {
			DWC_ERROR("dwc_otg_hcd_init failed\n");
			dwc_otg_device->hcd = NULL;
			goto err_init_hcd;
		}
	}

	/*
	 * Driver data must be re-init to point to the global
	 * DWC_otg Device structure because usb_create_hcd
	 * (called in dwc_otg_hcd_init) modifies it.
	 */
	platform_set_drvdata(pdev, dwc_otg_device);

	/*
	 * Enable the global interrupt after all the interrupt
	 * handlers are installed.
	 */
	dwc_otg_enable_global_interrupts(dwc_otg_device->core_if);

	return 0;

err_init_hcd:
	if (dwc_otg_device->core_if->core_params->ctrl_mode != DWC_OTG_HOST_ONLY) {
		dwc_otg_pcd_remove(pdev);
	}

err_init_pcd:
	if (dwc_otg_device->core_if->core_params->fiq_enable) {
		disable_fiq(platform_get_irq(pdev, 0));
		release_fiq(&fh);
	}
	else
		free_irq(platform_get_irq(pdev, 0), dwc_otg_device);

no_irq :
	dwc_otg_cil_remove(dwc_otg_device->core_if);

err_init_cil:
no_dwc:
no_ctrl:
no_map:
	release_resource(dwc_otg_device->mem);

no_res:
no_regs:
	kfree(dwc_otg_device);

no_dev:
	return retval;
}


/**
 * This structure defines the methods to be called by a bus driver
 * during the lifecycle of a device on that bus. Both drivers and
 * devices are registered with a bus driver. The bus driver matches
 * devices to drivers based on information in the device and driver
 * structures.
 *
 * The probe function is called when the bus driver matches a device
 * to this driver. The remove function is called when a device is
 * unregistered with the bus driver.
 */
static struct platform_driver dwc_otg_driver = {
	.probe          = dwc_otg_driver_probe,
	.remove		= dwc_otg_driver_remove,
	.driver         = {
		.name	= (char*)dwc_driver_name,
	},
};


/**
 * This function is called when the dwc_otg_driver is installed with the
 * insmod command. It registers the dwc_otg_driver structure with the
 * appropriate bus driver. This will cause the dwc_otg_driver_probe function
 * to be called. In addition, the bus driver will automatically expose
 * attributes defined for the device and driver in the special sysfs file
 * system.
 *
 * @return
 */
static int __init dwc_otg_driver_init(void)
{
	int retval = 0;

	printk(KERN_INFO "%s: version %s\n", dwc_driver_name, DWC_DRIVER_VERSION);

	retval = platform_driver_register(&dwc_otg_driver);
	if (retval < 0) {
		printk(KERN_ERR "%s : driver register failed (%d)\n", __func__, retval);
		goto err_driver_register;
	}

	retval = driver_create_file(&dwc_otg_driver.driver, &driver_attr_version);
	if (retval < 0) {
		printk(KERN_ERR "%s : create version file (sysfs) failed (%d)\n", __func__, retval);
		goto err_attr_version;
	}

	retval = driver_create_file(&dwc_otg_driver.driver, &driver_attr_debuglevel);
	if (retval < 0) {
		printk(KERN_ERR "%s : create debug level file (sysfs) failed (%d)\n", __func__, retval);
		goto err_attr_debug_level;
	}

	return 0;

err_attr_debug_level:
	driver_remove_file(&dwc_otg_driver.driver, &driver_attr_version);
err_attr_version:
	platform_driver_unregister(&dwc_otg_driver);
err_driver_register:

	return retval;
}
subsys_initcall(dwc_otg_driver_init);


/**
 * This function is called when the driver is removed from the kernel
 * with the rmmod command. The driver unregisters itself with its bus
 * driver.
 *
 */
static void __exit dwc_otg_driver_cleanup(void)
{
	printk(KERN_DEBUG "dwc_otg_driver_cleanup()\n");

	driver_remove_file(&dwc_otg_driver.driver, &driver_attr_debuglevel);
	driver_remove_file(&dwc_otg_driver.driver, &driver_attr_version);

	platform_driver_unregister(&dwc_otg_driver);

	printk(KERN_INFO "%s module removed\n", dwc_driver_name);
}
module_exit(dwc_otg_driver_cleanup);


MODULE_DESCRIPTION(DWC_DRIVER_DESC);
MODULE_AUTHOR("Synopsys Inc.");
MODULE_LICENSE("GPL");
