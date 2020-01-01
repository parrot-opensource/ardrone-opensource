/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_cil.c $
 * $Revision: #45 $
 * $Date: 2008/04/29 $
 * $Change: 1024967 $
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
 *
 * The Core Interface Layer provides basic services for accessing and
 * managing the DWC_otg hardware. These services are used by both the
 * Host Controller Driver and the Peripheral Controller Driver.
 *
 * The CIL manages the memory map for the core so that the HCD and PCD
 * don't have to do this separately. It also handles basic tasks like
 * reading/writing the registers and data FIFOs in the controller.
 * Some of the data access functions provide encapsulation of several
 * operations required to perform a task, such as writing multiple
 * registers to start a transfer. Finally, the CIL performs basic
 * services that are not specific to either the host or device modes
 * of operation. These services include management of the OTG Host
 * Negotiation Protocol (HNP) and Session Request Protocol (SRP). A
 * Diagnostic API is also provided to allow testing of the controller
 * hardware.
 *
 * The Core Interface Layer has the following requirements:
 * - Provides basic controller operations.
 * - Minimal use of OS services.
 * - The OS services used will be abstracted by using inline functions
 *	 or macros.
 *
 */
#include <asm/unaligned.h>
#ifdef DEBUG
#include <linux/jiffies.h>
#endif

#include "linux/dwc_otg_plat.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_cil.h"

/**
 * This function is called to initialize the DWC_otg CSR data
 * structures.	The register addresses in the device and host
 * structures are initialized from the base address supplied by the
 * caller.	The calling function must make the OS calls to get the
 * base address of the DWC_otg controller registers.  The core_params
 * argument holds the parameters that specify how the core should be
 * configured.
 *
 * @param[in] reg_base_addr Base address of DWC_otg core registers
 * @param[in] core_params Pointer to the core configuration parameters
 *
 */
dwc_otg_core_if_t *dwc_otg_cil_init(const uint32_t *reg_base_addr,
					dwc_otg_core_params_t *core_params)
{
	dwc_otg_core_if_t *core_if = 0;
	dwc_otg_dev_if_t *dev_if = 0;
	dwc_otg_host_if_t *host_if = 0;
	uint8_t *reg_base = (uint8_t *)reg_base_addr;
	int i = 0;

	DWC_DEBUGPL(DBG_CILV, "%s(%p,%p)\n", __func__, reg_base_addr, core_params);

	core_if = kmalloc(sizeof(dwc_otg_core_if_t), GFP_KERNEL);

	if (core_if == 0) {
		DWC_DEBUGPL(DBG_CIL, "Allocation of dwc_otg_core_if_t failed\n");
		return 0;
	}

	memset(core_if, 0, sizeof(dwc_otg_core_if_t));

	core_if->core_params = core_params;
	core_if->core_global_regs = (dwc_otg_core_global_regs_t *)reg_base;

	/*
	 * Allocate the Device Mode structures.
	 */
	dev_if = kmalloc(sizeof(dwc_otg_dev_if_t), GFP_KERNEL);

	if (dev_if == 0) {
		DWC_DEBUGPL(DBG_CIL, "Allocation of dwc_otg_dev_if_t failed\n");
		kfree(core_if);
		return 0;
	}

	dev_if->dev_global_regs =
			(dwc_otg_device_global_regs_t *)(reg_base + DWC_DEV_GLOBAL_REG_OFFSET);

	for (i=0; i<MAX_EPS_CHANNELS; i++)
	{
		dev_if->in_ep_regs[i] = (dwc_otg_dev_in_ep_regs_t *)
				(reg_base + DWC_DEV_IN_EP_REG_OFFSET +
				 (i * DWC_EP_REG_OFFSET));

		dev_if->out_ep_regs[i] = (dwc_otg_dev_out_ep_regs_t *)
				(reg_base + DWC_DEV_OUT_EP_REG_OFFSET +
				 (i * DWC_EP_REG_OFFSET));
		DWC_DEBUGPL(DBG_CILV, "in_ep_regs[%d]->diepctl=%p\n",
					i, &dev_if->in_ep_regs[i]->diepctl);
		DWC_DEBUGPL(DBG_CILV, "out_ep_regs[%d]->doepctl=%p\n",
					i, &dev_if->out_ep_regs[i]->doepctl);
	}

	dev_if->speed = 0; // unknown

	core_if->dev_if = dev_if;

	/*
	 * Allocate the Host Mode structures.
	 */
	host_if = kmalloc(sizeof(dwc_otg_host_if_t), GFP_KERNEL);

	if (host_if == 0) {
		DWC_DEBUGPL(DBG_CIL, "Allocation of dwc_otg_host_if_t failed\n");
		kfree(dev_if);
		kfree(core_if);
		return 0;
	}

	host_if->host_global_regs = (dwc_otg_host_global_regs_t *)
			(reg_base + DWC_OTG_HOST_GLOBAL_REG_OFFSET);

	host_if->hprt0 = (uint32_t*)(reg_base + DWC_OTG_HOST_PORT_REGS_OFFSET);

	for (i=0; i<MAX_EPS_CHANNELS; i++)
	{
		host_if->hc_regs[i] = (dwc_otg_hc_regs_t *)
				(reg_base + DWC_OTG_HOST_CHAN_REGS_OFFSET +
				 (i * DWC_OTG_CHAN_REGS_OFFSET));
		DWC_DEBUGPL(DBG_CILV, "hc_reg[%d]->hcchar=%p\n",
					i, &host_if->hc_regs[i]->hcchar);
	}

	host_if->num_host_channels = MAX_EPS_CHANNELS;
	core_if->host_if = host_if;

	for (i=0; i<MAX_EPS_CHANNELS; i++)
	{
		core_if->data_fifo[i] =
				(uint32_t *)(reg_base + DWC_OTG_DATA_FIFO_OFFSET +
							 (i * DWC_OTG_DATA_FIFO_SIZE));
		DWC_DEBUGPL(DBG_CILV, "data_fifo[%d]=0x%08x\n",
					i, (unsigned)core_if->data_fifo[i]);
	}

	core_if->pcgcctl = (uint32_t*)(reg_base + DWC_OTG_PCGCCTL_OFFSET);

	/*
	 * Store the contents of the hardware configuration registers here for
	 * easy access later.
	 */
	core_if->hwcfg1.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg1);
	core_if->hwcfg2.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg2);
	core_if->hwcfg3.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg3);
	core_if->hwcfg4.d32 = dwc_read_reg32(&core_if->core_global_regs->ghwcfg4);

	DWC_DEBUGPL(DBG_CILV,"hwcfg1=%08x\n",core_if->hwcfg1.d32);
	DWC_DEBUGPL(DBG_CILV,"hwcfg2=%08x\n",core_if->hwcfg2.d32);
	DWC_DEBUGPL(DBG_CILV,"hwcfg3=%08x\n",core_if->hwcfg3.d32);
	DWC_DEBUGPL(DBG_CILV,"hwcfg4=%08x\n",core_if->hwcfg4.d32);

	core_if->hcfg.d32 = dwc_read_reg32(&core_if->host_if->host_global_regs->hcfg);
	core_if->dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);

	DWC_DEBUGPL(DBG_CILV,"hcfg=%08x\n",core_if->hcfg.d32);
	DWC_DEBUGPL(DBG_CILV,"dcfg=%08x\n",core_if->dcfg.d32);

	DWC_DEBUGPL(DBG_CILV,"op_mode=%0x\n",core_if->hwcfg2.b.op_mode);
	DWC_DEBUGPL(DBG_CILV,"arch=%0x\n",core_if->hwcfg2.b.architecture);
	DWC_DEBUGPL(DBG_CILV,"num_dev_ep=%d\n",core_if->hwcfg2.b.num_dev_ep);
	DWC_DEBUGPL(DBG_CILV,"num_host_chan=%d\n",core_if->hwcfg2.b.num_host_chan);
	DWC_DEBUGPL(DBG_CILV,"nonperio_tx_q_depth=0x%0x\n",core_if->hwcfg2.b.nonperio_tx_q_depth);
	DWC_DEBUGPL(DBG_CILV,"host_perio_tx_q_depth=0x%0x\n",core_if->hwcfg2.b.host_perio_tx_q_depth);
	DWC_DEBUGPL(DBG_CILV,"dev_token_q_depth=0x%0x\n",core_if->hwcfg2.b.dev_token_q_depth);

	DWC_DEBUGPL(DBG_CILV,"Total FIFO SZ=%d\n", core_if->hwcfg3.b.dfifo_depth);
	DWC_DEBUGPL(DBG_CILV,"xfer_size_cntr_width=0x%0x\n", core_if->hwcfg3.b.xfer_size_cntr_width);

	return core_if;
}

/**
 * This function frees the structures allocated by dwc_otg_cil_init().
 *
 * @param[in] core_if The core interface pointer returned from
 * dwc_otg_cil_init().
 *
 */
void dwc_otg_cil_remove(dwc_otg_core_if_t *core_if)
{
	/* Disable all interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, 1, 0);
	dwc_write_reg32(&core_if->core_global_regs->gintmsk, 0);

	if (core_if->dev_if) {
		kfree(core_if->dev_if);
	}
	if (core_if->host_if) {
		kfree(core_if->host_if);
	}
	kfree(core_if);
}

/**
 * This function enables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param[in] core_if Programming view of DWC_otg controller.
 */
void dwc_otg_enable_global_interrupts(dwc_otg_core_if_t *core_if)
{
	gahbcfg_data_t ahbcfg = { .d32 = 0};
	ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, 0, ahbcfg.d32);
}

/**
 * This function disables the controller's Global Interrupt in the AHB Config
 * register.
 *
 * @param[in] core_if Programming view of DWC_otg controller.
 */
void dwc_otg_disable_global_interrupts(dwc_otg_core_if_t *core_if)
{
	gahbcfg_data_t ahbcfg = { .d32 = 0};
	ahbcfg.b.glblintrmsk = 1; /* Enable interrupts */
	dwc_modify_reg32(&core_if->core_global_regs->gahbcfg, ahbcfg.d32, 0);
}

/**
 * This function initializes the commmon interrupts, used in both
 * device and host modes.
 *
 * @param[in] core_if Programming view of the DWC_otg controller
 *
 */
static void dwc_otg_enable_common_interrupts(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_core_params_t *params = core_if->core_params;
	gintmsk_data_t intr_mask = { .d32 = 0};

	/* Clear any pending OTG Interrupts */
	dwc_write_reg32(&global_regs->gotgint, 0xFFFFFFFF);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	if (!params->fiq_enable) {
		/*
		 * Enable the interrupts in the GINTMSK.
		 */
		intr_mask.b.modemismatch = 0;
		intr_mask.b.otgintr = 0;
		if (!core_if->dma_enable) {
			intr_mask.b.rxstsqlvl = 1;
		}
		intr_mask.b.conidstschng = 0;
		intr_mask.b.wkupintr = 1;
		intr_mask.b.disconnect = 1;
		intr_mask.b.usbsuspend = 1;
		intr_mask.b.sessreqintr = 0;

		dwc_write_reg32(&global_regs->gintmsk, intr_mask.d32);
	}
}

/**
 * Initializes the FSLSPClkSel field of the HCFG register depending on the PHY
 * type.
 */
static void init_fslspclksel(dwc_otg_core_if_t *core_if)
{
	uint32_t	val;
	hcfg_data_t		hcfg;

	if (((core_if->hwcfg2.b.hs_phy_type == 2) &&
		 (core_if->hwcfg2.b.fs_phy_type == 1) &&
		 (core_if->core_params->ulpi_fs_ls)) ||
		(core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* Full speed PHY */
		val = DWC_HCFG_48_MHZ;
	}
	else {
		/* High speed PHY running at full speed or high speed */
		val = DWC_HCFG_30_60_MHZ;
	}

	DWC_DEBUGPL(DBG_CIL, "Initializing HCFG.FSLSPClkSel to 0x%1x\n", val);
	hcfg.d32 = dwc_read_reg32(&core_if->host_if->host_global_regs->hcfg);
	hcfg.b.fslspclksel = val;
	dwc_write_reg32(&core_if->host_if->host_global_regs->hcfg, hcfg.d32);
}

/**
 * Initializes the DevSpd field of the DCFG register depending on the PHY type
 * and the enumeration speed of the device.
 */
static void init_devspd(dwc_otg_core_if_t *core_if)
{
	uint32_t	val;
	dcfg_data_t		dcfg;

	if (((core_if->hwcfg2.b.hs_phy_type == 2) &&
		 (core_if->hwcfg2.b.fs_phy_type == 1) &&
		 (core_if->core_params->ulpi_fs_ls)) ||
		(core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* Full speed PHY */
		val = 0x3;
	}
	else if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL) {
		/* High speed PHY running at full speed */
		val = 0x1;
	}
	else {
		/* High speed PHY running at high speed */
		val = 0x0;
	}

	DWC_DEBUGPL(DBG_CIL, "Initializing DCFG.DevSpd to 0x%1x\n", val);

	dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);
	dcfg.b.devspd = val;
	dwc_write_reg32(&core_if->dev_if->dev_global_regs->dcfg, dcfg.d32);
}

/**
 * This function calculates the number of IN EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_in_eps(dwc_otg_core_if_t *core_if)
{
	uint32_t num_in_eps = 0;
	uint32_t num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hwcfg1 = core_if->hwcfg1.d32 >> 3;
	int i;


	for(i = 0; i < num_eps; ++i)
	{
		if(!(hwcfg1 & 0x1))
			num_in_eps++;

		hwcfg1 >>= 2;
	}

	return num_in_eps;
}


/**
 * This function calculates the number of OUT EPS
 * using GHWCFG1 and GHWCFG2 registers values
 *
 * @param core_if Programming view of the DWC_otg controller
 */
static uint32_t calc_num_out_eps(dwc_otg_core_if_t *core_if)
{
	uint32_t num_out_eps = 0;
	uint32_t num_eps = core_if->hwcfg2.b.num_dev_ep;
	uint32_t hwcfg1 = core_if->hwcfg1.d32 >> 2;
	int i;

	for(i = 0; i < num_eps; ++i)
	{
		if(!(hwcfg1 & 0x1))
			num_out_eps++;

		hwcfg1 >>= 2;
	}
	return num_out_eps;
}


/**
 * This function is called to detect VBUS
 */
#define MAX_PHY_ACCESS 10
int dwc_otg_core_vbus_detect(dwc_otg_core_if_t *core_if)
{
#if 0
	/* sesion valid is okay when vbus < 1.4
	   this is too long to wait for that time
	 */
	gotgctl_data_t gotgctl;

	gotgctl.d32 = dwc_read_reg32(&core_if->core_global_regs->gotgctl);
	//printk("gotgctl %x=%x\n", &core_if->core_global_regs->gotgctl,
	//		gotgctl.d32);

	return gotgctl.b.bsesvld;
#else
	/* vbus valid when vbus < 4.57 */
	int ret = 0, attempt = 0;
	gpvndctl_data_t pvndctl = {.d32 = 0};

	/* new request */
	pvndctl.b.regaddr = DWC_GPVNDCTL_USB_INT_STATUS_REG;
	pvndctl.b.newregreq = 1;
	dwc_write_reg32(&core_if->core_global_regs->gpvndctl, pvndctl.d32);

	/* wait for Vstatus done */
	do {
		pvndctl.d32 = dwc_read_reg32(&core_if->core_global_regs->gpvndctl);
		attempt++;
	} while (!pvndctl.b.vstsdone && (attempt < MAX_PHY_ACCESS));

	if (!pvndctl.b.vstsdone) {
		DWC_ERROR("phy hang ....\n");
	}

	/* read register */
	if (pvndctl.b.regdata & DWC_GPVNDCTL_VBUS_VALID_MSK)
		ret = 1;

	return ret;
#endif
}

/**
 * This function initializes the DWC_otg controller registers and
 * prepares the core for device mode or host mode operation.
 *
 * @param core_if Programming view of the DWC_otg controller
 *
 */
void dwc_otg_core_init(dwc_otg_core_if_t *core_if)
{
	int i = 0;
	dwc_otg_core_global_regs_t *global_regs =
			core_if->core_global_regs;
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	gahbcfg_data_t ahbcfg = { .d32 = 0 };
	gusbcfg_data_t usbcfg = { .d32 = 0 };
	gi2cctl_data_t i2cctl = { .d32 = 0 };

	DWC_DEBUGPL(DBG_CILV, "dwc_otg_core_init(%p)\n", core_if);

	/* Reset the Controller */
	dwc_otg_core_reset(core_if);

	/* VBUS detection */
	if ((core_if->core_params->ctrl_mode != DWC_OTG_DEVICE_ONLY) &&
	    (core_if->core_params->vbus_detection) && dwc_otg_core_vbus_detect(core_if)) {
		core_if->core_params->ctrl_mode = DWC_OTG_DEVICE_ONLY;
		DWC_PRINT("VBUS detected - new control mode is DEVICE only\n");
	}

	/* Common Initialization */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);

	/* Program the ULPI External VBUS bit if needed */
	usbcfg.b.ulpi_ext_vbus_drv =
		(core_if->core_params->phy_ulpi_ext_vbus == DWC_PHY_ULPI_EXTERNAL_VBUS) ? 1 : 0;

	/* Program mode */
	if (core_if->core_params->ctrl_mode == DWC_OTG_DEVICE_ONLY) {
		usbcfg.b.hstmode = 0;
		usbcfg.b.devmode = 1;
	} else if (core_if->core_params->ctrl_mode == DWC_OTG_HOST_ONLY) {
		usbcfg.b.hstmode = 1;
		usbcfg.b.devmode = 0;
	}

	/* Set external TS Dline pulsing */
	usbcfg.b.term_sel_dl_pulse = (core_if->core_params->ts_dline == 1) ? 1 : 0;
	dwc_write_reg32 (&global_regs->gusbcfg, usbcfg.d32);

	/* Reset the Controller */
	//dwc_otg_core_reset(core_if);

	/* Initialize parameters from Hardware configuration registers. */
	dev_if->num_in_eps = calc_num_in_eps(core_if);
	dev_if->num_out_eps = calc_num_out_eps(core_if);

	DWC_DEBUGPL(DBG_CIL, "num_dev_perio_in_ep=%d\n", core_if->hwcfg4.b.num_dev_perio_in_ep);

	for (i=0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++)
	{
		dev_if->perio_tx_fifo_size[i] =
			dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]) >> 16;
		DWC_DEBUGPL(DBG_CIL, "Periodic Tx FIFO SZ #%d=0x%0x\n",
				i, dev_if->perio_tx_fifo_size[i]);
	}

	for (i=0; i < core_if->hwcfg4.b.num_in_eps; i++)
	{
		dev_if->tx_fifo_size[i] =
			dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]) >> 16;
		DWC_DEBUGPL(DBG_CIL, "Tx FIFO SZ #%d=0x%0x\n",
			i, dev_if->perio_tx_fifo_size[i]);
	}

	core_if->total_fifo_size = core_if->hwcfg3.b.dfifo_depth;
	core_if->rx_fifo_size =
			dwc_read_reg32(&global_regs->grxfsiz);
	core_if->nperio_tx_fifo_size =
			dwc_read_reg32(&global_regs->gnptxfsiz) >> 16;

	DWC_DEBUGPL(DBG_CIL, "Total FIFO SZ=%d\n", core_if->total_fifo_size);
	DWC_DEBUGPL(DBG_CIL, "Rx FIFO SZ=%d\n", core_if->rx_fifo_size);
	DWC_DEBUGPL(DBG_CIL, "NP Tx FIFO SZ=%d\n", core_if->nperio_tx_fifo_size);

	/* This programming sequence needs to happen in FS mode before any other
	 * programming occurs */
	if ((core_if->core_params->speed == DWC_SPEED_PARAM_FULL) &&
	    (core_if->core_params->phy_type == DWC_PHY_TYPE_PARAM_FS)) {
		/* If FS mode with FS PHY */

		/* core_init() is now called on every switch so only call the
		 * following for the first time through. */
		if (!core_if->phy_init_done) {
			core_if->phy_init_done = 1;
			DWC_DEBUGPL(DBG_CIL, "FS_PHY detected\n");
			usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
			usbcfg.b.physel = 1;
			dwc_write_reg32 (&global_regs->gusbcfg, usbcfg.d32);

			/* Reset after a PHY select */
			dwc_otg_core_reset(core_if);
		}

		/* Program DCFG.DevSpd or HCFG.FSLSPclkSel to 48Mhz in FS.	Also
		 * do this on HNP Dev/Host mode switches (done in dev_init and
		 * host_init). */
		if (dwc_otg_is_host_mode(core_if)) {
			init_fslspclksel(core_if);
		}
		else {
			init_devspd(core_if);
		}

		if (core_if->core_params->i2c_enable) {
			DWC_DEBUGPL(DBG_CIL, "FS_PHY Enabling I2c\n");
			/* Program GUSBCFG.OtgUtmifsSel to I2C */
			usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
			usbcfg.b.otgutmifssel = 1;
			dwc_write_reg32 (&global_regs->gusbcfg, usbcfg.d32);

			/* Program GI2CCTL.I2CEn */
			i2cctl.d32 = dwc_read_reg32(&global_regs->gi2cctl);
			i2cctl.b.i2cdevaddr = 1;
			i2cctl.b.i2cen = 0;
			dwc_write_reg32 (&global_regs->gi2cctl, i2cctl.d32);
			i2cctl.b.i2cen = 1;
			dwc_write_reg32 (&global_regs->gi2cctl, i2cctl.d32);
		}

	} /* endif speed == DWC_SPEED_PARAM_FULL */
	else {
		/* High speed PHY. */
		if (!core_if->phy_init_done) {
			/* HS PHY parameters.  These parameters are preserved
			 * during soft reset so only program the first time.  Do
			 * a soft reset immediately after setting phyif.  */
			if (core_if->core_params->phy_type == 1) {
				/* UTMI+ interface */
				usbcfg.b.ulpi_utmi_sel = 0;
				if (core_if->core_params->phy_utmi_width == 16) {
					usbcfg.b.phyif = 1;
				}
				else {
					usbcfg.b.phyif = 0;
				}
			}
			else {
				/* ULPI interface */
				usbcfg.b.ulpi_utmi_sel = 1;
				usbcfg.b.phyif = 0;
				usbcfg.b.ddrsel = core_if->core_params->phy_ulpi_ddr;
			}
			core_if->phy_init_done = 1;
		}

		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

		/* Reset after setting the PHY parameters */
		dwc_otg_core_reset(core_if);
	}

	if ((core_if->hwcfg2.b.hs_phy_type == 2) &&
		(core_if->hwcfg2.b.fs_phy_type == 1) &&
		(core_if->core_params->ulpi_fs_ls)) {
		DWC_DEBUGPL(DBG_CIL, "Setting ULPI FSLS\n");
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 1;
		usbcfg.b.ulpi_clk_sus_m = 1;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	}
	else {
		usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);
		usbcfg.b.ulpi_fsls = 0;
		usbcfg.b.ulpi_clk_sus_m = 0;
		dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);
	}

	/* Program the GAHBCFG Register.*/
	switch (core_if->hwcfg2.b.architecture) {

	case DWC_SLAVE_ONLY_ARCH:
		DWC_DEBUGPL(DBG_CIL, "Slave Only Mode\n");
		ahbcfg.b.nptxfemplvl_txfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		ahbcfg.b.ptxfemplvl = DWC_GAHBCFG_TXFEMPTYLVL_HALFEMPTY;
		core_if->dma_enable = 0;
		core_if->dma_desc_enable = 0;
		break;

	case DWC_EXT_DMA_ARCH:
		DWC_DEBUGPL(DBG_CIL, "External DMA Mode\n");
		ahbcfg.b.hburstlen = core_if->core_params->dma_burst_size;
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		core_if->dma_desc_enable = (core_if->core_params->dma_desc_enable != 0);
		break;

	case DWC_INT_DMA_ARCH:
		DWC_DEBUGPL(DBG_CIL, "Internal DMA Mode\n");
		ahbcfg.b.hburstlen = DWC_GAHBCFG_INT_DMA_BURST_INCR;
		core_if->dma_enable = (core_if->core_params->dma_enable != 0);
		core_if->dma_desc_enable = (core_if->core_params->dma_desc_enable != 0);
		break;

	}
	ahbcfg.b.dmaenable = core_if->dma_enable;
	dwc_write_reg32(&global_regs->gahbcfg, ahbcfg.d32);

	core_if->en_multiple_tx_fifo = core_if->hwcfg4.b.ded_fifo_en;


	/*
	 * Program the GUSBCFG register.
	 */
	usbcfg.d32 = dwc_read_reg32(&global_regs->gusbcfg);

	switch (core_if->hwcfg2.b.op_mode) {
	case DWC_MODE_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = (core_if->core_params->otg_cap ==
		   DWC_OTG_CAP_PARAM_HNP_SRP_CAPABLE);
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
		   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_SRP_ONLY_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
		   DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_HNP_SRP_CAPABLE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;

	case DWC_MODE_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
		DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_SRP_CAPABLE_DEVICE:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;

	case DWC_MODE_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = (core_if->core_params->otg_cap !=
		DWC_OTG_CAP_PARAM_NO_HNP_SRP_CAPABLE);
		break;

	case DWC_MODE_NO_SRP_CAPABLE_HOST:
		usbcfg.b.hnpcap = 0;
		usbcfg.b.srpcap = 0;
		break;
	}

	dwc_write_reg32(&global_regs->gusbcfg, usbcfg.d32);

	/* SOF filtering */
	if (core_if->core_params->sof_filter > 0) {
		core_if->core_global_regs->ggpio = (DWC_GPPIO_ENABLE_SOF_FILTERING |
						    core_if->core_params->sof_filter << 16);
	} else {
		core_if->core_global_regs->ggpio = 0;
	}

	/* Enable common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

 	/* Do device or host intialization based on mode during PCD
	 * and HCD initialization  */
	if (dwc_otg_is_host_mode(core_if)) {
		DWC_DEBUGPL(DBG_ANY, "Host Mode\n");
	}
	else {
		DWC_DEBUGPL(DBG_ANY, "Device Mode\n");
	}
}


/**
 * This function enables the Device mode interrupts.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void dwc_otg_enable_device_interrupts(dwc_otg_core_if_t *core_if)
{
	gintmsk_data_t intr_mask = { .d32 = 0};
	dwc_otg_core_global_regs_t *global_regs =
		core_if->core_global_regs;

	DWC_DEBUGPL(DBG_CIL, "%s()\n", __func__);

	/* Disable all interrupts. */
	dwc_write_reg32(&global_regs->gintmsk, 0);

	/* Clear any pending interrupts */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/* Enable the common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/* Enable interrupts */
	intr_mask.b.usbreset = 1;
	intr_mask.b.enumdone = 1;
	intr_mask.b.inepintr = 1;
	intr_mask.b.outepintr = 1;
	intr_mask.b.erlysuspend = 1;

	if(core_if->en_multiple_tx_fifo == 0) {
		intr_mask.b.epmismatch = 1;
	}

	/** @todo NGS: Should this be a module parameter? */
#ifdef USE_PERIODIC_EP
	intr_mask.b.isooutdrop = 1;
	intr_mask.b.eopframe = 1;
	intr_mask.b.incomplisoin = 1;
	intr_mask.b.incomplisoout = 1;
#endif

	dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, intr_mask.d32);

	DWC_DEBUGPL(DBG_CIL, "%s() gintmsk=%0x\n", __func__,
		dwc_read_reg32(&global_regs->gintmsk));
}

/**
 * This function initializes the DWC_otg controller registers for
 * device mode.
 *
 * @param core_if Programming view of DWC_otg controller
 *
 */
void dwc_otg_core_dev_init(dwc_otg_core_if_t *core_if)
{
	int i;
	dwc_otg_core_global_regs_t *global_regs =
		core_if->core_global_regs;
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dwc_otg_core_params_t *params = core_if->core_params;
	dcfg_data_t dcfg = {.d32 = 0};
	dctl_data_t dctl = {.d32 = 0};
	grstctl_t resetctl = {.d32 = 0};
	uint32_t rx_fifo_size;
	fifosize_data_t nptxfifosize;
	fifosize_data_t txfifosize;
	dthrctl_data_t dthrctl;
	fifosize_data_t ptxfifosize;

	printk("dwc_otg_core_dev_init() started\n" );

	/* Enable Soft Disconnect */
	dctl.b.sftdiscon = 1;
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dctl, 0, dctl.d32);

	/* Restart the Phy Clock */
	dwc_write_reg32(core_if->pcgcctl, 0);

	/* Device configuration register */
	init_devspd(core_if);
	dcfg.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dcfg);
	dcfg.b.descdma = (core_if->dma_desc_enable) ? 1 : 0;
	dcfg.b.perfrint = DWC_DCFG_FRAME_INTERVAL_80;

	dwc_write_reg32(&dev_if->dev_global_regs->dcfg, dcfg.d32);

	/* Configure data FIFO sizes */
	if (core_if->hwcfg2.b.dynamic_fifo && params->enable_dynamic_fifo) {
		DWC_DEBUGPL(DBG_CIL, "Total FIFO Size=%d\n", core_if->total_fifo_size);
		DWC_DEBUGPL(DBG_CIL, "Rx FIFO Size=%d\n", params->dev_rx_fifo_size);
		DWC_DEBUGPL(DBG_CIL, "NP Tx FIFO Size=%d\n", params->dev_nperio_tx_fifo_size);

		/* Rx FIFO */
		DWC_DEBUGPL(DBG_CIL, "initial grxfsiz=%08x\n",
						dwc_read_reg32(&global_regs->grxfsiz));

		rx_fifo_size = params->dev_rx_fifo_size;
		dwc_write_reg32(&global_regs->grxfsiz, rx_fifo_size);

		DWC_DEBUGPL(DBG_CIL, "new grxfsiz=%08x\n",
			dwc_read_reg32(&global_regs->grxfsiz));

		/** Set Periodic Tx FIFO Mask all bits 0 */
		core_if->p_tx_msk = 0;

	 	/** Set Tx FIFO Mask all bits 0 */
		core_if->tx_msk = 0;

 		if (core_if->en_multiple_tx_fifo == 0) {
			/* Non-periodic Tx FIFO */
			DWC_DEBUGPL(DBG_CIL, "initial gnptxfsiz=%08x\n",
						   dwc_read_reg32(&global_regs->gnptxfsiz));

			nptxfifosize.b.depth  = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			dwc_write_reg32(&global_regs->gnptxfsiz, nptxfifosize.d32);

			DWC_DEBUGPL(DBG_CIL, "new gnptxfsiz=%08x\n",
						   dwc_read_reg32(&global_regs->gnptxfsiz));

			/**@todo NGS: Fix Periodic FIFO Sizing! */
			/*
			 * Periodic Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_perio_tx_fifo_size array and the FIFO size registers in
			 * the dptxfsiz array run from 0 to 14.
			 */
			/** @todo Finish debug of this */
			ptxfifosize.b.startaddr = nptxfifosize.b.startaddr + nptxfifosize.b.depth;
			for (i=0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; i++)
			{
				ptxfifosize.b.depth = params->dev_perio_tx_fifo_size[i];
				DWC_DEBUGPL(DBG_CIL, "initial dptxfsiz_dieptxf[%d]=%08x\n", i,
							dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]));
				dwc_write_reg32(&global_regs->dptxfsiz_dieptxf[i],
								 ptxfifosize.d32);
				DWC_DEBUGPL(DBG_CIL, "new dptxfsiz_dieptxf[%d]=%08x\n", i,
							dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]));
				ptxfifosize.b.startaddr += ptxfifosize.b.depth;
			}
		}
		else {
			/*
			 * Tx FIFOs These FIFOs are numbered from 1 to 15.
			 * Indexes of the FIFO size module parameters in the
			 * dev_tx_fifo_size array and the FIFO size registers in
			 * the dptxfsiz_dieptxf array run from 0 to 14.
			 */


			/* Non-periodic Tx FIFO */
			DWC_DEBUGPL(DBG_CIL, "initial gnptxfsiz=%08x\n",
							dwc_read_reg32(&global_regs->gnptxfsiz));

			nptxfifosize.b.depth  = params->dev_nperio_tx_fifo_size;
			nptxfifosize.b.startaddr = params->dev_rx_fifo_size;

			dwc_write_reg32(&global_regs->gnptxfsiz, nptxfifosize.d32);

			DWC_DEBUGPL(DBG_CIL, "new gnptxfsiz=%08x\n",
							dwc_read_reg32(&global_regs->gnptxfsiz));

			txfifosize.b.startaddr = nptxfifosize.b.startaddr + nptxfifosize.b.depth;

			for (i=1; i < core_if->hwcfg4.b.num_in_eps; i++)
			{

				txfifosize.b.depth = params->dev_tx_fifo_size[i];

				DWC_DEBUGPL(DBG_CIL, "initial dptxfsiz_dieptxf[%d]=%08x\n", i,
					dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i]));

				dwc_write_reg32(&global_regs->dptxfsiz_dieptxf[i-1],
					txfifosize.d32);

				DWC_DEBUGPL(DBG_CIL, "new dptxfsiz_dieptxf[%d]=%08x\n", i,
					dwc_read_reg32(&global_regs->dptxfsiz_dieptxf[i-1]));

				txfifosize.b.startaddr += txfifosize.b.depth;
			}
		}
	}

	/* Flush the FIFOs */
	dwc_otg_flush_tx_fifo(core_if, 0x10); /* all Tx FIFOs */
	dwc_otg_flush_rx_fifo(core_if);

	/* Flush the Learning Queue. */
	resetctl.b.intknqflsh = 1;
	dwc_write_reg32(&core_if->core_global_regs->grstctl, resetctl.d32);

	/* Clear all pending Device Interrupts */
	dwc_write_reg32(&dev_if->dev_global_regs->diepmsk, 0);
	dwc_write_reg32(&dev_if->dev_global_regs->doepmsk, 0);
	dwc_write_reg32(&dev_if->dev_global_regs->daint, 0xFFFFFFFF);
	dwc_write_reg32(&dev_if->dev_global_regs->daintmsk, 0);

	for (i=0; i <= dev_if->num_in_eps; i++)
	{
		depctl_data_t depctl;
		depctl.d32 = dwc_read_reg32(&dev_if->in_ep_regs[i]->diepctl);
		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		}
		else {
			depctl.d32 = 0;
		}

		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepctl, depctl.d32);


		dwc_write_reg32(&dev_if->in_ep_regs[i]->dieptsiz, 0);
		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepdma, 0);
		dwc_write_reg32(&dev_if->in_ep_regs[i]->diepint, 0xFF);
	}

	for (i=0; i <= dev_if->num_out_eps; i++)
	{
		depctl_data_t depctl;
		depctl.d32 = dwc_read_reg32(&dev_if->out_ep_regs[i]->doepctl);
		if (depctl.b.epena) {
			depctl.d32 = 0;
			depctl.b.epdis = 1;
			depctl.b.snak = 1;
		}
		else {
			depctl.d32 = 0;
		}

		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepctl, depctl.d32);

		dwc_write_reg32(&dev_if->out_ep_regs[i]->doeptsiz, 0);
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepdma, 0);
		dwc_write_reg32(&dev_if->out_ep_regs[i]->doepint, 0xFF);
	}

	if(core_if->en_multiple_tx_fifo && core_if->dma_enable) {
		dev_if->non_iso_tx_thr_en = params->thr_ctl & 0x1;
		dev_if->iso_tx_thr_en = (params->thr_ctl >> 1) & 0x1;
		dev_if->rx_thr_en = (params->thr_ctl >> 2) & 0x1;

		dev_if->rx_thr_length = params->rx_thr_length;
		dev_if->tx_thr_length = params->tx_thr_length;

		dev_if->setup_desc_index = 0;

		dthrctl.d32 = 0;
		dthrctl.b.non_iso_thr_en = dev_if->non_iso_tx_thr_en;
		dthrctl.b.iso_thr_en = dev_if->iso_tx_thr_en;
		dthrctl.b.tx_thr_len = dev_if->tx_thr_length;
		dthrctl.b.rx_thr_en = dev_if->rx_thr_en;
		dthrctl.b.rx_thr_len = dev_if->rx_thr_length;

		dwc_write_reg32(&dev_if->dev_global_regs->dtknqr3_dthrctl, dthrctl.d32);

		DWC_DEBUGPL(DBG_CIL, "Non ISO Tx Thr - %d\nISO Tx Thr - %d\nRx Thr - %d\nTx Thr Len - %d\nRx Thr Len - %d\n",
			dthrctl.b.non_iso_thr_en, dthrctl.b.iso_thr_en, dthrctl.b.rx_thr_en, dthrctl.b.tx_thr_len, dthrctl.b.rx_thr_len);

	}

	dwc_otg_enable_device_interrupts(core_if);

	{
		diepmsk_data_t msk = { .d32 = 0 };
		msk.b.txfifoundrn = 1;
		dwc_modify_reg32(&dev_if->dev_global_regs->diepmsk, msk.d32, msk.d32);
	}


}

/**
 * This function enables the Host mode interrupts.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void dwc_otg_enable_host_interrupts(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gintmsk_data_t intr_mask = { .d32 = 0 };

	DWC_DEBUGPL(DBG_CIL, "%s()\n", __func__);

	/* Disable all interrupts. */
	dwc_write_reg32(&global_regs->gintmsk, 0);

	/* Clear any pending interrupts. */
	dwc_write_reg32(&global_regs->gintsts, 0xFFFFFFFF);

	/* Enable the common interrupts */
	dwc_otg_enable_common_interrupts(core_if);

	/*
	 * Enable host mode interrupts without disturbing common
	 * interrupts.
	 */
	intr_mask.b.sofintr = 1;
	intr_mask.b.portintr = 1;
	intr_mask.b.hcintr = 1;

	dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, intr_mask.d32);
}

/**
 * This function disables the Host Mode interrupts.
 *
 * @param core_if Programming view of DWC_otg controller
 */
void dwc_otg_disable_host_interrupts(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs =
	core_if->core_global_regs;
	gintmsk_data_t intr_mask = { .d32 = 0 };

	DWC_DEBUGPL(DBG_CILV, "%s()\n", __func__);

	/*
	 * Disable host mode interrupts without disturbing common
	 * interrupts.
	 */
	intr_mask.b.sofintr = 1;
	intr_mask.b.portintr = 1;
	intr_mask.b.hcintr = 1;
	intr_mask.b.ptxfempty = 1;
	intr_mask.b.nptxfempty = 1;

	dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, 0);
}

/**
 * This function initializes the DWC_otg controller registers for
 * host mode.
 *
 * This function flushes the Tx and Rx FIFOs and it flushes any entries in the
 * request queues. Host channels are reset to ensure that they are ready for
 * performing transfers.
 *
 * @param core_if Programming view of DWC_otg controller
 *
 */
void dwc_otg_core_host_init(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	dwc_otg_host_if_t	*host_if = core_if->host_if;
	dwc_otg_core_params_t	*params = core_if->core_params;
	hprt0_data_t		hprt0 = { .d32 = 0 };
	fifosize_data_t		nptxfifosize;
	fifosize_data_t		ptxfifosize;
	int			i;
	hcchar_data_t		hcchar;
	hcfg_data_t		hcfg;
	dwc_otg_hc_regs_t	*hc_regs;
	int			num_channels;
	gotgctl_data_t	gotgctl = { .d32 = 0 };

	DWC_DEBUGPL(DBG_CILV,"%s(%p)\n", __func__, core_if);

	/* Restart the Phy Clock */
	dwc_write_reg32(core_if->pcgcctl, 0);

	/* Initialize Host Configuration Register */
	init_fslspclksel(core_if);
	if (core_if->core_params->speed == DWC_SPEED_PARAM_FULL)
	{
		hcfg.d32 = dwc_read_reg32(&host_if->host_global_regs->hcfg);
		hcfg.b.fslssupp = 1;
		dwc_write_reg32(&host_if->host_global_regs->hcfg, hcfg.d32);
	}

	/* Configure data FIFO sizes */
	if (core_if->hwcfg2.b.dynamic_fifo && params->enable_dynamic_fifo) {
		DWC_DEBUGPL(DBG_CIL,"Total FIFO Size=%d\n", core_if->total_fifo_size);
		DWC_DEBUGPL(DBG_CIL,"Rx FIFO Size=%d\n", params->host_rx_fifo_size);
		DWC_DEBUGPL(DBG_CIL,"NP Tx FIFO Size=%d\n", params->host_nperio_tx_fifo_size);
		DWC_DEBUGPL(DBG_CIL,"P Tx FIFO Size=%d\n", params->host_perio_tx_fifo_size);

		/* Rx FIFO */
		DWC_DEBUGPL(DBG_CIL,"initial grxfsiz=%08x\n", dwc_read_reg32(&global_regs->grxfsiz));
		dwc_write_reg32(&global_regs->grxfsiz, params->host_rx_fifo_size);
		DWC_DEBUGPL(DBG_CIL,"new grxfsiz=%08x\n", dwc_read_reg32(&global_regs->grxfsiz));

		/* Non-periodic Tx FIFO */
		DWC_DEBUGPL(DBG_CIL,"initial gnptxfsiz=%08x\n", dwc_read_reg32(&global_regs->gnptxfsiz));
		nptxfifosize.b.depth  = params->host_nperio_tx_fifo_size;
		nptxfifosize.b.startaddr = params->host_rx_fifo_size;
		dwc_write_reg32(&global_regs->gnptxfsiz, nptxfifosize.d32);
		DWC_DEBUGPL(DBG_CIL,"new gnptxfsiz=%08x\n", dwc_read_reg32(&global_regs->gnptxfsiz));

		/* Periodic Tx FIFO */
		DWC_DEBUGPL(DBG_CIL,"initial hptxfsiz=%08x\n", dwc_read_reg32(&global_regs->hptxfsiz));
		ptxfifosize.b.depth	 = params->host_perio_tx_fifo_size;
		ptxfifosize.b.startaddr = nptxfifosize.b.startaddr + nptxfifosize.b.depth;
		dwc_write_reg32(&global_regs->hptxfsiz, ptxfifosize.d32);
		DWC_DEBUGPL(DBG_CIL,"new hptxfsiz=%08x\n", dwc_read_reg32(&global_regs->hptxfsiz));
	}

	/* Clear Host Set HNP Enable in the OTG Control Register */
	gotgctl.b.hstsethnpen = 1;
	dwc_modify_reg32(&global_regs->gotgctl, gotgctl.d32, 0);

	/* Make sure the FIFOs are flushed. */
	dwc_otg_flush_tx_fifo(core_if, 0x10 /* all Tx FIFOs */);
	dwc_otg_flush_rx_fifo(core_if);

	/* Flush out any leftover queued requests. */
	num_channels = core_if->core_params->host_channels;
	for (i = 0; i < num_channels; i++)
	{
		hc_regs = core_if->host_if->hc_regs[i];
		hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
		hcchar.b.chen = 0;
		hcchar.b.chdis = 1;
		hcchar.b.epdir = 0;
		dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);
	}

	/* Halt all channels to put them into a known state. */
	for (i = 0; i < num_channels; i++)
	{
		int count = 0;
		hc_regs = core_if->host_if->hc_regs[i];
		hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
		hcchar.b.chen = 1;
		hcchar.b.chdis = 1;
		hcchar.b.epdir = 0;
		dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);
		DWC_DEBUGPL(DBG_HCDV, "%s: Halt channel %d\n", __func__, i);
		do {
			hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
			if (++count > 1000)
			{
				DWC_ERROR("%s: Unable to clear halt on channel %d\n",
					  __func__, i);
				break;
			}
		}
		while (hcchar.b.chen);
	}

	/* Turn on the vbus power. */
	if (dwc_otg_is_host_mode(core_if)) {
		hprt0.d32 = dwc_otg_read_hprt0(core_if);
		DWC_PRINT("Init: Power Port (%d)\n", hprt0.b.prtpwr);
		if (hprt0.b.prtpwr == 0) {
			hprt0.b.prtpwr = 1;
			dwc_write_reg32(host_if->hprt0, hprt0.d32);
		}
	}

	dwc_otg_enable_host_interrupts(core_if);
}

/**
 * Prepares a host channel for transferring packets to/from a specific
 * endpoint. The HCCHARn register is set up with the characteristics specified
 * in _hc. Host channel interrupts that may need to be serviced while this
 * transfer is in progress are enabled.
 *
 * @param core_if Programming view of DWC_otg controller
 * @param hc Information needed to initialize the host channel
 */
void dwc_otg_hc_init(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	uint32_t intr_enable;
	hcintmsk_data_t hc_intr_mask;
	gintmsk_data_t gintmsk = { .d32 = 0 };
	hcchar_data_t hcchar;
	hcsplt_data_t hcsplt;

	uint8_t hc_num = hc->hc_num;
	dwc_otg_host_if_t *host_if = core_if->host_if;
	dwc_otg_hc_regs_t *hc_regs = host_if->hc_regs[hc_num];

	/* Clear old interrupt conditions for this host channel. */
	hc_intr_mask.d32 = 0xFFFFFFFF;
	hc_intr_mask.b.reserved = 0;
	dwc_write_reg32(&hc_regs->hcint, hc_intr_mask.d32);

	/* Enable channel interrupts required for this transfer. */
	hc_intr_mask.d32 = 0;
	hc_intr_mask.b.chhltd = 1;
	if (core_if->dma_enable) {
		hc_intr_mask.b.ahberr = 1;
		if (hc->error_state && !hc->do_split &&
			hc->ep_type != DWC_OTG_EP_TYPE_ISOC) {
			hc_intr_mask.b.ack = 1;
			if (hc->ep_is_in) {
				hc_intr_mask.b.datatglerr = 1;
				if (hc->ep_type != DWC_OTG_EP_TYPE_INTR) {
					hc_intr_mask.b.nak = 1;
				}
			}
		}
	}
	else {
		switch (hc->ep_type) {
		case DWC_OTG_EP_TYPE_CONTROL:
		case DWC_OTG_EP_TYPE_BULK:
			hc_intr_mask.b.xfercompl = 1;
			hc_intr_mask.b.stall = 1;
			hc_intr_mask.b.xacterr = 1;
			hc_intr_mask.b.datatglerr = 1;
			if (hc->ep_is_in) {
				hc_intr_mask.b.bblerr = 1;
			}
			else {
				hc_intr_mask.b.nak = 1;
				hc_intr_mask.b.nyet = 1;
				if (hc->do_ping) {
					hc_intr_mask.b.ack = 1;
				}
			}

			if (hc->do_split) {
				hc_intr_mask.b.nak = 1;
				if (hc->complete_split) {
					hc_intr_mask.b.nyet = 1;
				}
				else {
					hc_intr_mask.b.ack = 1;
				}
			}

			if (hc->error_state) {
				hc_intr_mask.b.ack = 1;
			}
			break;
		case DWC_OTG_EP_TYPE_INTR:
			hc_intr_mask.b.xfercompl = 1;
			hc_intr_mask.b.nak = 1;
			hc_intr_mask.b.stall = 1;
			hc_intr_mask.b.xacterr = 1;
			hc_intr_mask.b.datatglerr = 1;
			hc_intr_mask.b.frmovrun = 1;

			if (hc->ep_is_in) {
				hc_intr_mask.b.bblerr = 1;
			}
			if (hc->error_state) {
				hc_intr_mask.b.ack = 1;
			}
			if (hc->do_split) {
				if (hc->complete_split) {
					hc_intr_mask.b.nyet = 1;
				}
				else {
					hc_intr_mask.b.ack = 1;
				}
			}
			break;
		case DWC_OTG_EP_TYPE_ISOC:
			hc_intr_mask.b.xfercompl = 1;
			hc_intr_mask.b.frmovrun = 1;
			hc_intr_mask.b.ack = 1;

			if (hc->ep_is_in) {
				hc_intr_mask.b.xacterr = 1;
				hc_intr_mask.b.bblerr = 1;
			}
			break;
		}
	}
	dwc_write_reg32(&hc_regs->hcintmsk, hc_intr_mask.d32);

	/* Enable the top level host channel interrupt. */
	intr_enable = (1 << hc_num);
	dwc_modify_reg32(&host_if->host_global_regs->haintmsk, 0, intr_enable);

	/* Make sure host channel interrupts are enabled. */
	gintmsk.b.hcintr = 1;
	dwc_modify_reg32(&core_if->core_global_regs->gintmsk, 0, gintmsk.d32);

	/*
	 * Program the HCCHARn register with the endpoint characteristics for
	 * the current transfer.
	 */
	hcchar.d32 = 0;
	hcchar.b.devaddr = hc->dev_addr;
	hcchar.b.epnum = hc->ep_num;
	hcchar.b.epdir = hc->ep_is_in;
	hcchar.b.lspddev = (hc->speed == DWC_OTG_EP_SPEED_LOW);
	hcchar.b.eptype = hc->ep_type;
	hcchar.b.mps = hc->max_packet;

	dwc_write_reg32(&host_if->hc_regs[hc_num]->hcchar, hcchar.d32);

	DWC_DEBUGPL(DBG_HCDV, "%s: Channel %d\n", __func__, hc->hc_num);
	DWC_DEBUGPL(DBG_HCDV, "	 Dev Addr: %d\n", hcchar.b.devaddr);
	DWC_DEBUGPL(DBG_HCDV, "	 Ep Num: %d\n", hcchar.b.epnum);
	DWC_DEBUGPL(DBG_HCDV, "	 Is In: %d\n", hcchar.b.epdir);
	DWC_DEBUGPL(DBG_HCDV, "	 Is Low Speed: %d\n", hcchar.b.lspddev);
	DWC_DEBUGPL(DBG_HCDV, "	 Ep Type: %d\n", hcchar.b.eptype);
	DWC_DEBUGPL(DBG_HCDV, "	 Max Pkt: %d\n", hcchar.b.mps);
	DWC_DEBUGPL(DBG_HCDV, "	 Multi Cnt: %d\n", hcchar.b.multicnt);

	/*
	 * Program the HCSPLIT register for SPLITs
	 */
	hcsplt.d32 = 0;
	if (hc->do_split) {
		DWC_DEBUGPL(DBG_HCDV, "Programming HC %d with split --> %s\n", hc->hc_num,
			   hc->complete_split ? "CSPLIT" : "SSPLIT");
		hcsplt.b.compsplt = hc->complete_split;
		hcsplt.b.xactpos = hc->xact_pos;
		hcsplt.b.hubaddr = hc->hub_addr;
		hcsplt.b.prtaddr = hc->port_addr;
		DWC_DEBUGPL(DBG_HCDV, "	  comp split %d\n", hc->complete_split);
		DWC_DEBUGPL(DBG_HCDV, "	  xact pos %d\n", hc->xact_pos);
		DWC_DEBUGPL(DBG_HCDV, "	  hub addr %d\n", hc->hub_addr);
		DWC_DEBUGPL(DBG_HCDV, "	  port addr %d\n", hc->port_addr);
		DWC_DEBUGPL(DBG_HCDV, "	  is_in %d\n", hc->ep_is_in);
		DWC_DEBUGPL(DBG_HCDV, "	  Max Pkt: %d\n", hcchar.b.mps);
		DWC_DEBUGPL(DBG_HCDV, "	  xferlen: %d\n", hc->xfer_len);
	}
	dwc_write_reg32(&host_if->hc_regs[hc_num]->hcsplt, hcsplt.d32);

}

/**
 * Attempts to halt a host channel. This function should only be called in
 * Slave mode or to abort a transfer in either Slave mode or DMA mode. Under
 * normal circumstances in DMA mode, the controller halts the channel when the
 * transfer is complete or a condition occurs that requires application
 * intervention.
 *
 * In slave mode, checks for a free request queue entry, then sets the Channel
 * Enable and Channel Disable bits of the Host Channel Characteristics
 * register of the specified channel to intiate the halt. If there is no free
 * request queue entry, sets only the Channel Disable bit of the HCCHARn
 * register to flush requests for this channel. In the latter case, sets a
 * flag to indicate that the host channel needs to be halted when a request
 * queue slot is open.
 *
 * In DMA mode, always sets the Channel Enable and Channel Disable bits of the
 * HCCHARn register. The controller ensures there is space in the request
 * queue before submitting the halt request.
 *
 * Some time may elapse before the core flushes any posted requests for this
 * host channel and halts. The Channel Halted interrupt handler completes the
 * deactivation of the host channel.
 *
 * @param core_if Controller register interface.
 * @param hc Host channel to halt.
 * @param halt_status Reason for halting the channel.
 */
void dwc_otg_hc_halt(dwc_otg_core_if_t *core_if,
			 dwc_hc_t *hc,
			 dwc_otg_halt_status_e halt_status)
{
	gnptxsts_data_t			nptxsts;
	hptxsts_data_t			hptxsts;
	hcchar_data_t			hcchar;
	dwc_otg_hc_regs_t		*hc_regs;
	dwc_otg_core_global_regs_t	*global_regs;
	dwc_otg_host_global_regs_t	*host_global_regs;

	hc_regs = core_if->host_if->hc_regs[hc->hc_num];
	global_regs = core_if->core_global_regs;
	host_global_regs = core_if->host_if->host_global_regs;

	WARN_ON(halt_status == DWC_OTG_HC_XFER_NO_HALT_STATUS);

	if (halt_status == DWC_OTG_HC_XFER_URB_DEQUEUE ||
		halt_status == DWC_OTG_HC_XFER_AHB_ERR) {
		/*
		 * Disable all channel interrupts except Ch Halted. The QTD
		 * and QH state associated with this transfer has been cleared
		 * (in the case of URB_DEQUEUE), so the channel needs to be
		 * shut down carefully to prevent crashes.
		 */
		hcintmsk_data_t hcintmsk;
		hcintmsk.d32 = 0;
		hcintmsk.b.chhltd = 1;
		dwc_write_reg32(&hc_regs->hcintmsk, hcintmsk.d32);

		/*
		 * Make sure no other interrupts besides halt are currently
		 * pending. Handling another interrupt could cause a crash due
		 * to the QTD and QH state.
		 */
		dwc_write_reg32(&hc_regs->hcint, ~hcintmsk.d32);

		/*
		 * Make sure the halt status is set to URB_DEQUEUE or AHB_ERR
		 * even if the channel was already halted for some other
		 * reason.
		 */
		hc->halt_status = halt_status;

		hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
		if (hcchar.b.chen == 0) {
			/*
			 * The channel is either already halted or it hasn't
			 * started yet. In DMA mode, the transfer may halt if
			 * it finishes normally or a condition occurs that
			 * requires driver intervention. Don't want to halt
			 * the channel again. In either Slave or DMA mode,
			 * it's possible that the transfer has been assigned
			 * to a channel, but not started yet when an URB is
			 * dequeued. Don't want to halt a channel that hasn't
			 * started yet.
			 */
			return;
		}
	}

	if (hc->halt_pending) {
		/*
		 * A halt has already been issued for this channel. This might
		 * happen when a transfer is aborted by a higher level in
		 * the stack.
		 */
#ifdef DEBUG
		DWC_PRINT("*** %s: Channel %d, _hc->halt_pending already set ***\n",
			  __func__, hc->hc_num);

/*		dwc_otg_dump_global_registers(core_if); */
/*		dwc_otg_dump_host_registers(core_if); */
#endif
		return;
	}

	hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
	hcchar.b.chen = 1;
	hcchar.b.chdis = 1;

	if (!core_if->dma_enable) {
		/* Check for space in the request queue to issue the halt. */
		if (hc->ep_type == DWC_OTG_EP_TYPE_CONTROL ||
			hc->ep_type == DWC_OTG_EP_TYPE_BULK) {
			nptxsts.d32 = dwc_read_reg32(&global_regs->gnptxsts);
			if (nptxsts.b.nptxqspcavail == 0) {
				hcchar.b.chen = 0;
			}
		}
		else {
			hptxsts.d32 = dwc_read_reg32(&host_global_regs->hptxsts);
			if ((hptxsts.b.ptxqspcavail == 0) || (core_if->queuing_high_bandwidth)) {
				hcchar.b.chen = 0;
			}
		}
	}

	dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);

	hc->halt_status = halt_status;

	if (hcchar.b.chen) {
		hc->halt_pending = 1;
		hc->halt_on_queue = 0;
	}
	else {
		hc->halt_on_queue = 1;
	}

	DWC_DEBUGPL(DBG_HCDV, "%s: Channel %d\n", __func__, hc->hc_num);
	DWC_DEBUGPL(DBG_HCDV, "	 hcchar: 0x%08x\n", hcchar.d32);
	DWC_DEBUGPL(DBG_HCDV, "	 halt_pending: %d\n", hc->halt_pending);
	DWC_DEBUGPL(DBG_HCDV, "	 halt_on_queue: %d\n", hc->halt_on_queue);
	DWC_DEBUGPL(DBG_HCDV, "	 halt_status: %d\n", hc->halt_status);

	return;
}

/**
 * Clears the transfer state for a host channel. This function is normally
 * called after a transfer is done and the host channel is being released.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param hc Identifies the host channel to clean up.
 */
void dwc_otg_hc_cleanup(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	dwc_otg_hc_regs_t *hc_regs;
	hcchar_data_t hcchar;

	hc->xfer_started = 0;

	/*
	 * Clear channel interrupt enables and any unhandled channel interrupt
	 * conditions.
	 */
	hc_regs = core_if->host_if->hc_regs[hc->hc_num];
	dwc_write_reg32(&hc_regs->hcintmsk, 0);
	dwc_write_reg32(&hc_regs->hcint, 0xFFFFFFFF);

	del_timer(&core_if->hc_xfer_timer[hc->hc_num]);
	hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
	if (hcchar.b.chdis) {
		DWC_WARN("%s: chdis set, channel %d, hcchar 0x%08x\n",
			 __func__, hc->hc_num, hcchar.d32);
	}
}

/**
 * Sets the channel property that indicates in which frame a periodic transfer
 * should occur. This is always set to the _next_ frame. This function has no
 * effect on non-periodic transfers.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param hc Identifies the host channel to set up and its properties.
 * @param hcchar Current value of the HCCHAR register for the specified host
 * channel.
 */
static inline void hc_set_even_odd_frame(dwc_otg_core_if_t *core_if,
					 dwc_hc_t *hc,
					 hcchar_data_t *hcchar)
{
	if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
		hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
		hfnum_data_t	hfnum;
		hfnum.d32 = dwc_read_reg32(&core_if->host_if->host_global_regs->hfnum);

		/* 1 if _next_ frame is odd, 0 if it's even */
		hcchar->b.oddfrm = (hfnum.b.frnum & 0x1) ? 0 : 1;
#ifdef DEBUG
		if (hc->ep_type == DWC_OTG_EP_TYPE_INTR && hc->do_split && !hc->complete_split) {
			switch (hfnum.b.frnum & 0x7) {
			case 7:
				core_if->hfnum_7_samples++;
				core_if->hfnum_7_frrem_accum += hfnum.b.frrem;
				break;
			case 0:
				core_if->hfnum_0_samples++;
				core_if->hfnum_0_frrem_accum += hfnum.b.frrem;
				break;
			default:
				core_if->hfnum_other_samples++;
				core_if->hfnum_other_frrem_accum += hfnum.b.frrem;
				break;
			}
		}
#endif
	}
}

static void hc_xfer_timeout(unsigned long ptr)
{
	hc_xfer_info_t *xfer_info = (hc_xfer_info_t *)ptr;
	int hc_num = xfer_info->hc->hc_num;
	DWC_WARN("%s: timeout on channel %d\n", __func__, hc_num);
#ifdef DEBUG
	DWC_WARN("start_hcchar_val 0x%08x\n", xfer_info->core_if->start_hcchar_val[hc_num]);
#endif
	dwc_otg_hc_cleanup(xfer_info->core_if, xfer_info->hc);
}


/*
 * This function does the setup for a data transfer for a host channel and
 * starts the transfer. May be called in either Slave mode or DMA mode. In
 * Slave mode, the caller must ensure that there is sufficient space in the
 * request queue and Tx Data FIFO.
 *
 * For an OUT transfer in Slave mode, it loads a data packet into the
 * appropriate FIFO. If necessary, additional data packets will be loaded in
 * the Host ISR.
 *
 * For an IN transfer in Slave mode, a data packet is requested. The data
 * packets are unloaded from the Rx FIFO in the Host ISR. If necessary,
 * additional data packets are requested in the Host ISR.
 *
 * For a PING transfer in Slave mode, the Do Ping bit is set in the HCTSIZ
 * register along with a packet count of 1 and the channel is enabled. This
 * causes a single PING transaction to occur. Other fields in HCTSIZ are
 * simply set to 0 since no data transfer occurs in this case.
 *
 * For a PING transfer in DMA mode, the HCTSIZ register is initialized with
 * all the information required to perform the subsequent data transfer. In
 * addition, the Do Ping bit is set in the HCTSIZ register. In this case, the
 * controller performs the entire PING protocol, then starts the data
 * transfer.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param hc Information needed to initialize the host channel. The xfer_len
 * value may be reduced to accommodate the max widths of the XferSize and
 * PktCnt fields in the HCTSIZn register. The multi_count value may be changed
 * to reflect the final xfer_len value.
 */
void dwc_otg_hc_start_transfer(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	hcchar_data_t hcchar;
	hctsiz_data_t hctsiz;
	uint16_t num_packets;
	uint32_t max_hc_xfer_size = core_if->core_params->max_transfer_size;
	uint16_t max_hc_pkt_count = core_if->core_params->max_packet_count;
	dwc_otg_hc_regs_t *hc_regs = core_if->host_if->hc_regs[hc->hc_num];

	hctsiz.d32 = 0;

	if (hc->do_ping) {
		if (!core_if->dma_enable) {
			dwc_otg_hc_do_ping(core_if, hc);
			hc->xfer_started = 1;
			return;
		}
		else {
			hctsiz.b.dopng = 1;
		}
	}

	if (hc->do_split) {
		num_packets = 1;

		if (hc->complete_split && !hc->ep_is_in) {
			/* For CSPLIT OUT Transfer, set the size to 0 so the
			 * core doesn't expect any data written to the FIFO */
			hc->xfer_len = 0;
		}
		else if (hc->ep_is_in || (hc->xfer_len > hc->max_packet)) {
			hc->xfer_len = hc->max_packet;
		}
		else if (!hc->ep_is_in && (hc->xfer_len > 188)) {
			hc->xfer_len = 188;
		}

		hctsiz.b.xfersize = hc->xfer_len;
	}
	else {
		/*
		 * Ensure that the transfer length and packet count will fit
		 * in the widths allocated for them in the HCTSIZn register.
		 */
		if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
			hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
			/*
			 * Make sure the transfer size is no larger than one
			 * (micro)frame's worth of data. (A check was done
			 * when the periodic transfer was accepted to ensure
			 * that a (micro)frame's worth of data can be
			 * programmed into a channel.)
			 */
			uint32_t max_periodic_len = hc->multi_count * hc->max_packet;
			if (hc->xfer_len > max_periodic_len) {
				hc->xfer_len = max_periodic_len;
			}
			else {
			}
		}
		else if (hc->xfer_len > max_hc_xfer_size) {
			/* Make sure that xfer_len is a multiple of max packet size. */
			hc->xfer_len = max_hc_xfer_size - hc->max_packet + 1;
		}

		if (hc->xfer_len > 0) {
			num_packets = (hc->xfer_len + hc->max_packet - 1) / hc->max_packet;
			if (num_packets > max_hc_pkt_count) {
				num_packets = max_hc_pkt_count;
				hc->xfer_len = num_packets * hc->max_packet;
			}
		}
		else {
			/* Need 1 packet for transfer length of 0. */
			num_packets = 1;
		}

		if (hc->ep_is_in) {
			/* Always program an integral # of max packets for IN transfers. */
			hc->xfer_len = num_packets * hc->max_packet;
		}

		if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
			hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
			/*
			 * Make sure that the multi_count field matches the
			 * actual transfer length.
			 */
			hc->multi_count = num_packets;
		}

		if (hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
			/* Set up the initial PID for the transfer. */
			if (hc->speed == DWC_OTG_EP_SPEED_HIGH) {
				if (hc->ep_is_in) {
					if (hc->multi_count == 1) {
						hc->data_pid_start = DWC_OTG_HC_PID_DATA0;
					}
					else if (hc->multi_count == 2) {
						hc->data_pid_start = DWC_OTG_HC_PID_DATA1;
					}
					else {
						hc->data_pid_start = DWC_OTG_HC_PID_DATA2;
					}
				}
				else {
					if (hc->multi_count == 1) {
						hc->data_pid_start = DWC_OTG_HC_PID_DATA0;
					}
					else {
						hc->data_pid_start = DWC_OTG_HC_PID_MDATA;
					}
				}
			}
			else {
				hc->data_pid_start = DWC_OTG_HC_PID_DATA0;
			}
		}

		hctsiz.b.xfersize = hc->xfer_len;
	}

	hc->start_pkt_count = num_packets;
	hctsiz.b.pktcnt = num_packets;
	hctsiz.b.pid = hc->data_pid_start;
	dwc_write_reg32(&hc_regs->hctsiz, hctsiz.d32);

	DWC_DEBUGPL(DBG_HCDV, "%s: Channel %d\n", __func__, hc->hc_num);
	DWC_DEBUGPL(DBG_HCDV, "	 Xfer Size: %d\n", hctsiz.b.xfersize);
	DWC_DEBUGPL(DBG_HCDV, "	 Num Pkts: %d\n", hctsiz.b.pktcnt);
	DWC_DEBUGPL(DBG_HCDV, "	 Start PID: %d\n", hctsiz.b.pid);

	if (core_if->dma_enable) {
		if (((unsigned long)hc->xfer_buff) & 3) {
			printk(KERN_ERR "dwc_otg: unaligned dma transfer not supported\n");
		}
		dwc_write_reg32(&hc_regs->hcdma, (uint32_t)hc->xfer_buff);
	}

	/* Start the split */
	if (hc->do_split) {
		hcsplt_data_t hcsplt;
		hcsplt.d32 = dwc_read_reg32 (&hc_regs->hcsplt);
		hcsplt.b.spltena = 1;
		dwc_write_reg32(&hc_regs->hcsplt, hcsplt.d32);
	}

	hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
	hcchar.b.multicnt = hc->multi_count;
	hc_set_even_odd_frame(core_if, hc, &hcchar);
#ifdef DEBUG
	core_if->start_hcchar_val[hc->hc_num] = hcchar.d32;
	if (hcchar.b.chdis) {
		DWC_WARN("%s: chdis set, channel %d, hcchar 0x%08x\n",
			 __func__, hc->hc_num, hcchar.d32);
	}
#endif

	/* Set host channel enable after all other setup is complete. */
	hcchar.b.chen = 1;
	hcchar.b.chdis = 0;
	dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);

	hc->xfer_started = 1;
	hc->requests++;

	if (!core_if->dma_enable &&
		!hc->ep_is_in && hc->xfer_len > 0) {
		/* Load OUT packet into the appropriate Tx FIFO. */
		dwc_otg_hc_write_packet(core_if, hc);
	}

	if (core_if->core_params->xfer_debug) {
		/* Start a timer for this transfer. */
		core_if->hc_xfer_timer[hc->hc_num].function = hc_xfer_timeout;
		core_if->hc_xfer_info[hc->hc_num].core_if = core_if;
		core_if->hc_xfer_info[hc->hc_num].hc = hc;
		core_if->hc_xfer_timer[hc->hc_num].data = (unsigned long)(&core_if->hc_xfer_info[hc->hc_num]);
		core_if->hc_xfer_timer[hc->hc_num].expires = jiffies + (HZ*10);
		add_timer(&core_if->hc_xfer_timer[hc->hc_num]);
	}
}

/**
 * This function continues a data transfer that was started by previous call
 * to <code>dwc_otg_hc_start_transfer</code>. The caller must ensure there is
 * sufficient space in the request queue and Tx Data FIFO. This function
 * should only be called in Slave mode. In DMA mode, the controller acts
 * autonomously to complete transfers programmed to a host channel.
 *
 * For an OUT transfer, a new data packet is loaded into the appropriate FIFO
 * if there is any data remaining to be queued. For an IN transfer, another
 * data packet is always requested. For the SETUP phase of a control transfer,
 * this function does nothing.
 *
 * @return 1 if a new request is queued, 0 if no more requests are required
 * for this transfer.
 */
int dwc_otg_hc_continue_transfer(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	DWC_DEBUGPL(DBG_HCDV, "%s: Channel %d\n", __func__, hc->hc_num);

	if (hc->do_split) {
		/* SPLITs always queue just once per channel */
		return 0;
	}
	else if (hc->data_pid_start == DWC_OTG_HC_PID_SETUP) {
		/* SETUPs are queued only once since they can't be NAKed. */
		return 0;
	}
	else if (hc->ep_is_in) {
		/*
		 * Always queue another request for other IN transfers. If
		 * back-to-back INs are issued and NAKs are received for both,
		 * the driver may still be processing the first NAK when the
		 * second NAK is received. When the interrupt handler clears
		 * the NAK interrupt for the first NAK, the second NAK will
		 * not be seen. So we can't depend on the NAK interrupt
		 * handler to requeue a NAKed request. Instead, IN requests
		 * are issued each time this function is called. When the
		 * transfer completes, the extra requests for the channel will
		 * be flushed.
		 */
		hcchar_data_t hcchar;
		dwc_otg_hc_regs_t *hc_regs = core_if->host_if->hc_regs[hc->hc_num];

		hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
		hc_set_even_odd_frame(core_if, hc, &hcchar);
		hcchar.b.chen = 1;
		hcchar.b.chdis = 0;
		DWC_DEBUGPL(DBG_HCDV, "	 IN xfer: hcchar = 0x%08x\n", hcchar.d32);
		dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);
		hc->requests++;
		return 1;
	}
	else {
		/* OUT transfers. */
		if (hc->xfer_count < hc->xfer_len) {
			if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
				hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
				hcchar_data_t hcchar;
				dwc_otg_hc_regs_t *hc_regs;
				hc_regs = core_if->host_if->hc_regs[hc->hc_num];
				hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
				hc_set_even_odd_frame(core_if, hc, &hcchar);
			}

			/* Load OUT packet into the appropriate Tx FIFO. */
			dwc_otg_hc_write_packet(core_if, hc);
			hc->requests++;
			return 1;
		}
		else {
			return 0;
		}
	}
}

/**
 * Starts a PING transfer. This function should only be called in Slave mode.
 * The Do Ping bit is set in the HCTSIZ register, then the channel is enabled.
 */
void dwc_otg_hc_do_ping(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	hcchar_data_t hcchar;
	hctsiz_data_t hctsiz;
	dwc_otg_hc_regs_t *hc_regs = core_if->host_if->hc_regs[hc->hc_num];

	DWC_DEBUGPL(DBG_HCDV, "%s: Channel %d\n", __func__, hc->hc_num);

	hctsiz.d32 = 0;
	hctsiz.b.dopng = 1;
	hctsiz.b.pktcnt = 1;
	dwc_write_reg32(&hc_regs->hctsiz, hctsiz.d32);

	hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
	hcchar.b.chen = 1;
	hcchar.b.chdis = 0;
	dwc_write_reg32(&hc_regs->hcchar, hcchar.d32);
}

/*
 * This function writes a packet into the Tx FIFO associated with the Host
 * Channel. For a channel associated with a non-periodic EP, the non-periodic
 * Tx FIFO is written. For a channel associated with a periodic EP, the
 * periodic Tx FIFO is written. This function should only be called in Slave
 * mode.
 *
 * Upon return the xfer_buff and xfer_count fields in _hc are incremented by
 * then number of bytes written to the Tx FIFO.
 */
void dwc_otg_hc_write_packet(dwc_otg_core_if_t *core_if, dwc_hc_t *hc)
{
	uint32_t i;
	uint32_t remaining_count;
	uint32_t byte_count;
	uint32_t dword_count;

	uint32_t *data_buff = (uint32_t *)(hc->xfer_buff);
	uint32_t *data_fifo = core_if->data_fifo[hc->hc_num];

	remaining_count = hc->xfer_len - hc->xfer_count;
	if (remaining_count > hc->max_packet) {
		byte_count = hc->max_packet;
	}
	else {
		byte_count = remaining_count;
	}

	dword_count = (byte_count + 3) / 4;

	if ((((unsigned long)data_buff) & 0x3) == 0) {
		/* xfer_buff is DWORD aligned. */
		for (i = 0; i < dword_count; i++, data_buff++)
		{
			dwc_write_reg32(data_fifo, *data_buff);
		}
	}
	else {
		/* xfer_buff is not DWORD aligned. */
		for (i = 0; i < dword_count; i++, data_buff++)
		{
			dwc_write_reg32(data_fifo, get_unaligned(data_buff));
		}
	}

	hc->xfer_count += byte_count;
	hc->xfer_buff += byte_count;
}

/**
 * Gets the current USB frame number. This is the frame number from the last
 * SOF packet.
 */
uint32_t dwc_otg_get_frame_number(dwc_otg_core_if_t *core_if)
{
	dsts_data_t dsts;
	dsts.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dsts);

	/* read current frame/microframe number from DSTS register */
	return dsts.b.soffn;
}

/**
 * This function reads a setup packet from the Rx FIFO into the destination
 * buffer.	This function is called from the Rx Status Queue Level (RxStsQLvl)
 * Interrupt routine when a SETUP packet has been received in Slave mode.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param dest Destination buffer for packet data.
 */
void dwc_otg_read_setup_packet(dwc_otg_core_if_t *core_if, uint32_t *dest)
{
	/* Get the 8 bytes of a setup transaction data */

	/* Pop 2 DWORDS off the receive data FIFO into memory */
	dest[0] = dwc_read_reg32(core_if->data_fifo[0]);
	dest[1] = dwc_read_reg32(core_if->data_fifo[0]);
}


/**
 * This function enables EP0 OUT to receive SETUP packets and configures EP0
 * IN for transmitting packets.	 It is normally called when the
 * "Enumeration Done" interrupt occurs.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_activate(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	dsts_data_t dsts;
	depctl_data_t diepctl;
	depctl_data_t doepctl;
	dctl_data_t dctl = { .d32 = 0 };

	/* Read the Device Status and Endpoint 0 Control registers */
	dsts.d32 = dwc_read_reg32(&dev_if->dev_global_regs->dsts);
	diepctl.d32 = dwc_read_reg32(&dev_if->in_ep_regs[0]->diepctl);
	doepctl.d32 = dwc_read_reg32(&dev_if->out_ep_regs[0]->doepctl);

	/* Set the MPS of the IN EP based on the enumeration speed */
	switch (dsts.b.enumspd) {
	case DWC_DSTS_ENUMSPD_HS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_30MHZ_OR_60MHZ:
	case DWC_DSTS_ENUMSPD_FS_PHY_48MHZ:
		diepctl.b.mps = DWC_DEP0CTL_MPS_64;
		break;
	case DWC_DSTS_ENUMSPD_LS_PHY_6MHZ:
		diepctl.b.mps = DWC_DEP0CTL_MPS_8;
		break;
	}

	dwc_write_reg32(&dev_if->in_ep_regs[0]->diepctl, diepctl.d32);

	/* Enable OUT EP for receive */
	doepctl.b.epena = 1;
	dwc_write_reg32(&dev_if->out_ep_regs[0]->doepctl, doepctl.d32);

#ifdef VERBOSE
	DWC_DEBUGPL(DBG_PCDV,"doepctl0=%0x\n",
		dwc_read_reg32(&dev_if->out_ep_regs[0]->doepctl));
	DWC_DEBUGPL(DBG_PCDV,"diepctl0=%0x\n",
		dwc_read_reg32(&dev_if->in_ep_regs[0]->diepctl));
#endif
	dctl.b.cgnpinnak = 1;
	dwc_modify_reg32(&dev_if->dev_global_regs->dctl, dctl.d32, dctl.d32);
	DWC_DEBUGPL(DBG_PCDV,"dctl=%0x\n",
		dwc_read_reg32(&dev_if->dev_global_regs->dctl));
}

/**
 * This function activates an EP.  The Device EP control register for
 * the EP is configured as defined in the ep structure.	 Note: This
 * function is not used for EP0.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to activate.
 */
void dwc_otg_ep_activate(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	dwc_otg_dev_if_t *dev_if = core_if->dev_if;
	depctl_data_t depctl;
	volatile uint32_t *addr;
	daint_data_t daintmsk = { .d32 = 0 };

	DWC_DEBUGPL(DBG_PCDV, "%s() EP%d-%s\n", __func__, ep->num,
		(ep->is_in?"IN":"OUT"));

	/* Read DEPCTLn register */
	if (ep->is_in == 1) {
		addr = &dev_if->in_ep_regs[ep->num]->diepctl;
			daintmsk.ep.in = 1<<ep->num;
	}
	else {
		addr = &dev_if->out_ep_regs[ep->num]->doepctl;
			daintmsk.ep.out = 1<<ep->num;
	}

	/* If the EP is already active don't change the EP Control
	 * register. */
	depctl.d32 = dwc_read_reg32(addr);
	if (!depctl.b.usbactep) {
		depctl.b.mps = ep->maxpacket;
		depctl.b.eptype = ep->type;
		depctl.b.txfnum = ep->tx_fifo_num;

		if (ep->type == DWC_OTG_EP_TYPE_ISOC) {
			depctl.b.setd0pid = 1; // ???
		}
		else {
			depctl.b.setd0pid = 1;
		}
		depctl.b.usbactep = 1;

		dwc_write_reg32(addr, depctl.d32);
		DWC_DEBUGPL(DBG_PCDV,"DEPCTL=%08x\n", dwc_read_reg32(addr));
	}

	/* Enable the Interrupt for this EP */
	dwc_modify_reg32(&dev_if->dev_global_regs->daintmsk,
					 0, daintmsk.d32);

	DWC_DEBUGPL(DBG_PCDV,"DAINTMSK=%0x\n",
		dwc_read_reg32(&dev_if->dev_global_regs->daintmsk));

	ep->stall_clear_flag = 0;
	return;
}

/**
 * This function deactivates an EP. This is done by clearing the USB Active
 * EP bit in the Device EP control register. Note: This function is not used
 * for EP0. EP0 cannot be deactivated.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to deactivate.
 */
void dwc_otg_ep_deactivate(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	depctl_data_t depctl = { .d32 = 0 };
	volatile uint32_t *addr;
	daint_data_t daintmsk = { .d32 = 0};

	/* Read DEPCTLn register */
	if (ep->is_in == 1) {
		addr = &core_if->dev_if->in_ep_regs[ep->num]->diepctl;
				daintmsk.ep.in = 1<<ep->num;
	}
	else {
		addr = &core_if->dev_if->out_ep_regs[ep->num]->doepctl;
			daintmsk.ep.out = 1<<ep->num;
	}

	depctl.b.usbactep = 0;

	if(core_if->dma_desc_enable)
		depctl.b.epdis = 1;

	dwc_write_reg32(addr, depctl.d32);

	/* Disable the Interrupt for this EP */
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->daintmsk,
					 daintmsk.d32, 0);

	return;
}

/**
 * This function does the setup for a data transfer for an EP and
 * starts the transfer.	 For an IN transfer, the packets will be
 * loaded into the appropriate Tx FIFO in the ISR. For OUT transfers,
 * the packets are unloaded from the Rx FIFO in the ISR.  the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 */

void dwc_otg_ep_start_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	/** @todo Refactor this funciton to check the transfer size
	 * count value does not execed the number bits in the Transfer
	 * count register. */
	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
	gintmsk_data_t intr_mask = { .d32 = 0};
	dwc_otg_dma_desc_t* dma_desc;

	DWC_DEBUGPL((DBG_PCDV | DBG_CILV), "%s()\n", __func__);

	DWC_DEBUGPL(DBG_PCD, "ep%d-%s xfer_len=%d xfer_cnt=%d "
		"xfer_buff=%p start_xfer_buff=%p\n",
		ep->num, (ep->is_in?"IN":"OUT"), ep->xfer_len,
		ep->xfer_count, ep->xfer_buff, ep->start_xfer_buff);

	if (core_if->dma_enable && (ep->dma_addr & 3)) {
		printk(KERN_ERR "dwc_otg: unaligned dma transfer not supported\n");
	}

	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
			core_if->dev_if->in_ep_regs[ep->num];

		gnptxsts_data_t gtxstatus;

		gtxstatus.d32 =
			dwc_read_reg32(&core_if->core_global_regs->gnptxsts);

		if(core_if->en_multiple_tx_fifo == 0 && gtxstatus.b.nptxqspcavail == 0) {
#ifdef DEBUG
			DWC_PRINT("TX Queue Full (0x%0x)\n", gtxstatus.d32);
#endif
			return;
		}

		depctl.d32 = dwc_read_reg32(&(in_regs->diepctl));
		deptsiz.d32 = dwc_read_reg32(&(in_regs->dieptsiz));

		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			deptsiz.b.xfersize = 0;
			deptsiz.b.pktcnt = 1;
		}
		else {
			/* Program the transfer size and packet count
			 *	as follows: xfersize = N * maxpacket +
			 *	short_packet pktcnt = N + (short_packet
			 *	exist ? 1 : 0)
			 */
			deptsiz.b.xfersize = ep->xfer_len;
			deptsiz.b.pktcnt =
				(ep->xfer_len - 1 + ep->maxpacket) /
				ep->maxpacket;
		}


		/* Write the DMA register */
		if (core_if->dma_enable) {
			if (core_if->dma_desc_enable == 0) {
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
				dwc_write_reg32 (&(in_regs->diepdma),
						 (uint32_t)ep->dma_addr);
			}
			else {
				dma_desc = ep->desc_addr;

				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.sp = ep->sent_zlp; //(_ep->xfer_len % _ep->maxpacket) ? 1 : 0;
				dma_desc->status.b.bytes = ep->xfer_len;
				dma_desc->buf = (uint32_t*)ep->dma_addr;
				dma_desc->status.b.bs = BS_HOST_READY;

				/** DIEPDMAn Register write */
				dwc_write_reg32(&in_regs->diepdma, ep->dma_desc_addr);
			}

		}
		else {
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
			if(ep->type != DWC_OTG_EP_TYPE_ISOC) {
				/**
				 * Enable the Non-Periodic Tx FIFO empty interrupt,
				 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
				 * the data will be written into the fifo by the ISR.
				 */
				if(core_if->en_multiple_tx_fifo == 0) {
					intr_mask.b.nptxfempty = 1;
					dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
						intr_mask.d32, intr_mask.d32);
				}
				else {
					/* Enable the Tx FIFO Empty Interrupt for this EP */
					if(ep->xfer_len > 0) {
						uint32_t fifoemptymsk = 0;
						fifoemptymsk = 1 << ep->num;
						dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
						0, fifoemptymsk);
					}
				}
			}
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		depctl.d32 = dwc_read_reg32 (&core_if->dev_if->in_ep_regs[0]->diepctl);
		depctl.b.nextep = ep->num;
		dwc_write_reg32 (&core_if->dev_if->in_ep_regs[0]->diepctl, depctl.d32);

	}
	else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
		core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&(out_regs->doepctl));
		deptsiz.d32 = dwc_read_reg32(&(out_regs->doeptsiz));

		/* Program the transfer size and packet count as follows:
		 *
		 *	pktcnt = N
		 *	xfersize = N * maxpacket
		 */
		if (ep->xfer_len == 0) {
			/* Zero Length Packet */
			deptsiz.b.xfersize = ep->maxpacket;
			deptsiz.b.pktcnt = 1;
		}
		else {
			deptsiz.b.pktcnt =
					(ep->xfer_len + (ep->maxpacket - 1)) /
					ep->maxpacket;
			deptsiz.b.xfersize = deptsiz.b.pktcnt * ep->maxpacket;
		}

		DWC_DEBUGPL(DBG_PCDV, "ep%d xfersize=%d pktcnt=%d\n",
			ep->num,
			deptsiz.b.xfersize, deptsiz.b.pktcnt);

		if (core_if->dma_enable) {
			if (!core_if->dma_desc_enable) {
				dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);

				dwc_write_reg32 (&(out_regs->doepdma),
					(uint32_t)ep->dma_addr);
			}
			else {
				dma_desc = ep->desc_addr;

				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.bytes = ep->xfer_len;
				dma_desc->buf = (uint32_t*)ep->dma_addr;
				dma_desc->status.b.bs = BS_HOST_READY;

				/** DOEPDMAn Register write */
				dwc_write_reg32(&out_regs->doepdma, ep->dma_desc_addr);
			}
		}
		else {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		}

		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;

		dwc_write_reg32(&out_regs->doepctl, depctl.d32);

		DWC_DEBUGPL(DBG_PCD, "DOEPCTL=%08x DOEPTSIZ=%08x\n",
			dwc_read_reg32(&out_regs->doepctl),
			dwc_read_reg32(&out_regs->doeptsiz));
		DWC_DEBUGPL(DBG_PCD, "DAINTMSK=%08x GINTMSK=%08x\n",
			dwc_read_reg32(&core_if->dev_if->dev_global_regs->daintmsk),
			dwc_read_reg32(&core_if->core_global_regs->gintmsk));
	}
}


/**
 * This function does the setup for a data transfer for an EP and
 * starts the transfer.	 For an IN transfer, the packets will be
 * loaded into the appropriate Tx FIFO in the ISR. For OUT transfers,
 * the packets are unloaded from the Rx FIFO in the ISR.  the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to start the transfer on.
 */

void dwc_otg_ep_start_zl_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{

 	depctl_data_t depctl;
	deptsiz_data_t deptsiz;
	gintmsk_data_t intr_mask = { .d32 = 0};

	DWC_DEBUGPL((DBG_PCDV | DBG_CILV), "%s()\n", __func__);

	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
			core_if->dev_if->in_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&(in_regs->diepctl));
		deptsiz.d32 = dwc_read_reg32(&(in_regs->dieptsiz));

		deptsiz.b.xfersize = 0;
		deptsiz.b.pktcnt = 1;


		/* Write the DMA register */
		if (core_if->dma_enable) {
			if (core_if->dma_desc_enable == 0) {
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
				dwc_write_reg32 (&(in_regs->diepdma),
						 (uint32_t)ep->dma_addr);
			}
		}
		else {
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
			/**
			 * Enable the Non-Periodic Tx FIFO empty interrupt,
			 * or the Tx FIFO epmty interrupt in dedicated Tx FIFO mode,
			 * the data will be written into the fifo by the ISR.
			 */
			if(core_if->en_multiple_tx_fifo == 0) {
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
					intr_mask.d32, intr_mask.d32);
			}
			else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if(ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk = 1 << ep->num;
					dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
					0, fifoemptymsk);
				}
			}
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		depctl.d32 = dwc_read_reg32 (&core_if->dev_if->in_ep_regs[0]->diepctl);
		depctl.b.nextep = ep->num;
		dwc_write_reg32 (&core_if->dev_if->in_ep_regs[0]->diepctl, depctl.d32);

	}
	else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
		core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&(out_regs->doepctl));
		deptsiz.d32 = dwc_read_reg32(&(out_regs->doeptsiz));

		/* Zero Length Packet */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

		if (core_if->dma_enable) {
			if (!core_if->dma_desc_enable) {
				dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);

				dwc_write_reg32 (&(out_regs->doepdma),
					(uint32_t)ep->dma_addr);
			}
		}
		else {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		}

		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;

		dwc_write_reg32(&out_regs->doepctl, depctl.d32);

	}
}

/**
 * This function does the setup for a data transfer for EP0 and starts
 * the transfer.  For an IN transfer, the packets will be loaded into
 * the appropriate Tx FIFO in the ISR. For OUT transfers, the packets are
 * unloaded from the Rx FIFO in the ISR.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_start_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	gintmsk_data_t intr_mask = { .d32 = 0};
	dwc_otg_dma_desc_t* dma_desc;

	DWC_DEBUGPL(DBG_PCD, "ep%d-%s xfer_len=%d xfer_cnt=%d "
	"xfer_buff=%p start_xfer_buff=%p \n",
	ep->num, (ep->is_in?"IN":"OUT"), ep->xfer_len,
	ep->xfer_count, ep->xfer_buff, ep->start_xfer_buff);

	ep->total_len = ep->xfer_len;

	/* IN endpoint */
	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
		core_if->dev_if->in_ep_regs[0];

		gnptxsts_data_t gtxstatus;

		gtxstatus.d32 =
			dwc_read_reg32(&core_if->core_global_regs->gnptxsts);

		if(core_if->en_multiple_tx_fifo == 0 && gtxstatus.b.nptxqspcavail == 0) {
#ifdef DEBUG
			deptsiz.d32 = dwc_read_reg32(&in_regs->dieptsiz);
			DWC_DEBUGPL(DBG_PCD,"DIEPCTL0=%0x\n",
				dwc_read_reg32(&in_regs->diepctl));
			DWC_DEBUGPL(DBG_PCD, "DIEPTSIZ0=%0x (sz=%d, pcnt=%d)\n",
				deptsiz.d32,
				deptsiz.b.xfersize, deptsiz.b.pktcnt);
			DWC_PRINT("TX Queue or FIFO Full (0x%0x)\n",
				  gtxstatus.d32);
#endif
			return;
		}


		depctl.d32 = dwc_read_reg32(&in_regs->diepctl);
		deptsiz.d32 = dwc_read_reg32(&in_regs->dieptsiz);

		/* Zero Length Packet? */
		if (ep->xfer_len == 0) {
			deptsiz.b.xfersize = 0;
			deptsiz.b.pktcnt = 1;
		}
		else {
			/* Program the transfer size and packet count
			 *	as follows: xfersize = N * maxpacket +
			 *	short_packet pktcnt = N + (short_packet
			 *	exist ? 1 : 0)
			 */
			if (ep->xfer_len > ep->maxpacket) {
				ep->xfer_len = ep->maxpacket;
				deptsiz.b.xfersize = ep->maxpacket;
			}
			else {
				deptsiz.b.xfersize = ep->xfer_len;
			}
			deptsiz.b.pktcnt = 1;

		}
		DWC_DEBUGPL(DBG_PCDV, "IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
			ep->xfer_len,
			deptsiz.b.xfersize, deptsiz.b.pktcnt, deptsiz.d32);

		/* Write the DMA register */
		if (core_if->dma_enable) {
			if(core_if->dma_desc_enable == 0) {
				dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);

				dwc_write_reg32 (&(in_regs->diepdma),
				(uint32_t)ep->dma_addr);
			}
			else {
				dma_desc = core_if->dev_if->in_desc_addr;

				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.sp = (ep->xfer_len == ep->maxpacket) ? 0 : 1;
				dma_desc->status.b.bytes = ep->xfer_len;
				dma_desc->buf = (uint32_t*)ep->dma_addr;
				dma_desc->status.b.bs = BS_HOST_READY;

				/** DIEPDMA0 Register write */
				dwc_write_reg32(&in_regs->diepdma, core_if->dev_if->dma_in_desc_addr);
			}
		}
		else {
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		/**
		 * Enable the Non-Periodic Tx FIFO empty interrupt, the
		 * data will be written into the fifo by the ISR.
		 */
		if (!core_if->dma_enable) {
			if(core_if->en_multiple_tx_fifo == 0) {
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
					intr_mask.d32, intr_mask.d32);
			}
			else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if(ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk |= 1 << ep->num;
					dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
						0, fifoemptymsk);
				}
			}
		}
	}
	else {
		/* OUT endpoint */
		dwc_otg_dev_out_ep_regs_t *out_regs =
			core_if->dev_if->out_ep_regs[ep->num];

		depctl.d32 = dwc_read_reg32(&out_regs->doepctl);
		deptsiz.d32 = dwc_read_reg32(&out_regs->doeptsiz);

		/* Program the transfer size and packet count as follows:
		 *	xfersize = N * (maxpacket + 4 - (maxpacket % 4))
		 *	pktcnt = N											*/
		/* Zero Length Packet */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;

		DWC_DEBUGPL(DBG_PCDV, "len=%d  xfersize=%d pktcnt=%d\n",
			ep->xfer_len,
			deptsiz.b.xfersize, deptsiz.b.pktcnt);

		if (core_if->dma_enable) {
			if(!core_if->dma_desc_enable) {
				dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);

				dwc_write_reg32 (&(out_regs->doepdma),
				 (uint32_t)ep->dma_addr);
			}
			else {
				dma_desc = core_if->dev_if->out_desc_addr;

				/** DMA Descriptor Setup */
				dma_desc->status.b.bs = BS_HOST_BUSY;
				dma_desc->status.b.l = 1;
				dma_desc->status.b.ioc = 1;
				dma_desc->status.b.bytes = ep->maxpacket;
				dma_desc->buf = (uint32_t*)ep->dma_addr;
				dma_desc->status.b.bs = BS_HOST_READY;

				/** DOEPDMA0 Register write */
				dwc_write_reg32(&out_regs->doepdma, core_if->dev_if->dma_out_desc_addr);
			}
		}
		else {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		}

		/* EP enable */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32 (&(out_regs->doepctl), depctl.d32);
	}
}

/**
 * This function continues control IN transfers started by
 * dwc_otg_ep0_start_transfer, when the transfer does not fit in a
 * single packet.  NOTE: The DIEPCTL0/DOEPCTL0 registers only have one
 * bit for the packet count.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP0 data.
 */
void dwc_otg_ep0_continue_transfer(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	depctl_data_t depctl;
	deptsiz0_data_t deptsiz;
	gintmsk_data_t intr_mask = { .d32 = 0};
	dwc_otg_dma_desc_t* dma_desc;

	if (ep->is_in == 1) {
		dwc_otg_dev_in_ep_regs_t *in_regs =
			core_if->dev_if->in_ep_regs[0];
		gnptxsts_data_t tx_status = { .d32 = 0 };

		tx_status.d32 = dwc_read_reg32(&core_if->core_global_regs->gnptxsts);
		/** @todo Should there be check for room in the Tx
		 * Status Queue.  If not remove the code above this comment. */

		depctl.d32 = dwc_read_reg32(&in_regs->diepctl);
		deptsiz.d32 = dwc_read_reg32(&in_regs->dieptsiz);

		/* Program the transfer size and packet count
		 *	as follows: xfersize = N * maxpacket +
		 *	short_packet pktcnt = N + (short_packet
		 *	exist ? 1 : 0)
		 */


		if(core_if->dma_desc_enable == 0) {
			deptsiz.b.xfersize = (ep->total_len - ep->xfer_count) > ep->maxpacket ? ep->maxpacket :
					(ep->total_len - ep->xfer_count);
			deptsiz.b.pktcnt = 1;
			ep->xfer_len += deptsiz.b.xfersize;
			dwc_write_reg32(&in_regs->dieptsiz, deptsiz.d32);
		}
		else {
			ep->xfer_len = (ep->total_len - ep->xfer_count) > ep->maxpacket ? ep->maxpacket :
				(ep->total_len - ep->xfer_count);

			dma_desc = core_if->dev_if->in_desc_addr;

			/** DMA Descriptor Setup */
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			dma_desc->status.b.sp = (ep->xfer_len == ep->maxpacket) ? 0 : 1;
			dma_desc->status.b.bytes = ep->xfer_len;
			dma_desc->buf = (uint32_t*)ep->dma_addr;
			dma_desc->status.b.bs = BS_HOST_READY;

			/** DIEPDMA0 Register write */
			dwc_write_reg32(&in_regs->diepdma, core_if->dev_if->dma_in_desc_addr);
		}


		DWC_DEBUGPL(DBG_PCDV, "IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
			ep->xfer_len,
			deptsiz.b.xfersize, deptsiz.b.pktcnt, deptsiz.d32);

		/* Write the DMA register */
		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
			if(core_if->dma_desc_enable == 0)
				dwc_write_reg32 (&(in_regs->diepdma), (uint32_t)ep->dma_addr);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&in_regs->diepctl, depctl.d32);

		/**
		 * Enable the Non-Periodic Tx FIFO empty interrupt, the
		 * data will be written into the fifo by the ISR.
		 */
		if (!core_if->dma_enable) {
			if(core_if->en_multiple_tx_fifo == 0) {
				/* First clear it from GINTSTS */
				intr_mask.b.nptxfempty = 1;
				dwc_modify_reg32(&core_if->core_global_regs->gintmsk,
					intr_mask.d32, intr_mask.d32);

			}
			else {
				/* Enable the Tx FIFO Empty Interrupt for this EP */
				if(ep->xfer_len > 0) {
					uint32_t fifoemptymsk = 0;
					fifoemptymsk |= 1 << ep->num;
					dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk,
						0, fifoemptymsk);
				}
			}
		}
	}
	else {
		dwc_otg_dev_out_ep_regs_t *out_regs =
			core_if->dev_if->out_ep_regs[0];


		depctl.d32 = dwc_read_reg32(&out_regs->doepctl);
		deptsiz.d32 = dwc_read_reg32(&out_regs->doeptsiz);

		/* Program the transfer size and packet count
		 *	as follows: xfersize = N * maxpacket +
		 *	short_packet pktcnt = N + (short_packet
		 *	exist ? 1 : 0)
		 */
		deptsiz.b.xfersize = ep->maxpacket;
		deptsiz.b.pktcnt = 1;


		if(core_if->dma_desc_enable == 0) {
			dwc_write_reg32(&out_regs->doeptsiz, deptsiz.d32);
		}
		else {
			dma_desc = core_if->dev_if->out_desc_addr;

			/** DMA Descriptor Setup */
			dma_desc->status.b.bs = BS_HOST_BUSY;
			dma_desc->status.b.l = 1;
			dma_desc->status.b.ioc = 1;
			dma_desc->status.b.bytes = ep->maxpacket;
			dma_desc->buf = (uint32_t*)ep->dma_addr;
			dma_desc->status.b.bs = BS_HOST_READY;

			/** DOEPDMA0 Register write */
			dwc_write_reg32(&out_regs->doepdma, core_if->dev_if->dma_out_desc_addr);
		}


		DWC_DEBUGPL(DBG_PCDV, "IN len=%d  xfersize=%d pktcnt=%d [%08x]\n",
			ep->xfer_len,
			deptsiz.b.xfersize, deptsiz.b.pktcnt, deptsiz.d32);

		/* Write the DMA register */
		if (core_if->hwcfg2.b.architecture == DWC_INT_DMA_ARCH) {
			if(core_if->dma_desc_enable == 0)
				dwc_write_reg32 (&(out_regs->doepdma), (uint32_t)ep->dma_addr);
		}

		/* EP enable, IN data in FIFO */
		depctl.b.cnak = 1;
		depctl.b.epena = 1;
		dwc_write_reg32(&out_regs->doepctl, depctl.d32);

	}
}

#ifdef DEBUG
void dump_msg(const u8 *buf, unsigned int length)
{
	unsigned int	start, num, i;
	char		line[52], *p;

	if (length >= 512)
		return;
	start = 0;
	while (length > 0) {
		num = min(length, 16u);
		p = line;
		for (i = 0; i < num; ++i)
		{
			if (i == 8)
				*p++ = ' ';
			sprintf(p, " %02x", buf[i]);
			p += 3;
		}
		*p = 0;
		DWC_PRINT("%6x: %s\n", start, line);
		buf += num;
		start += num;
		length -= num;
	}
}
#else
static inline void dump_msg(const u8 *buf, unsigned int length)
{
}
#endif

/**
 * This function writes a packet into the Tx FIFO associated with the
 * EP.	For non-periodic EPs the non-periodic Tx FIFO is written.  For
 * periodic EPs the periodic Tx FIFO associated with the EP is written
 * with all packets for the next micro-frame.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to write packet for.
 * @param dma Indicates if DMA is being used.
 */
void dwc_otg_ep_write_packet(dwc_otg_core_if_t *core_if, dwc_ep_t *ep, int dma)
{
	/**
	 * The buffer is padded to DWORD on a per packet basis in
	 * slave/dma mode if the MPS is not DWORD aligned.	The last
	 * packet, if short, is also padded to a multiple of DWORD.
	 *
	 * ep->xfer_buff always starts DWORD aligned in memory and is a
	 * multiple of DWORD in length
	 *
	 * ep->xfer_len can be any number of bytes
	 *
	 * ep->xfer_count is a multiple of ep->maxpacket until the last
	 *	packet
	 *
	 * FIFO access is DWORD */

	uint32_t i;
	uint32_t byte_count;
	uint32_t dword_count;
	uint32_t *fifo;
	uint32_t *data_buff = (uint32_t *)ep->xfer_buff;

	DWC_DEBUGPL((DBG_PCDV | DBG_CILV), "%s(%p,%p)\n", __func__, core_if, ep);
	if (ep->xfer_count >= ep->xfer_len) {
			DWC_WARN("%s() No data for EP%d!!!\n", __func__, ep->num);
			return;
	}

	/* Find the byte length of the packet either short packet or MPS */
	if ((ep->xfer_len - ep->xfer_count) < ep->maxpacket) {
		byte_count = ep->xfer_len - ep->xfer_count;
	}
	else {
		byte_count = ep->maxpacket;
	}

	/* Find the DWORD length, padded by extra bytes as neccessary if MPS
	 * is not a multiple of DWORD */
	dword_count =  (byte_count + 3) / 4;

#ifdef VERBOSE
	dump_msg(ep->xfer_buff, byte_count);
#endif

	/**@todo NGS Where are the Periodic Tx FIFO addresses
	 * intialized?	What should this be? */

	fifo = core_if->data_fifo[ep->num];


	DWC_DEBUGPL((DBG_PCDV|DBG_CILV), "fifo=%p buff=%p *p=%08x bc=%d\n", fifo, data_buff, *data_buff, byte_count);

	if (!dma) {
		for (i=0; i<dword_count; i++, data_buff++)
		{
			dwc_write_reg32(fifo, *data_buff);
		}
	}

	ep->xfer_count += byte_count;
	ep->xfer_buff += byte_count;
	ep->dma_addr += byte_count;
}

/**
 * Set the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to set the stall on.
 */
void dwc_otg_ep_set_stall(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;

	DWC_DEBUGPL(DBG_PCD, "%s ep%d-%s\n", __func__, ep->num,
		(ep->is_in?"IN":"OUT"));


	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
		depctl.d32 = dwc_read_reg32(depctl_addr);

		/* set the disable and stall bits */
		if (depctl.b.epena) {
			depctl.b.epdis = 1;
		}
		depctl.b.stall = 1;
		dwc_write_reg32(depctl_addr, depctl.d32);
	}
	else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
		depctl.d32 = dwc_read_reg32(depctl_addr);

		/* set the stall bit */
		depctl.b.stall = 1;
		dwc_write_reg32(depctl_addr, depctl.d32);
	}

	DWC_DEBUGPL(DBG_PCD,"DEPCTL=%0x\n",dwc_read_reg32(depctl_addr));

	return;
}

/**
 * Clear the EP STALL.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param ep The EP to clear stall from.
 */
void dwc_otg_ep_clear_stall(dwc_otg_core_if_t *core_if, dwc_ep_t *ep)
{
	depctl_data_t depctl;
	volatile uint32_t *depctl_addr;

	DWC_DEBUGPL(DBG_PCD, "%s ep%d-%s\n", __func__, ep->num,
		(ep->is_in?"IN":"OUT"));

	if (ep->is_in == 1) {
		depctl_addr = &(core_if->dev_if->in_ep_regs[ep->num]->diepctl);
	}
	else {
		depctl_addr = &(core_if->dev_if->out_ep_regs[ep->num]->doepctl);
	}

	depctl.d32 = dwc_read_reg32(depctl_addr);

	/* clear the stall bits */
	depctl.b.stall = 0;

	/*
	 * USB Spec 9.4.5: For endpoints using data toggle, regardless
	 * of whether an endpoint has the Halt feature set, a
	 * ClearFeature(ENDPOINT_HALT) request always results in the
	 * data toggle being reinitialized to DATA0.
	 */
	if (ep->type == DWC_OTG_EP_TYPE_INTR ||
		ep->type == DWC_OTG_EP_TYPE_BULK) {
		depctl.b.setd0pid = 1; /* DATA0 */
	}

	dwc_write_reg32(depctl_addr, depctl.d32);
	DWC_DEBUGPL(DBG_PCD,"DEPCTL=%0x\n",dwc_read_reg32(depctl_addr));
	return;
}

/**
 * This function reads a packet from the Rx FIFO into the destination
 * buffer.	To read SETUP data use dwc_otg_read_setup_packet.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param dest	  Destination buffer for the packet.
 * @param bytes  Number of bytes to copy to the destination.
 */
void dwc_otg_read_packet(dwc_otg_core_if_t *core_if,
			 uint8_t *dest,
			 uint16_t bytes)
{
	int i;
	int word_count = (bytes + 3) / 4;

	volatile uint32_t *fifo = core_if->data_fifo[0];
	uint32_t *data_buff = (uint32_t *)dest;

	/**
	 * @todo Account for the case where _dest is not dword aligned. This
	 * requires reading data from the FIFO into a uint32_t temp buffer,
	 * then moving it into the data buffer.
	 */

	DWC_DEBUGPL((DBG_PCDV | DBG_CILV), "%s(%p,%p,%d)\n", __func__,
					core_if, dest, bytes);

	for (i=0; i<word_count; i++, data_buff++)
	{
		*data_buff = dwc_read_reg32(fifo);
	}

	return;
}



/**
 * This functions reads the device registers and prints them
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_dev_registers(dwc_otg_core_if_t *core_if)
{
	int i;
	volatile uint32_t *addr;

	DWC_PRINT("Device Global Registers\n");
	addr=&core_if->dev_if->dev_global_regs->dcfg;
	DWC_PRINT("DCFG		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->dctl;
	DWC_PRINT("DCTL		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->dsts;
	DWC_PRINT("DSTS		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->diepmsk;
	DWC_PRINT("DIEPMSK	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->doepmsk;
	DWC_PRINT("DOEPMSK	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->daint;
	DWC_PRINT("DAINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->dev_if->dev_global_regs->dtknqr1;
	DWC_PRINT("DTKNQR1	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	if (core_if->hwcfg2.b.dev_token_q_depth > 6) {
		addr=&core_if->dev_if->dev_global_regs->dtknqr2;
		DWC_PRINT("DTKNQR2	 @0x%08X : 0x%08X\n",
		  (uint32_t)addr,dwc_read_reg32(addr));
	}

	addr=&core_if->dev_if->dev_global_regs->dvbusdis;
	DWC_PRINT("DVBUSID	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));

	addr=&core_if->dev_if->dev_global_regs->dvbuspulse;
	DWC_PRINT("DVBUSPULSE	@0x%08X : 0x%08X\n",
				  (uint32_t)addr,dwc_read_reg32(addr));

	if (core_if->hwcfg2.b.dev_token_q_depth > 14) {
		addr=&core_if->dev_if->dev_global_regs->dtknqr3_dthrctl;
		DWC_PRINT("DTKNQR3_DTHRCTL	 @0x%08X : 0x%08X\n",
		  (uint32_t)addr, dwc_read_reg32(addr));
	}

	if (core_if->hwcfg2.b.dev_token_q_depth > 22) {
		addr=&core_if->dev_if->dev_global_regs->dtknqr4_fifoemptymsk;
		DWC_PRINT("DTKNQR4	 @0x%08X : 0x%08X\n",
				  (uint32_t)addr, dwc_read_reg32(addr));
	}

	for (i=0; i<= core_if->dev_if->num_in_eps; i++)
	{
		DWC_PRINT("Device IN EP %d Registers\n", i);
		addr=&core_if->dev_if->in_ep_regs[i]->diepctl;
		DWC_PRINT("DIEPCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->in_ep_regs[i]->diepint;
		DWC_PRINT("DIEPINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->in_ep_regs[i]->dieptsiz;
		DWC_PRINT("DIETSIZ	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->in_ep_regs[i]->diepdma;
		DWC_PRINT("DIEPDMA	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->in_ep_regs[i]->dtxfsts;
		DWC_PRINT("DTXFSTS	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->in_ep_regs[i]->diepdmab;
		DWC_PRINT("DIEPDMAB	 @0x%08X : 0x%08X\n",(uint32_t)addr,0/*dwc_read_reg32(addr)*/);
	}


	for (i=0; i<= core_if->dev_if->num_out_eps; i++)
	{
		DWC_PRINT("Device OUT EP %d Registers\n", i);
		addr=&core_if->dev_if->out_ep_regs[i]->doepctl;
		DWC_PRINT("DOEPCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->out_ep_regs[i]->doepfn;
		DWC_PRINT("DOEPFN	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->out_ep_regs[i]->doepint;
		DWC_PRINT("DOEPINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->out_ep_regs[i]->doeptsiz;
		DWC_PRINT("DOETSIZ	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->out_ep_regs[i]->doepdma;
		DWC_PRINT("DOEPDMA	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->dev_if->out_ep_regs[i]->doepdmab;
		DWC_PRINT("DOEPDMAB	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));

	}
	return;
}

/**
 * This functions reads the SPRAM and prints its content
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_spram(dwc_otg_core_if_t *core_if)
{
	volatile uint8_t *addr, *start_addr, *end_addr;

	DWC_PRINT("SPRAM Data:\n");
	start_addr = (void*)core_if->core_global_regs;
	DWC_PRINT("Base Address: 0x%8X\n", (uint32_t)start_addr);
	start_addr += 0x00028000;
	end_addr=(void*)core_if->core_global_regs;
	end_addr += 0x000280e0;

	for(addr = start_addr; addr < end_addr; addr+=16)
	{
		DWC_PRINT("0x%8X:\t%2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X %2X\n", (uint32_t)addr,
			addr[0],
			addr[1],
			addr[2],
			addr[3],
			addr[4],
			addr[5],
			addr[6],
			addr[7],
			addr[8],
			addr[9],
			addr[10],
			addr[11],
			addr[12],
			addr[13],
			addr[14],
			addr[15]
			);
	}

	return;
}
/**
 * This function reads the host registers and prints them
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_host_registers(dwc_otg_core_if_t *core_if)
{
	int i;
	volatile uint32_t *addr;

	DWC_PRINT("Host Global Registers\n");
	addr=&core_if->host_if->host_global_regs->hcfg;
	DWC_PRINT("HCFG		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->host_if->host_global_regs->hfir;
	DWC_PRINT("HFIR		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->host_if->host_global_regs->hfnum;
	DWC_PRINT("HFNUM	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->host_if->host_global_regs->hptxsts;
	DWC_PRINT("HPTXSTS	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->host_if->host_global_regs->haint;
	DWC_PRINT("HAINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->host_if->host_global_regs->haintmsk;
	DWC_PRINT("HAINTMSK	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=core_if->host_if->hprt0;
	DWC_PRINT("HPRT0	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));

	for (i=0; i<core_if->core_params->host_channels; i++)
	{
		DWC_PRINT("Host Channel %d Specific Registers\n", i);
		addr=&core_if->host_if->hc_regs[i]->hcchar;
		DWC_PRINT("HCCHAR	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->host_if->hc_regs[i]->hcsplt;
		DWC_PRINT("HCSPLT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->host_if->hc_regs[i]->hcint;
		DWC_PRINT("HCINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->host_if->hc_regs[i]->hcintmsk;
		DWC_PRINT("HCINTMSK	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->host_if->hc_regs[i]->hctsiz;
		DWC_PRINT("HCTSIZ	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
		addr=&core_if->host_if->hc_regs[i]->hcdma;
		DWC_PRINT("HCDMA	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	}
	return;
}

/**
 * This function reads the core global registers and prints them
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_dump_global_registers(dwc_otg_core_if_t *core_if)
{
	int i;
	volatile uint32_t *addr;

	DWC_PRINT("Core Global Registers\n");
	addr=&core_if->core_global_regs->gotgctl;
	DWC_PRINT("GOTGCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gotgint;
	DWC_PRINT("GOTGINT	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gahbcfg;
	DWC_PRINT("GAHBCFG	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gusbcfg;
	DWC_PRINT("GUSBCFG	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->grstctl;
	DWC_PRINT("GRSTCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gintsts;
	DWC_PRINT("GINTSTS	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gintmsk;
	DWC_PRINT("GINTMSK	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->grxstsr;
	DWC_PRINT("GRXSTSR	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	//addr=&core_if->core_global_regs->grxstsp;
	//DWC_PRINT("GRXSTSP   @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->grxfsiz;
	DWC_PRINT("GRXFSIZ	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gnptxfsiz;
	DWC_PRINT("GNPTXFSIZ @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gnptxsts;
	DWC_PRINT("GNPTXSTS	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gi2cctl;
	DWC_PRINT("GI2CCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gpvndctl;
	DWC_PRINT("GPVNDCTL	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->ggpio;
	DWC_PRINT("GGPIO	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->guid;
	DWC_PRINT("GUID		 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->gsnpsid;
	DWC_PRINT("GSNPSID	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->ghwcfg1;
	DWC_PRINT("GHWCFG1	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->ghwcfg2;
	DWC_PRINT("GHWCFG2	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->ghwcfg3;
	DWC_PRINT("GHWCFG3	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->ghwcfg4;
	DWC_PRINT("GHWCFG4	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));
	addr=&core_if->core_global_regs->hptxfsiz;
	DWC_PRINT("HPTXFSIZ	 @0x%08X : 0x%08X\n",(uint32_t)addr,dwc_read_reg32(addr));

	for (i=0; i<core_if->hwcfg4.b.num_dev_perio_in_ep; i++)
	{
		addr=&core_if->core_global_regs->dptxfsiz_dieptxf[i];
		DWC_PRINT("DPTXFSIZ[%d] @0x%08X : 0x%08X\n",i,(uint32_t)addr,dwc_read_reg32(addr));
	}
}

/**
 * Flush a Tx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 * @param num Tx FIFO to flush.
 */
void dwc_otg_flush_tx_fifo(dwc_otg_core_if_t *core_if,
					   const int num)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset = { .d32 = 0};
	int count = 0;

	DWC_DEBUGPL((DBG_CIL|DBG_PCDV), "Flush Tx FIFO %d\n", num);

	greset.b.txfflsh = 1;
	greset.b.txfnum = num;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			DWC_WARN("%s() HANG! GRSTCTL=%0x GNPTXSTS=0x%08x\n",
					  __func__, greset.d32,
			dwc_read_reg32(&global_regs->gnptxsts));
			break;
		}
	}
	while (greset.b.txfflsh == 1);

	/* Wait for 3 PHY Clocks*/
	UDELAY(1);
}

/**
 * Flush Rx FIFO.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
void dwc_otg_flush_rx_fifo(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset = { .d32 = 0};
	int count = 0;

	DWC_DEBUGPL((DBG_CIL|DBG_PCDV), "%s\n", __func__);

	greset.b.rxfflsh = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 10000) {
			DWC_WARN("%s() HANG! GRSTCTL=%0x\n", __func__,
				greset.d32);
			break;
		}
	}
	while (greset.b.rxfflsh == 1);

	/* Wait for 3 PHY Clocks*/
	UDELAY(1);
}

/**
 * Do core a soft reset of the core.  Be careful with this because it
 * resets all the internal state machines of the core.
 */
void dwc_otg_core_reset(dwc_otg_core_if_t *core_if)
{
	dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	volatile grstctl_t greset = { .d32 = 0};
	int count = 0;

	DWC_DEBUGPL(DBG_CILV, "%s\n", __func__);

	/* Core Soft Reset */
	count = 0;
	greset.b.csftrst = 1;
	dwc_write_reg32(&global_regs->grstctl, greset.d32);

	/* Wait for core soft reset finished */
	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 100000) {
			DWC_WARN("%s() HANG! Soft Reset GRSTCTL=%0x\n", __func__,
				 greset.d32);
			break;
		}
	} while (greset.b.csftrst == 1);

	/* Wait for AHB master IDLE state. */
	do {
		greset.d32 = dwc_read_reg32(&global_regs->grstctl);
		if (++count > 100000) {
			DWC_WARN("%s() HANG! AHB Idle GRSTCTL=%0x\n", __func__,
				 greset.d32);
			return;
		}
	} while (greset.b.ahbidle == 0);

	/* Wait for 3 PHY Clocks */
	msleep(100);
}
