/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_hcd.c $
 * $Revision: #19 $
 * $Date: 2008/03/13 $
 * $Change: 998338 $
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
/**
 * @file
 *
 * This file contains the implementation of the HCD. In Linux, the HCD
 * implements the hc_driver API.
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/errno.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/string.h>
#include <linux/dma-mapping.h>
#include <linux/version.h>
#include <linux/platform_device.h>

#include <mach/gpio.h>
#include <mach/gpio_parrot.h>

#include <asm/fiq.h>

#include "dwc_otg_driver.h"
#include "dwc_otg_hcd.h"
#include "dwc_otg_regs.h"


static const char dwc_otg_hcd_name [] = "dwc_otg_hcd";

static const struct hc_driver dwc_otg_hc_driver = {

	.description =		dwc_otg_hcd_name,
	.product_desc = 	"DWC OTG Controller",
	.hcd_priv_size = 	sizeof(dwc_otg_hcd_t),

	.irq =			dwc_otg_hcd_irq,
	.flags =		HCD_MEMORY | HCD_USB2,

	.start =		dwc_otg_hcd_start,
	.stop =			dwc_otg_hcd_stop,

	.urb_enqueue =		dwc_otg_hcd_urb_enqueue,
	.urb_dequeue =		dwc_otg_hcd_urb_dequeue,
	.endpoint_disable =	dwc_otg_hcd_endpoint_disable,

	.get_frame_number =	dwc_otg_hcd_get_frame_number,

	.hub_status_data =	dwc_otg_hcd_hub_status_data,
	.hub_control =		dwc_otg_hcd_hub_control,
};


static void del_xfer_timers(dwc_otg_hcd_t *_hcd)
{
	int i;
	int num_channels = _hcd->core_if->core_params->host_channels;
	for (i = 0; i < num_channels; i++) {
		del_timer(&_hcd->core_if->hc_xfer_timer[i]);
	}
}

static void del_timers(dwc_otg_hcd_t *_hcd)
{
	del_xfer_timers(_hcd);
}

/**
 * Processes all the URBs in a single list of QHs. Completes them with
 * -ETIMEDOUT and frees the QTD.
 */
static void kill_urbs_in_qh_list(dwc_otg_hcd_t *_hcd, struct list_head *_qh_list)
{
	struct list_head	*qh_item;
	dwc_otg_qh_t		*qh;
	struct list_head	*qtd_item;
	dwc_otg_qtd_t		*qtd;

	list_for_each(qh_item, _qh_list) {
		qh = list_entry(qh_item, dwc_otg_qh_t, qh_list_entry);
		for (qtd_item = qh->qtd_list.next;
		     qtd_item != &qh->qtd_list;
		     qtd_item = qh->qtd_list.next) {
			qtd = list_entry(qtd_item, dwc_otg_qtd_t, qtd_list_entry);
			if (qtd->urb != NULL) {
				dwc_otg_hcd_complete_urb(_hcd, qtd->urb,
							 -ETIMEDOUT);
			}
			dwc_otg_hcd_qtd_remove_and_free(qtd);
		}
	}
}


/**
 * Initializes the HCD. This function allocates memory for and initializes the
 * static parts of the usb_hcd and dwc_otg_hcd structures. It also registers the
 * USB bus with the core and calls the hc_driver->start() function. It returns
 * a negative error on failure.
 */
int dwc_otg_hcd_init(struct platform_device *pdev)
{
	struct usb_hcd *hcd = NULL;
	dwc_otg_hcd_t *dwc_otg_hcd = NULL;
	dwc_otg_device_t *otg_dev = platform_get_drvdata(pdev);
	dwc_otg_core_params_t *params = otg_dev->core_if->core_params;
	int num_channels, irq;
	int i;
	dwc_hc_t *channel;

	int retval = 0;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD INIT\n");

	/* Set device flags indicating whether the HCD supports DMA */
	if (otg_dev->core_if->dma_enable) {
		DWC_PRINT("Using DMA mode\n");
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

		if(otg_dev->core_if->dma_desc_enable){
			DWC_PRINT("Device using Descriptor DMA mode\n");
		} else {
			DWC_PRINT("Device using Buffer DMA mode\n");
		}
	} else {
		DWC_PRINT("Using Slave mode\n");
		pdev->dev.dma_mask = (void *)0;
		pdev->dev.coherent_dma_mask = 0;
	}

	/*
	 * Allocate memory for the base HCD plus the DWC OTG HCD.
	 * Initialize the base HCD.
	 */
	hcd = usb_create_hcd(&dwc_otg_hc_driver, &pdev->dev, pdev->dev.bus_id);
	if (hcd == NULL) {
		retval = -ENOMEM;
		goto error1;
	}

	hcd->regs = otg_dev->base;
	hcd->self.otg_port = 0;

	/* Initialize the DWC OTG HCD */
	dwc_otg_hcd = hcd_to_dwc_otg_hcd(hcd);
	dwc_otg_hcd->core_if = otg_dev->core_if;
	otg_dev->hcd = dwc_otg_hcd;

	/* Initialize the non-periodic schedule */
	INIT_LIST_HEAD(&dwc_otg_hcd->non_periodic_sched_inactive);
	INIT_LIST_HEAD(&dwc_otg_hcd->non_periodic_sched_active);

	/* Initialize the periodic schedule */
	INIT_LIST_HEAD(&dwc_otg_hcd->periodic_sched_inactive);
	INIT_LIST_HEAD(&dwc_otg_hcd->periodic_sched_ready);
	INIT_LIST_HEAD(&dwc_otg_hcd->periodic_sched_assigned);
	INIT_LIST_HEAD(&dwc_otg_hcd->periodic_sched_queued);

	/*
	 * Create a host channel descriptor for each host channel implemented
	 * in the controller. Initialize the channel descriptor array.
	 */
	INIT_LIST_HEAD(&dwc_otg_hcd->free_hc_list);
	num_channels = dwc_otg_hcd->core_if->core_params->host_channels;
	for (i = 0; i < num_channels; i++) {
		channel = kmalloc(sizeof(dwc_hc_t), GFP_KERNEL);
		if (channel == NULL) {
			retval = -ENOMEM;
			DWC_ERROR("%s: host channel allocation failed\n", __func__);
			goto error2;
		}
		memset(channel, 0, sizeof(dwc_hc_t));
		channel->hc_num = i;
		dwc_otg_hcd->hc_ptr_array[i] = channel;

		init_timer(&dwc_otg_hcd->core_if->hc_xfer_timer[i]);

		DWC_DEBUGPL(DBG_HCDV, "HCD Added channel #%d, hc=%p\n", i, channel);
	}

	/*
	 * Finish generic HCD initialization and start the HCD. This function
	 * allocates the DMA buffer pool, registers the USB bus, requests the
	 * IRQ line, and calls dwc_otg_hcd_start method.
	 */
	if (params->fiq_enable) {
		/* irq redirection in fiq mode */
		retval = usb_add_hcd(hcd, FIQ_MODE_USB_IRQ, IRQF_SHARED);
	} else {
		retval = usb_add_hcd(hcd, platform_get_irq(pdev, 0), IRQF_SHARED);
	}
	if (retval < 0) {
		goto error2;
	}
	/* race against rh_registered and usb_hcd_poll_rh_status(usb_new_device) */
	usb_hcd_poll_rh_status(hcd);

	/*
	 * Allocate space for storing data on status transactions. Normally no
	 * data is sent, but this space acts as a bit bucket. This must be
	 * done after usb_add_hcd since that function allocates the DMA buffer
	 * pool.
	 */
	if (otg_dev->core_if->dma_enable) {
		dwc_otg_hcd->status_buf =
			dma_alloc_coherent(&pdev->dev,
					   DWC_OTG_HCD_STATUS_BUF_SIZE,
					   &dwc_otg_hcd->status_buf_dma,
					   GFP_KERNEL | GFP_DMA);
	} else {
		dwc_otg_hcd->status_buf = kmalloc(DWC_OTG_HCD_STATUS_BUF_SIZE,
						  GFP_KERNEL);
	}
	if (dwc_otg_hcd->status_buf == NULL) {
		retval = -ENOMEM;
		DWC_ERROR("%s: status_buf allocation failed\n", __func__);
		goto error3;
	}

	/* overcurrent IRQ init */
	if (dwc_otg_hcd->core_if->core_params->overcurrent_pin > 0) {
		gpio_direction_input(dwc_otg_hcd->core_if->core_params->overcurrent_pin);

		irq = gpio_interrupt_register(dwc_otg_hcd->core_if->core_params->overcurrent_pin,
					      GPIO_IRQ_BOTH, GPIO_DEBOUNCE_NONE);
		if (irq < 0) {
			retval = -EIO;
			goto error3;
		}

		retval = request_irq(irq, dwc_otg_hcd_overcurrent_irq,
				     IRQF_TRIGGER_NONE, hcd->irq_descr, dwc_otg_hcd);
		if (retval != 0) {
			DWC_ERROR("request of irq%d failed\n", irq);
			goto error3;
		}
	};

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD Initialized HCD, bus=%s, usbbus=%d\n",
		    pdev->dev.bus_id, hcd->self.busnum);

	return 0;

	/* Error conditions */
 error3:
	usb_remove_hcd(hcd);
 error2:
	dwc_otg_hcd_free(hcd);
	usb_put_hcd(hcd);
 error1:
	return retval;
}

/**
 * Removes the HCD.
 * Frees memory and resources associated with the HCD and deregisters the bus.
 */
void dwc_otg_hcd_remove(struct platform_device *pdev)
{
	dwc_otg_device_t *otg_dev = platform_get_drvdata(pdev);
	dwc_otg_hcd_t *dwc_otg_hcd;
	struct usb_hcd *hcd;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD REMOVE\n");

	if (!otg_dev) {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev NULL!\n", __func__);
		return;
	}

	dwc_otg_hcd = otg_dev->hcd;
	if (!dwc_otg_hcd) {
		DWC_DEBUGPL(DBG_ANY, "%s: otg_dev->hcd NULL!\n", __func__);
		return;
	}

	hcd = dwc_otg_hcd_to_hcd(dwc_otg_hcd);
	if (!hcd) {
		DWC_DEBUGPL(DBG_ANY, "%s: dwc_otg_hcd_to_hcd(dwc_otg_hcd) NULL!\n", __func__);
		return;
	}

	if (dwc_otg_hcd->core_if->core_params->overcurrent_pin > 0) {
		free_irq(gpio_to_irq(dwc_otg_hcd->core_if->core_params->overcurrent_pin), dwc_otg_hcd);
		gpio_interrupt_unregister(dwc_otg_hcd->core_if->core_params->overcurrent_pin);
	}
	/* Turn off all interrupts */
	dwc_write_reg32(&dwc_otg_hcd->core_if->core_global_regs->gintmsk, 0);
	dwc_modify_reg32(&dwc_otg_hcd->core_if->core_global_regs->gahbcfg, 1, 0);

	usb_remove_hcd(hcd);
	dwc_otg_hcd_free(hcd);
	usb_put_hcd(hcd);

	return;
}


/* =========================================================================
 *  Linux HC Driver Functions
 * ========================================================================= */

/**
 * Initializes dynamic portions of the DWC_otg HCD state.
 */
static void hcd_reinit(dwc_otg_hcd_t *_hcd)
{
	struct list_head *item;
	int num_channels;
	int i;
	dwc_hc_t *channel;

	_hcd->flags.d32 = 0;

	_hcd->non_periodic_qh_ptr = &_hcd->non_periodic_sched_active;
	_hcd->non_periodic_channels = 0;
	_hcd->periodic_channels = 0;

	/*
	 * Put all channels in the free channel list and clean up channel
	 * states.
	 */
	item = _hcd->free_hc_list.next;
	while (item != &_hcd->free_hc_list) {
		list_del(item);
		item = _hcd->free_hc_list.next;
	}
	num_channels = _hcd->core_if->core_params->host_channels;
	for (i = 0; i < num_channels; i++) {
		channel = _hcd->hc_ptr_array[i];
		list_add_tail(&channel->hc_list_entry, &_hcd->free_hc_list);
		dwc_otg_hc_cleanup(_hcd->core_if, channel);
	}

	/* Initialize the DWC core for host mode operation. */
	dwc_otg_core_host_init(_hcd->core_if);
}

/**
 * Initializes the DWC_otg controller and its root hub and prepares it for host
 * mode operation. Activates the root port. Returns 0 on success and a negative
 * error code on failure.
 */
int dwc_otg_hcd_start(struct usb_hcd *_hcd)
{
        dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
        dwc_otg_core_if_t *core_if = dwc_otg_hcd->core_if;
  	struct usb_bus *bus;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD START\n");

	bus = hcd_to_bus(_hcd);

	/* Initialize the bus state. If the core is in Device Mode
	 * HALT the USB bus and return. */
	if (dwc_otg_is_device_mode (core_if)) {
		_hcd->state = HC_STATE_RUNNING;
		return 0;
	}
	_hcd->state = HC_STATE_RUNNING;

	/* Initialize and connect root hub if one is not already attached */
	if (bus->root_hub) {
		DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD Has Root Hub\n");
                /* Inform the HUB driver to resume. */
		usb_hcd_resume_root_hub(_hcd);
        }
	else {
		DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD Does Not Have Root Hub\n");
	}

	hcd_reinit(dwc_otg_hcd);

	return 0;
}

static void qh_list_free(dwc_otg_hcd_t *_hcd, struct list_head *_qh_list)
{
	struct list_head *item;
	dwc_otg_qh_t *qh;

	if (_qh_list->next == NULL) {
		/* The list hasn't been initialized yet. */
		return;
	}

	/* Ensure there are no QTDs or URBs left. */
	kill_urbs_in_qh_list(_hcd, _qh_list);

	for (item = _qh_list->next; item != _qh_list; item = _qh_list->next) {
		qh = list_entry(item, dwc_otg_qh_t, qh_list_entry);
		dwc_otg_hcd_qh_remove_and_free(_hcd, qh);
	}
}

/**
 * Halts the DWC_otg host mode operations in a clean manner. USB transfers are
 * stopped.
 */
void dwc_otg_hcd_stop(struct usb_hcd *_hcd)
{
        dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
	hprt0_data_t hprt0 = { .d32=0 };

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD STOP\n");

	/* Turn off all host-specific interrupts. */
	dwc_otg_disable_host_interrupts( dwc_otg_hcd->core_if );

	/*
	 * The root hub should be disconnected before this function is called.
	 * The disconnect will clear the QTD lists (via ..._hcd_urb_dequeue)
	 * and the QH lists (via ..._hcd_endpoint_disable).
	 */

	/* Turn off the vbus power */
	DWC_PRINT("PortPower off\n");
	hprt0.b.prtpwr = 0;
	dwc_write_reg32(dwc_otg_hcd->core_if->host_if->hprt0, hprt0.d32);

	return;
}


/**
 * Returns the current frame number.
 */
int dwc_otg_hcd_get_frame_number(struct usb_hcd *_hcd)
{
        dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(_hcd);
	hfnum_data_t hfnum;

	hfnum.d32 = dwc_read_reg32(&dwc_otg_hcd->core_if->
				   host_if->host_global_regs->hfnum);

#ifdef DEBUG_SOF
	DWC_DEBUGPL(DBG_HCDV, "DWC OTG HCD GET FRAME NUMBER %d\n", hfnum.b.frnum);
#endif
	return hfnum.b.frnum;
}

/**
 * Frees secondary storage associated with the dwc_otg_hcd structure contained
 * in the struct usb_hcd field.
 */
void dwc_otg_hcd_free(struct usb_hcd *_hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(_hcd);
	int i;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD FREE\n");

	del_timers(dwc_otg_hcd);

	/* Free memory for QH/QTD lists */
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->non_periodic_sched_inactive);
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->non_periodic_sched_active);
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->periodic_sched_inactive);
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->periodic_sched_ready);
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->periodic_sched_assigned);
	qh_list_free(dwc_otg_hcd, &dwc_otg_hcd->periodic_sched_queued);

	/* Free memory for the host channels. */
	for (i = 0; i < MAX_EPS_CHANNELS; i++) {
		dwc_hc_t *hc = dwc_otg_hcd->hc_ptr_array[i];
		if (hc != NULL) {
			DWC_DEBUGPL(DBG_HCDV, "HCD Free channel #%i, hc=%p\n", i, hc);
			kfree(hc);
		}
	}

	if (dwc_otg_hcd->core_if->dma_enable) {
		if (dwc_otg_hcd->status_buf_dma) {
			dma_free_coherent(_hcd->self.controller,
					  DWC_OTG_HCD_STATUS_BUF_SIZE,
					  dwc_otg_hcd->status_buf,
					  dwc_otg_hcd->status_buf_dma);
		}
	} else if (dwc_otg_hcd->status_buf != NULL) {
		kfree(dwc_otg_hcd->status_buf);
	}

	return;
}


#ifdef DEBUG
static void dump_urb_info(struct urb *_urb, char* _fn_name)
{
	DWC_PRINT("%s, urb %p\n", _fn_name, _urb);
	DWC_PRINT("  Device address: %d\n", usb_pipedevice(_urb->pipe));
	DWC_PRINT("  Endpoint: %d, %s\n", usb_pipeendpoint(_urb->pipe),
		  (usb_pipein(_urb->pipe) ? "IN" : "OUT"));
	DWC_PRINT("  Endpoint type: %s\n",
		  ({char *pipetype;
		  switch (usb_pipetype(_urb->pipe)) {
		  case PIPE_CONTROL: pipetype = "CONTROL"; break;
		  case PIPE_BULK: pipetype = "BULK"; break;
		  case PIPE_INTERRUPT: pipetype = "INTERRUPT"; break;
		  case PIPE_ISOCHRONOUS: pipetype = "ISOCHRONOUS"; break;
		  default: pipetype = "UNKNOWN"; break;
		  }; pipetype;}));
	DWC_PRINT("  Speed: %s\n",
		  ({char *speed;
		  switch (_urb->dev->speed) {
		  case USB_SPEED_HIGH: speed = "HIGH"; break;
		  case USB_SPEED_FULL: speed = "FULL"; break;
		  case USB_SPEED_LOW: speed = "LOW"; break;
		  default: speed = "UNKNOWN"; break;
		  }; speed;}));
	DWC_PRINT("  Max packet size: %d\n",
		  usb_maxpacket(_urb->dev, _urb->pipe, usb_pipeout(_urb->pipe)));
	DWC_PRINT("  Data buffer length: %d\n", _urb->transfer_buffer_length);
	DWC_PRINT("  Transfer buffer: %p, Transfer DMA: %p\n",
		  _urb->transfer_buffer, (void *)_urb->transfer_dma);
	DWC_PRINT("  Setup buffer: %p, Setup DMA: %p\n",
		  _urb->setup_packet, (void *)_urb->setup_dma);
	DWC_PRINT("  Interval: %d\n", _urb->interval);
	if (usb_pipetype(_urb->pipe) == PIPE_ISOCHRONOUS) {
		int i;
		for (i = 0; i < _urb->number_of_packets;  i++) {
			DWC_PRINT("  ISO Desc %d:\n", i);
			DWC_PRINT("    offset: %d, length %d\n",
				  _urb->iso_frame_desc[i].offset,
				  _urb->iso_frame_desc[i].length);
		}
	}
}

static void dump_channel_info(dwc_otg_hcd_t *_hcd,
			      dwc_otg_qh_t *qh)
{
	if (qh->channel != NULL) {
		dwc_hc_t *hc = qh->channel;
		struct list_head *item;
		dwc_otg_qh_t *qh_item;
		int num_channels = _hcd->core_if->core_params->host_channels;
		int i;

		dwc_otg_hc_regs_t *hc_regs;
		hcchar_data_t 	hcchar;
		hcsplt_data_t	hcsplt;
		hctsiz_data_t 	hctsiz;
		uint32_t	hcdma;

		hc_regs = _hcd->core_if->host_if->hc_regs[hc->hc_num];
		hcchar.d32 = dwc_read_reg32(&hc_regs->hcchar);
		hcsplt.d32 = dwc_read_reg32(&hc_regs->hcsplt);
		hctsiz.d32 = dwc_read_reg32(&hc_regs->hctsiz);
		hcdma = dwc_read_reg32(&hc_regs->hcdma);

		DWC_PRINT("  Assigned to channel %p:\n", hc);
		DWC_PRINT("    hcchar 0x%08x, hcsplt 0x%08x\n", hcchar.d32, hcsplt.d32);
		DWC_PRINT("    hctsiz 0x%08x, hcdma 0x%08x\n", hctsiz.d32, hcdma);
		DWC_PRINT("    dev_addr: %d, ep_num: %d, ep_is_in: %d\n",
			  hc->dev_addr, hc->ep_num, hc->ep_is_in);
		DWC_PRINT("    ep_type: %d\n", hc->ep_type);
		DWC_PRINT("    max_packet: %d\n", hc->max_packet);
		DWC_PRINT("    data_pid_start: %d\n", hc->data_pid_start);
		DWC_PRINT("    xfer_started: %d\n", hc->xfer_started);
		DWC_PRINT("    halt_status: %d\n", hc->halt_status);
		DWC_PRINT("    xfer_buff: %p\n", hc->xfer_buff);
		DWC_PRINT("    xfer_len: %d\n", hc->xfer_len);
		DWC_PRINT("    qh: %p\n", hc->qh);
		DWC_PRINT("  NP inactive sched:\n");
		list_for_each(item, &_hcd->non_periodic_sched_inactive) {
			qh_item = list_entry(item, dwc_otg_qh_t, qh_list_entry);
			DWC_PRINT("    %p\n", qh_item);
		}
		DWC_PRINT("  NP active sched:\n");
		list_for_each(item, &_hcd->non_periodic_sched_active) {
			qh_item = list_entry(item, dwc_otg_qh_t, qh_list_entry);
			DWC_PRINT("    %p\n", qh_item);
		}
		DWC_PRINT("  Channels: \n");
		for (i = 0; i < num_channels; i++) {
			dwc_hc_t *hc = _hcd->hc_ptr_array[i];
			DWC_PRINT("    %2d: %p\n", i, hc);
		}
	}
}
#endif

/**
 * Starts processing a USB transfer request specified by a USB Request Block
 * (URB). mem_flags indicates the type of memory allocation to use while
 * processing this URB.
 */
int dwc_otg_hcd_urb_enqueue(struct usb_hcd *_hcd,
			    struct urb *_urb,
			    gfp_t _mem_flags)
{
	int retval = 0;
	unsigned long flags;
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
	dwc_otg_qtd_t *qtd;

	local_irq_save(flags);

#ifdef DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		dump_urb_info(_urb, "dwc_otg_hcd_urb_enqueue");
	}
#endif

	retval = usb_hcd_link_urb_to_ep(_hcd, _urb);
	if (retval)
		goto error_not_linked;

	qtd = dwc_otg_hcd_qtd_create (_urb, _mem_flags);
	if (qtd == NULL) {
		DWC_ERROR("DWC OTG HCD URB Enqueue failed creating QTD\n");
		retval = -ENOMEM;
		goto error;
	}

	retval = dwc_otg_hcd_qtd_add (qtd, dwc_otg_hcd, _mem_flags);
	if (retval < 0) {
		DWC_ERROR("DWC OTG HCD URB Enqueue failed adding QTD. "
			  "Error status %d\n", retval);
		dwc_otg_hcd_qtd_free(qtd);
		goto error;
	}

error:
	if (retval)
		usb_hcd_unlink_urb_from_ep(_hcd, _urb);

error_not_linked:
	local_irq_restore(flags);

	return retval;
}

/**
 * Aborts/cancels a USB transfer request. Always returns 0 to indicate
 * success.
 */
int dwc_otg_hcd_urb_dequeue(struct usb_hcd *_hcd,
			    struct urb *_urb,
			    int status)
{
	int retval = 0;
	unsigned long flags;
	dwc_otg_hcd_t *dwc_otg_hcd;
	dwc_otg_qtd_t *urb_qtd;
	dwc_otg_qh_t *qh;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD URB Dequeue\n");

	local_irq_save(flags);

	retval = usb_hcd_check_unlink_urb(_hcd, _urb, status);
	if (retval) {
		local_irq_restore(flags);
		return retval;
	}

	dwc_otg_hcd = hcd_to_dwc_otg_hcd(_hcd);

	urb_qtd = (dwc_otg_qtd_t *)_urb->hcpriv;
	if (urb_qtd == NULL) {
		/* URB already unlinked or never linked */
		goto out;
	}

	qh = (dwc_otg_qh_t *) urb_qtd->qtd_priv;
	if (qh == NULL) {
		/* QTD not linked to QH : not to be possible */
		goto out;
	}

#ifdef DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		dump_urb_info(_urb, "dwc_otg_hcd_urb_dequeue");
		if (urb_qtd == qh->qtd_in_process) {
			dump_channel_info(dwc_otg_hcd, qh);
		}
	}
#endif

	if (urb_qtd == qh->qtd_in_process) {
		dwc_otg_hc_halt(dwc_otg_hcd->core_if, qh->channel,
				DWC_OTG_HC_XFER_URB_DEQUEUE);
	}

	/*
	 * Free the QTD and clean up the associated QH. Leave the QH in the
	 * schedule if it has any remaining QTDs.
	 * Check if QTD list is empty in the associates QH
	 */
	if (!list_empty(&qh->qtd_list)) {
		dwc_otg_hcd_qtd_remove_and_free(urb_qtd);
	}

	if (urb_qtd == qh->qtd_in_process) {
		dwc_otg_hcd_qh_deactivate(dwc_otg_hcd, qh, 0);
		qh->channel = NULL;
		qh->qtd_in_process = NULL;
	} else if (list_empty(&qh->qtd_list)) {
		dwc_otg_hcd_qh_remove(dwc_otg_hcd, qh);
	}

out:
	local_irq_restore(flags);

	/* Higher layer software sets URB status. */
	usb_hcd_unlink_urb_from_ep(_hcd, _urb);
	usb_hcd_giveback_urb(_hcd, _urb, status);

	return retval;
}


/**
 * Frees resources in the DWC_otg controller related to a given endpoint. Also
 * clears state in the HCD related to the endpoint. Any URBs for the endpoint
 * must already be dequeued.
 */
void dwc_otg_hcd_endpoint_disable(struct usb_hcd *_hcd,
				  struct usb_host_endpoint *_ep)

{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd(_hcd);
	dwc_otg_qh_t *qh;
	unsigned long flags;
	int retry = 0;

	DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD EP DISABLE: _bEndpointAddress=0x%02x, "
		    "endpoint=%d\n", _ep->desc.bEndpointAddress,
		    dwc_ep_addr_to_endpoint(_ep->desc.bEndpointAddress));

	qh = (dwc_otg_qh_t *)(_ep->hcpriv);
        if (!qh) {
		return;
	}

rescan:
	local_irq_save(flags);

	/** Check that the QTD list is really empty */
	if (!list_empty(&qh->qtd_list)) {
		if (retry++ < 250) {
			local_irq_restore(flags);
			schedule_timeout_uninterruptible(1);
			goto rescan;
		}

		DWC_WARN("DWC OTG HCD EP DISABLE:"
			 " QTD List for this endpoint is not empty\n");
	}

	local_irq_restore(flags);

	_ep->hcpriv = NULL;
	dwc_otg_hcd_qh_remove_and_free(dwc_otg_hcd, qh);

	return;
}

/**
 * Handles host mode interrupts for the DWC_otg controller. Returns IRQ_NONE if
 * there was no interrupt to handle. Returns IRQ_HANDLED if there was a valid
 * interrupt.
  * This function is called by the USB core when an interrupt occurs
 */
irqreturn_t dwc_otg_hcd_irq(struct usb_hcd *_hcd)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
	dwc_otg_core_params_t *params = dwc_otg_hcd->core_if->core_params;
	int32_t retval = IRQ_NONE;

	retval = dwc_otg_hcd_handle_intr(dwc_otg_hcd);

	if (params->fiq_enable) {
		writel((1 << FIQ_MODE_USB_IRQ), PARROT6_VA_VIC + VIC_INT_SOFT_CLEAR);
		/* warning : static value -> fiq mode only used for USB1 controller */
		writel(0x40000000, PARROT6_VA_VIC + VIC_INT_ENABLE);
	}

	return IRQ_RETVAL(retval);
}

/**
 * Creates Status Change bitmap for the root hub and root port. The bitmap is
 * returned in buf. Bit 0 is the status change indicator for the root hub. Bit 1
 * is the status change indicator for the single root port. Returns 1 if either
 * change indicator is 1, otherwise returns 0.
 */
int dwc_otg_hcd_hub_status_data(struct usb_hcd *_hcd, char *_buf)
{
	dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
	unsigned long flags;

	local_irq_save(flags);
	_buf[0] = 0;
	_buf[0] |= (dwc_otg_hcd->flags.b.port_connect_status_change ||
		    dwc_otg_hcd->flags.b.port_reset_change ||
		    dwc_otg_hcd->flags.b.port_enable_change ||
		    dwc_otg_hcd->flags.b.port_suspend_change ||
		    dwc_otg_hcd->flags.b.port_over_current_change) << 1;
	local_irq_restore(flags);

#ifdef DEBUG
	if (_buf[0]) {
		DWC_DEBUGPL(DBG_HCD, "DWC OTG HCD HUB STATUS DATA:"
			    " Root port status changed\n");
		DWC_DEBUGPL(DBG_HCDV, "  port_connect_status_change: %d\n",
			    dwc_otg_hcd->flags.b.port_connect_status_change);
		DWC_DEBUGPL(DBG_HCDV, "  port_reset_change: %d\n",
			    dwc_otg_hcd->flags.b.port_reset_change);
		DWC_DEBUGPL(DBG_HCDV, "  port_enable_change: %d\n",
			    dwc_otg_hcd->flags.b.port_enable_change);
		DWC_DEBUGPL(DBG_HCDV, "  port_suspend_change: %d\n",
			    dwc_otg_hcd->flags.b.port_suspend_change);
		DWC_DEBUGPL(DBG_HCDV, "  port_over_current_change: %d\n",
			    dwc_otg_hcd->flags.b.port_over_current_change);
	}
#endif

	return (_buf[0] != 0);
}

/**
 * Handles hub class-specific requests.
*/
int dwc_otg_hcd_hub_control(struct usb_hcd *_hcd,
			    u16 _typeReq,
			    u16 _wValue,
			    u16 _wIndex,
			    char *_buf,
			    u16 _wLength)
{
	int retval = 0;

        dwc_otg_hcd_t *dwc_otg_hcd = hcd_to_dwc_otg_hcd (_hcd);
        dwc_otg_core_if_t *core_if = hcd_to_dwc_otg_hcd (_hcd)->core_if;
	struct usb_hub_descriptor *desc;
	hprt0_data_t hprt0 = {.d32 = 0};

	uint32_t port_status;
	unsigned long flags;

	local_irq_save(flags);

	switch (_typeReq) {
	case ClearHubFeature:
		DWC_DEBUGPL (DBG_HCD,"DWC OTG HCD HUB CONTROL - "
			     "ClearHubFeature 0x%x\n", _wValue);
		switch (_wValue) {
		case C_HUB_LOCAL_POWER:
		case C_HUB_OVER_CURRENT:
			/* Nothing required here */
			break;
		default:
			retval = -EINVAL;
			DWC_ERROR ("DWC OTG HCD - "
				   "ClearHubFeature request %xh unknown\n", _wValue);
		}
		break;
	case ClearPortFeature:
		if (!_wIndex || _wIndex > 1)
			goto error;

		switch (_wValue) {
		case USB_PORT_FEAT_ENABLE:
			DWC_DEBUGPL (DBG_ANY,"DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_ENABLE\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtena = 1;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			break;
		case USB_PORT_FEAT_SUSPEND:
			DWC_DEBUGPL (DBG_HCD,"DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_SUSPEND\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtres = 1;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			dwc_otg_hcd->flags.b.port_suspend_change = 1;
			break;
		case USB_PORT_FEAT_POWER:
			DWC_DEBUGPL (DBG_HCD,"DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_POWER\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtpwr = 0;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			break;
		case USB_PORT_FEAT_INDICATOR:
			DWC_DEBUGPL (DBG_HCD,"DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_INDICATOR\n");
			/* Port inidicator not supported */
			break;
		case USB_PORT_FEAT_C_CONNECTION:
			/* Clears drivers internal connect status change
			 * flag */
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_C_CONNECTION\n");
			dwc_otg_hcd->flags.b.port_connect_status_change = 0;
			break;
		case USB_PORT_FEAT_C_RESET:
			/* Clears the driver's internal Port Reset Change
			 * flag */
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_C_RESET\n");
			dwc_otg_hcd->flags.b.port_reset_change = 0;
			break;
		case USB_PORT_FEAT_C_ENABLE:
			/* Clears the driver's internal Port
			 * Enable/Disable Change flag */
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_C_ENABLE\n");
			dwc_otg_hcd->flags.b.port_enable_change = 0;
			break;
		case USB_PORT_FEAT_C_SUSPEND:
			/* Clears the driver's internal Port Suspend
			 * Change flag, which is set when resume signaling on
			 * the host port is complete */
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_C_SUSPEND\n");
			dwc_otg_hcd->flags.b.port_suspend_change = 0;
			break;
		case USB_PORT_FEAT_C_OVER_CURRENT:
			DWC_DEBUGPL (DBG_HCD,"DWC OTG HCD HUB CONTROL - "
				     "ClearPortFeature USB_PORT_FEAT_C_OVER_CURRENT\n");
			dwc_otg_hcd->flags.b.port_over_current_change = 0;
			break;
		default:
			retval = -EINVAL;
			DWC_ERROR ("DWC OTG HCD - "
                                   "ClearPortFeature request %xh "
                                   "unknown or unsupported\n", _wValue);
		}
		break;
	case GetHubDescriptor:
		DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
                             "GetHubDescriptor\n");
		desc = (struct usb_hub_descriptor *)_buf;
		desc->bDescLength = 9;
		desc->bDescriptorType = 0x29;
		desc->bNbrPorts = 1;
		desc->wHubCharacteristics = 0x08;
		desc->bPwrOn2PwrGood = 1;
		desc->bHubContrCurrent = 0;
		desc->bitmap[0] = 0;
		desc->bitmap[1] = 0xff;
		break;
	case GetHubStatus:
		DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
                             "GetHubStatus\n");
		memset (_buf, 0, 4);
		break;
	case GetPortStatus:
		DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
                             "GetPortStatus\n");

		if (!_wIndex || _wIndex > 1)
			goto error;

		port_status = 0;

		if (dwc_otg_hcd->flags.b.port_connect_status_change)
			port_status |= (1 << USB_PORT_FEAT_C_CONNECTION);

		if (dwc_otg_hcd->flags.b.port_enable_change)
			port_status |= (1 << USB_PORT_FEAT_C_ENABLE);

		if (dwc_otg_hcd->flags.b.port_suspend_change) {
			port_status |= (1 << USB_PORT_FEAT_C_SUSPEND);
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtres = 0;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
		}

		if (dwc_otg_hcd->flags.b.port_reset_change) {
			port_status |= (1 << USB_PORT_FEAT_C_RESET);
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtrst = 0;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
		}

		if (dwc_otg_hcd->flags.b.port_over_current_change) {
			DWC_ERROR("Device Not Supported\n");
			port_status |= (1 << USB_PORT_FEAT_C_OVER_CURRENT);
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtpwr = 0;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
		}

		hprt0.d32 = dwc_read_reg32(core_if->host_if->hprt0);
		DWC_DEBUGPL(DBG_HCDV, "  HPRT0: 0x%08x\n", hprt0.d32);

		if (hprt0.b.prtconnsts)
			port_status |= (1 << USB_PORT_FEAT_CONNECTION);

		if (hprt0.b.prtena)
			port_status |= (1 << USB_PORT_FEAT_ENABLE);

		if (hprt0.b.prtsusp)
			port_status |= (1 << USB_PORT_FEAT_SUSPEND);

		if (hprt0.b.prtovrcurract)
			port_status |= (1 << USB_PORT_FEAT_OVER_CURRENT);

		if (hprt0.b.prtrst)
			port_status |= (1 << USB_PORT_FEAT_RESET);

		if (hprt0.b.prtpwr)
			port_status |= (1 << USB_PORT_FEAT_POWER);

		if (hprt0.b.prtspd == DWC_HPRT0_PRTSPD_HIGH_SPEED)
			port_status |= (1 << USB_PORT_FEAT_HIGHSPEED);
		else if (hprt0.b.prtspd == DWC_HPRT0_PRTSPD_LOW_SPEED)
			port_status |= (1 << USB_PORT_FEAT_LOWSPEED);

		if (hprt0.b.prttstctl)
			port_status |= (1 << USB_PORT_FEAT_TEST);

		/* USB_PORT_FEAT_INDICATOR unsupported always 0 */

		*((__le32 *) _buf) = cpu_to_le32(port_status);

		break;
	case SetHubFeature:
		DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
                             "SetHubFeature\n");
		/* No HUB features supported */
		break;
	case SetPortFeature:
		if (_wValue != USB_PORT_FEAT_TEST && (!_wIndex || _wIndex > 1))
			goto error;

		switch (_wValue) {
		case USB_PORT_FEAT_SUSPEND:
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "SetPortFeature - USB_PORT_FEAT_SUSPEND\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
                        hprt0.b.prtsusp = 1;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			break;
		case USB_PORT_FEAT_POWER:
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "SetPortFeature - USB_PORT_FEAT_POWER\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtpwr = 1;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			break;
		case USB_PORT_FEAT_RESET:
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "SetPortFeature - USB_PORT_FEAT_RESET\n");
			hprt0.d32 = dwc_otg_read_hprt0 (core_if);
			hprt0.b.prtrst = 1;
			hprt0.b.prtena = 0;
			hprt0.b.prtconndet = 0;
			hprt0.b.prtenchng = 0;
			hprt0.b.prtovrcurrchng = 0;
			dwc_write_reg32(core_if->host_if->hprt0, hprt0.d32);
			dwc_otg_hcd->flags.b.port_reset_change = 1;
			break;
		case USB_PORT_FEAT_TEST:
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "SetPortFeature - USB_PORT_FEAT_TEST\n");
			/* Not supported */
			break;
		case USB_PORT_FEAT_INDICATOR:
			DWC_DEBUGPL (DBG_HCD, "DWC OTG HCD HUB CONTROL - "
				     "SetPortFeature - USB_PORT_FEAT_INDICATOR\n");
			/* Not supported */
			break;
		default:
			retval = -EINVAL;
			DWC_ERROR ("DWC OTG HCD - "
				   "SetPortFeature request %xh "
				   "unknown or unsupported\n", _wValue);
			break;
		}
		break;
	default:
	error:
		retval = -EINVAL;
		DWC_WARN ("DWC OTG HCD - "
                          "Unknown hub control request type or invalid typeReq: %xh wIndex: %xh wValue: %xh\n",
                          _typeReq, _wIndex, _wValue);
		break;
	}

	local_irq_restore(flags);

	return retval;
}


/**
 * Assigns transactions from a QTD to a free host channel and initializes the
 * host channel to perform the transactions. The host channel is removed from
 * the free list.
 *
 * @param _hcd The HCD state structure.
 * @param _qh Transactions from the first QTD for this QH are selected and
 * assigned to a free host channel.
 */
static void assign_and_init_hc(dwc_otg_hcd_t *_hcd, dwc_otg_qh_t *_qh)
{
	dwc_hc_t *hc;
	dwc_otg_qtd_t *qtd;
	struct urb *urb;

	DWC_DEBUGPL(DBG_HCDV, "%s(%p,%p)\n", __func__, _hcd, _qh);

	hc = list_entry(_hcd->free_hc_list.next, dwc_hc_t, hc_list_entry);

	/* Remove the host channel from the free list. */
	list_del_init(&hc->hc_list_entry);

	qtd = list_entry(_qh->qtd_list.next, dwc_otg_qtd_t, qtd_list_entry);
	urb = qtd->urb;
	_qh->channel = hc;
	_qh->qtd_in_process = qtd;

	/*
	 * Use usb_pipedevice to determine device address. This address is
	 * 0 before the SET_ADDRESS command and the correct address afterward.
	 */
	hc->dev_addr = usb_pipedevice(urb->pipe);
	hc->ep_num = usb_pipeendpoint(urb->pipe);

	if (urb->dev->speed == USB_SPEED_LOW) {
		hc->speed = DWC_OTG_EP_SPEED_LOW;
	} else if (urb->dev->speed == USB_SPEED_FULL) {
		hc->speed = DWC_OTG_EP_SPEED_FULL;
	} else {
		hc->speed = DWC_OTG_EP_SPEED_HIGH;
	}

	hc->max_packet = dwc_max_packet(_qh->maxp);

	hc->xfer_started = 0;
	hc->halt_status = DWC_OTG_HC_XFER_NO_HALT_STATUS;
	hc->error_state = (qtd->error_count > 0);
	hc->halt_on_queue = 0;
	hc->halt_pending = 0;
	hc->requests = 0;

	/*
	 * The following values may be modified in the transfer type section
	 * below. The xfer_len value may be reduced when the transfer is
	 * started to accommodate the max widths of the XferSize and PktCnt
	 * fields in the HCTSIZn register.
	 */
	hc->do_ping = _qh->ping_state;
	hc->ep_is_in = (usb_pipein(urb->pipe) != 0);
	hc->data_pid_start = _qh->data_toggle;
	hc->multi_count = 1;

	if (_hcd->core_if->dma_enable) {
		hc->xfer_buff = (uint8_t *)urb->transfer_dma + urb->actual_length;
	} else {
		hc->xfer_buff = (uint8_t *)urb->transfer_buffer + urb->actual_length;
	}
	hc->xfer_len = urb->transfer_buffer_length - urb->actual_length;
	hc->xfer_count = 0;

	/*
	 * Set the split attributes
	 */
	hc->do_split = 0;
	if (_qh->do_split) {
		hc->do_split = 1;
		hc->xact_pos = qtd->isoc_split_pos;
		hc->complete_split = qtd->complete_split;
		hc->hub_addr = urb->dev->tt->hub->devnum;
		hc->port_addr = urb->dev->ttport;
	}

	switch (usb_pipetype(urb->pipe)) {
	case PIPE_CONTROL:
		hc->ep_type = DWC_OTG_EP_TYPE_CONTROL;
		switch (qtd->control_phase) {
		case DWC_OTG_CONTROL_SETUP:
			DWC_DEBUGPL(DBG_HCDV, "  Control setup transaction\n");
			hc->do_ping = 0;
			hc->ep_is_in = 0;
			hc->data_pid_start = DWC_OTG_HC_PID_SETUP;
			if (_hcd->core_if->dma_enable) {
				hc->xfer_buff = (uint8_t *)urb->setup_dma;
			} else {
				hc->xfer_buff = (uint8_t *)urb->setup_packet;
			}
			hc->xfer_len = 8;
			break;
		case DWC_OTG_CONTROL_DATA:
			DWC_DEBUGPL(DBG_HCDV, "  Control data transaction\n");
			hc->data_pid_start = qtd->data_toggle;
			break;
		case DWC_OTG_CONTROL_STATUS:
			/*
			 * Direction is opposite of data direction or IN if no
			 * data.
			 */
			DWC_DEBUGPL(DBG_HCDV, "  Control status transaction\n");
			if (urb->transfer_buffer_length == 0) {
				hc->ep_is_in = 1;
			} else {
				hc->ep_is_in = (usb_pipein(urb->pipe) != USB_DIR_IN);
			}
			if (hc->ep_is_in) {
				hc->do_ping = 0;
			}
			hc->data_pid_start = DWC_OTG_HC_PID_DATA1;
			hc->xfer_len = 0;
			if (_hcd->core_if->dma_enable) {
				hc->xfer_buff = (uint8_t *)_hcd->status_buf_dma;
			} else {
				hc->xfer_buff = (uint8_t *)_hcd->status_buf;
			}
			break;
		}
		break;
	case PIPE_BULK:
		hc->ep_type = DWC_OTG_EP_TYPE_BULK;
		break;
	case PIPE_INTERRUPT:
		hc->ep_type = DWC_OTG_EP_TYPE_INTR;
		break;
	case PIPE_ISOCHRONOUS:
		{
			struct usb_iso_packet_descriptor *frame_desc;
			frame_desc = &urb->iso_frame_desc[qtd->isoc_frame_index];
			hc->ep_type = DWC_OTG_EP_TYPE_ISOC;
			if (_hcd->core_if->dma_enable) {
				hc->xfer_buff = (uint8_t *)urb->transfer_dma;
			} else {
				hc->xfer_buff = (uint8_t *)urb->transfer_buffer;
			}
			hc->xfer_buff += frame_desc->offset + qtd->isoc_split_offset;
			hc->xfer_len = frame_desc->length - qtd->isoc_split_offset;

			if (hc->xact_pos == DWC_HCSPLIT_XACTPOS_ALL) {
				if (hc->xfer_len <= 188) {
					hc->xact_pos = DWC_HCSPLIT_XACTPOS_ALL;
				}
				else {
					hc->xact_pos = DWC_HCSPLIT_XACTPOS_BEGIN;
				}
			}
		}
		break;
	}

	if (hc->ep_type == DWC_OTG_EP_TYPE_INTR ||
	    hc->ep_type == DWC_OTG_EP_TYPE_ISOC) {
		/*
		 * This value may be modified when the transfer is started to
		 * reflect the actual transfer length.
		 */
		hc->multi_count = dwc_hb_mult(_qh->maxp);
	}

	dwc_otg_hc_init(_hcd->core_if, hc);
	hc->qh = _qh;
}

/**
 * This function selects transactions from the HCD transfer schedule and
 * assigns them to available host channels. It is called from HCD interrupt
 * handler functions.
 *
 * @param _hcd The HCD state structure.
 *
 * @return The types of new transactions that were assigned to host channels.
 */
dwc_otg_transaction_type_e dwc_otg_hcd_select_transactions(dwc_otg_hcd_t *_hcd)
{
	struct list_head *qh_ptr;
	dwc_otg_qh_t *qh;
	int num_channels;
	unsigned long flags;
	dwc_otg_transaction_type_e ret_val = DWC_OTG_TRANSACTION_NONE;

#ifdef DEBUG_SOF
	DWC_DEBUGPL(DBG_HCD, "  Select Transactions\n");
#endif

	/* Process entries in the periodic ready list. */
	qh_ptr = _hcd->periodic_sched_ready.next;
	while (qh_ptr != &_hcd->periodic_sched_ready &&
	       !list_empty(&_hcd->free_hc_list)) {

		qh = list_entry(qh_ptr, dwc_otg_qh_t, qh_list_entry);
		assign_and_init_hc(_hcd, qh);

		/*
		 * Move the QH from the periodic ready schedule to the
		 * periodic assigned schedule.
		 */
		qh_ptr = qh_ptr->next;
		local_irq_save(flags);
		list_move(&qh->qh_list_entry, &_hcd->periodic_sched_assigned);
		local_irq_restore(flags);

		ret_val = DWC_OTG_TRANSACTION_PERIODIC;
	}

	/*
	 * Process entries in the inactive portion of the non-periodic
	 * schedule. Some free host channels may not be used if they are
	 * reserved for periodic transfers.
	 */
	qh_ptr = _hcd->non_periodic_sched_inactive.next;
	num_channels = _hcd->core_if->core_params->host_channels;
	while (qh_ptr != &_hcd->non_periodic_sched_inactive &&
	       (_hcd->non_periodic_channels <
		num_channels - _hcd->periodic_channels) &&
	       !list_empty(&_hcd->free_hc_list)) {

		qh = list_entry(qh_ptr, dwc_otg_qh_t, qh_list_entry);
		assign_and_init_hc(_hcd, qh);

		/*
		 * Move the QH from the non-periodic inactive schedule to the
		 * non-periodic active schedule.
		 */
		qh_ptr = qh_ptr->next;
		local_irq_save(flags);
		list_move(&qh->qh_list_entry, &_hcd->non_periodic_sched_active);
		local_irq_restore(flags);

		if (ret_val == DWC_OTG_TRANSACTION_NONE) {
			ret_val = DWC_OTG_TRANSACTION_NON_PERIODIC;
		} else {
			ret_val = DWC_OTG_TRANSACTION_ALL;
		}

		_hcd->non_periodic_channels++;
	}

	return ret_val;
}

/**
 * Attempts to queue a single transaction request for a host channel
 * associated with either a periodic or non-periodic transfer. This function
 * assumes that there is space available in the appropriate request queue. For
 * an OUT transfer or SETUP transaction in Slave mode, it checks whether space
 * is available in the appropriate Tx FIFO.
 *
 * @param _hcd The HCD state structure.
 * @param _hc Host channel descriptor associated with either a periodic or
 * non-periodic transfer.
 * @param _fifo_dwords_avail Number of DWORDs available in the periodic Tx
 * FIFO for periodic transfers or the non-periodic Tx FIFO for non-periodic
 * transfers.
 *
 * @return 1 if a request is queued and more requests may be needed to
 * complete the transfer, 0 if no more requests are required for this
 * transfer, -1 if there is insufficient space in the Tx FIFO.
 */
static int queue_transaction(dwc_otg_hcd_t *_hcd,
			     dwc_hc_t *_hc,
			     uint16_t _fifo_dwords_avail)
{
	int retval;

	if (_hcd->core_if->dma_enable) {
		if (!_hc->xfer_started) {
			dwc_otg_hc_start_transfer(_hcd->core_if, _hc);
			_hc->qh->ping_state = 0;
		}
		retval = 0;
	} else 	if (_hc->halt_pending) {
		/* Don't queue a request if the channel has been halted. */
		retval = 0;
	} else if (_hc->halt_on_queue) {
		dwc_otg_hc_halt(_hcd->core_if, _hc, _hc->halt_status);
		retval = 0;
	} else if (_hc->do_ping) {
		if (!_hc->xfer_started) {
			dwc_otg_hc_start_transfer(_hcd->core_if, _hc);
		}
		retval = 0;
	} else if (!_hc->ep_is_in ||
		   _hc->data_pid_start == DWC_OTG_HC_PID_SETUP) {
		if ((_fifo_dwords_avail * 4) >= _hc->max_packet) {
			if (!_hc->xfer_started) {
				dwc_otg_hc_start_transfer(_hcd->core_if, _hc);
				retval = 1;
			} else {
				retval = dwc_otg_hc_continue_transfer(_hcd->core_if, _hc);
			}
		} else {
			retval = -1;
		}
	} else {
		if (!_hc->xfer_started) {
			dwc_otg_hc_start_transfer(_hcd->core_if, _hc);
			retval = 1;
		} else {
			retval = dwc_otg_hc_continue_transfer(_hcd->core_if, _hc);
		}
	}

	return retval;
}

/**
 * Processes active non-periodic channels and queues transactions for these
 * channels to the DWC_otg controller. After queueing transactions, the NP Tx
 * FIFO Empty interrupt is enabled if there are more transactions to queue as
 * NP Tx FIFO or request queue space becomes available. Otherwise, the NP Tx
 * FIFO Empty interrupt is disabled.
 */
static void process_non_periodic_channels(dwc_otg_hcd_t *_hcd)
{
	gnptxsts_data_t	tx_status;
	struct list_head *orig_qh_ptr;
	dwc_otg_qh_t *qh;
	int status;
	int no_queue_space = 0;
	int no_fifo_space = 0;
	int more_to_do = 0;
	dwc_otg_core_global_regs_t *global_regs = _hcd->core_if->core_global_regs;

	DWC_DEBUGPL(DBG_HCDV, "Queue non-periodic transactions\n");
#ifdef DEBUG
	tx_status.d32 = dwc_read_reg32(&global_regs->gnptxsts);
	DWC_DEBUGPL(DBG_HCDV, "  NP Tx Req Queue Space Avail (before queue): %d\n",
		    tx_status.b.nptxqspcavail);
	DWC_DEBUGPL(DBG_HCDV, "  NP Tx FIFO Space Avail (before queue): %d\n",
		    tx_status.b.nptxfspcavail);
#endif
	/*
	 * Keep track of the starting point. Skip over the start-of-list
	 * entry.
	 */
	if (_hcd->non_periodic_qh_ptr == &_hcd->non_periodic_sched_active) {
		_hcd->non_periodic_qh_ptr = _hcd->non_periodic_qh_ptr->next;
	}
	orig_qh_ptr = _hcd->non_periodic_qh_ptr;

	/*
	 * Process once through the active list or until no more space is
	 * available in the request queue or the Tx FIFO.
	 */
	do {
		tx_status.d32 = dwc_read_reg32(&global_regs->gnptxsts);
		if (!_hcd->core_if->dma_enable && tx_status.b.nptxqspcavail == 0) {
			no_queue_space = 1;
			break;
		}

		qh = list_entry(_hcd->non_periodic_qh_ptr, dwc_otg_qh_t, qh_list_entry);
		status = queue_transaction(_hcd, qh->channel, tx_status.b.nptxfspcavail);

		if (status > 0) {
			more_to_do = 1;
		} else if (status < 0) {
			no_fifo_space = 1;
			break;
		}

		/* Advance to next QH, skipping start-of-list entry. */
		_hcd->non_periodic_qh_ptr = _hcd->non_periodic_qh_ptr->next;
		if (_hcd->non_periodic_qh_ptr == &_hcd->non_periodic_sched_active) {
			_hcd->non_periodic_qh_ptr = _hcd->non_periodic_qh_ptr->next;
		}

	} while (_hcd->non_periodic_qh_ptr != orig_qh_ptr);

	if (!_hcd->core_if->dma_enable) {
		gintmsk_data_t intr_mask = {.d32 = 0};
		intr_mask.b.nptxfempty = 1;

#ifdef DEBUG
		tx_status.d32 = dwc_read_reg32(&global_regs->gnptxsts);
		DWC_DEBUGPL(DBG_HCDV, "  NP Tx Req Queue Space Avail (after queue): %d\n",
			    tx_status.b.nptxqspcavail);
		DWC_DEBUGPL(DBG_HCDV, "  NP Tx FIFO Space Avail (after queue): %d\n",
			    tx_status.b.nptxfspcavail);
#endif
		if (more_to_do || no_queue_space || no_fifo_space) {
			/*
			 * May need to queue more transactions as the request
			 * queue or Tx FIFO empties. Enable the non-periodic
			 * Tx FIFO empty interrupt. (Always use the half-empty
			 * level to ensure that new requests are loaded as
			 * soon as possible.)
			 */
			dwc_modify_reg32(&global_regs->gintmsk, 0, intr_mask.d32);
		} else {
			/*
			 * Disable the Tx FIFO empty interrupt since there are
			 * no more transactions that need to be queued right
			 * now. This function is called from interrupt
			 * handlers to queue more transactions as transfer
			 * states change.
			 */
			dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, 0);
		}
	}
}

/**
 * Processes periodic channels for the next frame and queues transactions for
 * these channels to the DWC_otg controller. After queueing transactions, the
 * Periodic Tx FIFO Empty interrupt is enabled if there are more transactions
 * to queue as Periodic Tx FIFO or request queue space becomes available.
 * Otherwise, the Periodic Tx FIFO Empty interrupt is disabled.
 */
static void process_periodic_channels(dwc_otg_hcd_t *_hcd)
{
	hptxsts_data_t tx_status;
	struct list_head *qh_ptr;
	dwc_otg_qh_t *qh;
	int status;
	int no_queue_space = 0;
	int no_fifo_space = 0;
	dwc_otg_host_global_regs_t *host_regs;

	host_regs = _hcd->core_if->host_if->host_global_regs;

	DWC_DEBUGPL(DBG_HCDV, "Queue periodic transactions\n");
#ifdef DEBUG
	tx_status.d32 = dwc_read_reg32(&host_regs->hptxsts);
	DWC_DEBUGPL(DBG_HCDV, "  P Tx Req Queue Space Avail (before queue): %d\n",
		    tx_status.b.ptxqspcavail);
	DWC_DEBUGPL(DBG_HCDV, "  P Tx FIFO Space Avail (before queue): %d\n",
		    tx_status.b.ptxfspcavail);
#endif

	qh_ptr = _hcd->periodic_sched_assigned.next;
	while (qh_ptr != &_hcd->periodic_sched_assigned) {
		tx_status.d32 = dwc_read_reg32(&host_regs->hptxsts);
		if (tx_status.b.ptxqspcavail == 0) {
			no_queue_space = 1;
			break;
		}

		qh = list_entry(qh_ptr, dwc_otg_qh_t, qh_list_entry);

		/*
		 * Set a flag if we're queuing high-bandwidth in slave mode.
		 * The flag prevents any halts to get into the request queue in
		 * the middle of multiple high-bandwidth packets getting queued.
		 */
		if ((!_hcd->core_if->dma_enable) &&
		    (qh->channel->multi_count > 1))
		{
			_hcd->core_if->queuing_high_bandwidth = 1;
		}

		status = queue_transaction(_hcd, qh->channel, tx_status.b.ptxfspcavail);
		if (status < 0) {
			no_fifo_space = 1;
			break;
		}

		/*
		 * In Slave mode, stay on the current transfer until there is
		 * nothing more to do or the high-bandwidth request count is
		 * reached. In DMA mode, only need to queue one request. The
		 * controller automatically handles multiple packets for
		 * high-bandwidth transfers.
		 */
		if (_hcd->core_if->dma_enable ||
		    (status == 0 ||
		     qh->channel->requests == qh->channel->multi_count)) {
			qh_ptr = qh_ptr->next;
			/*
			 * Move the QH from the periodic assigned schedule to
			 * the periodic queued schedule.
			 */
			list_move(&qh->qh_list_entry, &_hcd->periodic_sched_queued);

			/* done queuing high bandwidth */
			_hcd->core_if->queuing_high_bandwidth = 0;
		}
	}

	if (!_hcd->core_if->dma_enable) {
		dwc_otg_core_global_regs_t *global_regs;
		gintmsk_data_t intr_mask = {.d32 = 0};

		global_regs = _hcd->core_if->core_global_regs;
		intr_mask.b.ptxfempty = 1;
#ifdef DEBUG
		tx_status.d32 = dwc_read_reg32(&host_regs->hptxsts);
		DWC_DEBUGPL(DBG_HCDV, "  P Tx Req Queue Space Avail (after queue): %d\n",
			    tx_status.b.ptxqspcavail);
		DWC_DEBUGPL(DBG_HCDV, "  P Tx FIFO Space Avail (after queue): %d\n",
			    tx_status.b.ptxfspcavail);
#endif
		if (!(list_empty(&_hcd->periodic_sched_assigned)) ||
		    no_queue_space || no_fifo_space) {
			/*
			 * May need to queue more transactions as the request
			 * queue or Tx FIFO empties. Enable the periodic Tx
			 * FIFO empty interrupt. (Always use the half-empty
			 * level to ensure that new requests are loaded as
			 * soon as possible.)
			 */
			dwc_modify_reg32(&global_regs->gintmsk, 0, intr_mask.d32);
		} else {
			/*
			 * Disable the Tx FIFO empty interrupt since there are
			 * no more transactions that need to be queued right
			 * now. This function is called from interrupt
			 * handlers to queue more transactions as transfer
			 * states change.
			 */
			dwc_modify_reg32(&global_regs->gintmsk, intr_mask.d32, 0);
		}
	}
}

/**
 * This function processes the currently active host channels and queues
 * transactions for these channels to the DWC_otg controller. It is called
 * from HCD interrupt handler functions.
 *
 * @param _hcd The HCD state structure.
 * @param _tr_type The type(s) of transactions to queue (non-periodic,
 * periodic, or both).
 */
void dwc_otg_hcd_queue_transactions(dwc_otg_hcd_t *_hcd,
				    dwc_otg_transaction_type_e _tr_type)
{
#ifdef DEBUG_SOF
	DWC_DEBUGPL(DBG_HCD, "Queue Transactions\n");
#endif
	/* Process host channels associated with periodic transfers. */
	if ((_tr_type == DWC_OTG_TRANSACTION_PERIODIC ||
	     _tr_type == DWC_OTG_TRANSACTION_ALL) &&
	    !list_empty(&_hcd->periodic_sched_assigned)) {

		process_periodic_channels(_hcd);
	}

	/* Process host channels associated with non-periodic transfers. */
	if ((_tr_type == DWC_OTG_TRANSACTION_NON_PERIODIC ||
	     _tr_type == DWC_OTG_TRANSACTION_ALL)) {
		if (!list_empty(&_hcd->non_periodic_sched_active)) {
			process_non_periodic_channels(_hcd);
		} else {
			/*
			 * Ensure NP Tx FIFO empty interrupt is disabled when
			 * there are no non-periodic transfers to process.
			 */
			gintmsk_data_t gintmsk = {.d32 = 0};
			gintmsk.b.nptxfempty = 1;
			dwc_modify_reg32(&_hcd->core_if->core_global_regs->gintmsk,
					 gintmsk.d32, 0);
		}
	}
}

/**
 * Sets the final status of an URB and returns it to the device driver. Any
 * required cleanup of the URB is performed.
 */
void dwc_otg_hcd_complete_urb(dwc_otg_hcd_t *_hcd, struct urb *_urb, int _status)
{
	unsigned long flags;

#ifdef DEBUG
	if (CHK_DEBUG_LEVEL(DBG_HCDV | DBG_HCD_URB)) {
		DWC_PRINT("%s: urb %p, device %d, ep %d %s, status=%d\n",
			  __func__, _urb, usb_pipedevice(_urb->pipe),
			  usb_pipeendpoint(_urb->pipe),
			  usb_pipein(_urb->pipe) ? "IN" : "OUT", _status);
		if (usb_pipetype(_urb->pipe) == PIPE_ISOCHRONOUS) {
			int i;
			for (i = 0; i < _urb->number_of_packets; i++) {
				DWC_PRINT("  ISO Desc %d status: %d\n",
					  i, _urb->iso_frame_desc[i].status);
			}
		}
	}
#endif

	usb_hcd_unlink_urb_from_ep(dwc_otg_hcd_to_hcd(_hcd), _urb);
	local_irq_save(flags);
	usb_hcd_giveback_urb(dwc_otg_hcd_to_hcd(_hcd), _urb, _status);
	local_irq_restore(flags);
}

/*
 * Returns the Queue Head for an URB.
 */
dwc_otg_qh_t *dwc_urb_to_qh(struct urb *_urb)
{
	struct usb_host_endpoint *ep = dwc_urb_to_endpoint(_urb);
	return (dwc_otg_qh_t *)ep->hcpriv;
}

#ifdef DEBUG
void dwc_print_setup_data (uint8_t *setup)
{
	int i;
	if (CHK_DEBUG_LEVEL(DBG_HCD)){
		DWC_PRINT("Setup Data = MSB ");
		for (i=7; i>=0; i--) DWC_PRINT ("%02x ", setup[i]);
		DWC_PRINT("\n");
		DWC_PRINT("  bmRequestType Tranfer = %s\n", (setup[0]&0x80) ? "Device-to-Host" : "Host-to-Device");
		DWC_PRINT("  bmRequestType Type = ");
		switch ((setup[0]&0x60) >> 5) {
		case 0: DWC_PRINT("Standard\n"); break;
		case 1:	DWC_PRINT("Class\n"); break;
		case 2:	DWC_PRINT("Vendor\n"); break;
		case 3: DWC_PRINT("Reserved\n"); break;
		}
		DWC_PRINT("  bmRequestType Recipient = ");
		switch (setup[0]&0x1f) {
		case 0: DWC_PRINT("Device\n"); break;
		case 1: DWC_PRINT("Interface\n"); break;
		case 2: DWC_PRINT("Endpoint\n"); break;
		case 3: DWC_PRINT("Other\n"); break;
		default: DWC_PRINT("Reserved\n"); break;
		}
		DWC_PRINT("  bRequest = 0x%0x\n", setup[1]);
		DWC_PRINT("  wValue = 0x%0x\n", *((uint16_t *)&setup[2]));
		DWC_PRINT("  wIndex = 0x%0x\n", *((uint16_t *)&setup[4]));
		DWC_PRINT("  wLength = 0x%0x\n\n", *((uint16_t *)&setup[6]));
	}
}
#endif

void dwc_otg_hcd_dump_frrem(dwc_otg_hcd_t *_hcd) {
// The 64-bit divisions here cause __udivdi3() to be invoked, which is no longer provided in newer kernels
#if defined(DEBUG) && LINUX_VERSION_CODE < KERNEL_VERSION(2,6,20)
	DWC_PRINT("Frame remaining at SOF:\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->frrem_samples, _hcd->frrem_accum,
		  (_hcd->frrem_samples > 0) ?
		  _hcd->frrem_accum/_hcd->frrem_samples : 0);

	DWC_PRINT("\n");
	DWC_PRINT("Frame remaining at start_transfer (uframe 7):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->core_if->hfnum_7_samples, _hcd->core_if->hfnum_7_frrem_accum,
		  (_hcd->core_if->hfnum_7_samples > 0) ?
		  _hcd->core_if->hfnum_7_frrem_accum/_hcd->core_if->hfnum_7_samples : 0);
	DWC_PRINT("Frame remaining at start_transfer (uframe 0):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->core_if->hfnum_0_samples, _hcd->core_if->hfnum_0_frrem_accum,
		  (_hcd->core_if->hfnum_0_samples > 0) ?
		  _hcd->core_if->hfnum_0_frrem_accum/_hcd->core_if->hfnum_0_samples : 0);
	DWC_PRINT("Frame remaining at start_transfer (uframe 1-6):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->core_if->hfnum_other_samples, _hcd->core_if->hfnum_other_frrem_accum,
		  (_hcd->core_if->hfnum_other_samples > 0) ?
		  _hcd->core_if->hfnum_other_frrem_accum/_hcd->core_if->hfnum_other_samples : 0);

	DWC_PRINT("\n");
	DWC_PRINT("Frame remaining at sample point A (uframe 7):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_7_samples_a, _hcd->hfnum_7_frrem_accum_a,
		  (_hcd->hfnum_7_samples_a > 0) ?
		  _hcd->hfnum_7_frrem_accum_a/_hcd->hfnum_7_samples_a : 0);
	DWC_PRINT("Frame remaining at sample point A (uframe 0):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_0_samples_a, _hcd->hfnum_0_frrem_accum_a,
		  (_hcd->hfnum_0_samples_a > 0) ?
		  _hcd->hfnum_0_frrem_accum_a/_hcd->hfnum_0_samples_a : 0);
	DWC_PRINT("Frame remaining at sample point A (uframe 1-6):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_other_samples_a, _hcd->hfnum_other_frrem_accum_a,
		  (_hcd->hfnum_other_samples_a > 0) ?
		  _hcd->hfnum_other_frrem_accum_a/_hcd->hfnum_other_samples_a : 0);

	DWC_PRINT("\n");
	DWC_PRINT("Frame remaining at sample point B (uframe 7):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_7_samples_b, _hcd->hfnum_7_frrem_accum_b,
		  (_hcd->hfnum_7_samples_b > 0) ?
		  _hcd->hfnum_7_frrem_accum_b/_hcd->hfnum_7_samples_b : 0);
	DWC_PRINT("Frame remaining at sample point B (uframe 0):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_0_samples_b, _hcd->hfnum_0_frrem_accum_b,
		  (_hcd->hfnum_0_samples_b > 0) ?
		  _hcd->hfnum_0_frrem_accum_b/_hcd->hfnum_0_samples_b : 0);
	DWC_PRINT("Frame remaining at sample point B (uframe 1-6):\n");
	DWC_PRINT("  samples %u, accum %llu, avg %llu\n",
		  _hcd->hfnum_other_samples_b, _hcd->hfnum_other_frrem_accum_b,
		  (_hcd->hfnum_other_samples_b > 0) ?
		  _hcd->hfnum_other_frrem_accum_b/_hcd->hfnum_other_samples_b : 0);
#endif
}

void dwc_otg_hcd_dump_state(dwc_otg_hcd_t *_hcd)
{
#ifdef DEBUG
	int num_channels;
	int i;
	gnptxsts_data_t	np_tx_status;
	hptxsts_data_t p_tx_status;

	num_channels = _hcd->core_if->core_params->host_channels;
	DWC_PRINT("\n");
	DWC_PRINT("************************************************************\n");
	DWC_PRINT("HCD State:\n");
	DWC_PRINT("  Num channels: %d\n", num_channels);
	for (i = 0; i < num_channels; i++) {
		dwc_hc_t *hc = _hcd->hc_ptr_array[i];
		DWC_PRINT("  Channel %d:\n", i);
		DWC_PRINT("    dev_addr: %d, ep_num: %d, ep_is_in: %d\n",
			  hc->dev_addr, hc->ep_num, hc->ep_is_in);
		DWC_PRINT("    speed: %d\n", hc->speed);
		DWC_PRINT("    ep_type: %d\n", hc->ep_type);
		DWC_PRINT("    max_packet: %d\n", hc->max_packet);
		DWC_PRINT("    data_pid_start: %d\n", hc->data_pid_start);
		DWC_PRINT("    multi_count: %d\n", hc->multi_count);
		DWC_PRINT("    xfer_started: %d\n", hc->xfer_started);
		DWC_PRINT("    xfer_buff: %p\n", hc->xfer_buff);
		DWC_PRINT("    xfer_len: %d\n", hc->xfer_len);
		DWC_PRINT("    xfer_count: %d\n", hc->xfer_count);
		DWC_PRINT("    halt_on_queue: %d\n", hc->halt_on_queue);
		DWC_PRINT("    halt_pending: %d\n", hc->halt_pending);
		DWC_PRINT("    halt_status: %d\n", hc->halt_status);
		DWC_PRINT("    do_split: %d\n", hc->do_split);
		DWC_PRINT("    complete_split: %d\n", hc->complete_split);
		DWC_PRINT("    hub_addr: %d\n", hc->hub_addr);
		DWC_PRINT("    port_addr: %d\n", hc->port_addr);
		DWC_PRINT("    xact_pos: %d\n", hc->xact_pos);
		DWC_PRINT("    requests: %d\n", hc->requests);
		DWC_PRINT("    qh: %p\n", hc->qh);
		if (hc->xfer_started) {
			hfnum_data_t hfnum;
			hcchar_data_t hcchar;
			hctsiz_data_t hctsiz;
			hcint_data_t hcint;
			hcintmsk_data_t hcintmsk;
			hfnum.d32 = dwc_read_reg32(&_hcd->core_if->host_if->host_global_regs->hfnum);
			hcchar.d32 = dwc_read_reg32(&_hcd->core_if->host_if->hc_regs[i]->hcchar);
			hctsiz.d32 = dwc_read_reg32(&_hcd->core_if->host_if->hc_regs[i]->hctsiz);
			hcint.d32 = dwc_read_reg32(&_hcd->core_if->host_if->hc_regs[i]->hcint);
			hcintmsk.d32 = dwc_read_reg32(&_hcd->core_if->host_if->hc_regs[i]->hcintmsk);
			DWC_PRINT("    hfnum: 0x%08x\n", hfnum.d32);
			DWC_PRINT("    hcchar: 0x%08x\n", hcchar.d32);
			DWC_PRINT("    hctsiz: 0x%08x\n", hctsiz.d32);
			DWC_PRINT("    hcint: 0x%08x\n", hcint.d32);
			DWC_PRINT("    hcintmsk: 0x%08x\n", hcintmsk.d32);
		}
		if (hc->xfer_started && (hc->qh != NULL) && (hc->qh->qtd_in_process != NULL)) {
			dwc_otg_qtd_t *qtd;
			struct urb *urb;
			qtd = hc->qh->qtd_in_process;
			urb = qtd->urb;
			DWC_PRINT("    URB Info:\n");
			DWC_PRINT("      qtd: %p, urb: %p\n", qtd, urb);
			if (urb != NULL) {
				DWC_PRINT("      Dev: %d, EP: %d %s\n",
					  usb_pipedevice(urb->pipe), usb_pipeendpoint(urb->pipe),
					  usb_pipein(urb->pipe) ? "IN" : "OUT");
				DWC_PRINT("      Max packet size: %d\n",
					  usb_maxpacket(urb->dev, urb->pipe, usb_pipeout(urb->pipe)));
				DWC_PRINT("      transfer_buffer: %p\n", urb->transfer_buffer);
				DWC_PRINT("      transfer_dma: %p\n", (void *)urb->transfer_dma);
				DWC_PRINT("      transfer_buffer_length: %d\n", urb->transfer_buffer_length);
				DWC_PRINT("      actual_length: %d\n", urb->actual_length);
			}
		}
	}
	DWC_PRINT("  non_periodic_channels: %d\n", _hcd->non_periodic_channels);
	DWC_PRINT("  periodic_channels: %d\n", _hcd->periodic_channels);
	DWC_PRINT("  periodic_usecs: %d\n", _hcd->periodic_usecs);
	np_tx_status.d32 = dwc_read_reg32(&_hcd->core_if->core_global_regs->gnptxsts);
	DWC_PRINT("  NP Tx Req Queue Space Avail: %d\n", np_tx_status.b.nptxqspcavail);
	DWC_PRINT("  NP Tx FIFO Space Avail: %d\n", np_tx_status.b.nptxfspcavail);
	p_tx_status.d32 = dwc_read_reg32(&_hcd->core_if->host_if->host_global_regs->hptxsts);
	DWC_PRINT("  P Tx Req Queue Space Avail: %d\n", p_tx_status.b.ptxqspcavail);
	DWC_PRINT("  P Tx FIFO Space Avail: %d\n", p_tx_status.b.ptxfspcavail);
	dwc_otg_hcd_dump_frrem(_hcd);
	dwc_otg_dump_global_registers(_hcd->core_if);
	dwc_otg_dump_host_registers(_hcd->core_if);
	DWC_PRINT("************************************************************\n");
	DWC_PRINT("\n");
#endif
}
