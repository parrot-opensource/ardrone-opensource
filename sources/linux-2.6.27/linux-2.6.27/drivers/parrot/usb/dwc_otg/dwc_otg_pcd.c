 /* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_pcd.c $
 * $Revision: #38 $
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
 * This file implements the Peripheral Controller Driver.
 *
 * The Peripheral Controller Driver (PCD) is responsible for
 * translating requests from the Function Driver into the appropriate
 * actions on the DWC_otg controller. It isolates the Function Driver
 * from the specifics of the controller by providing an API to the
 * Function Driver.
 *
 * The Peripheral Controller Driver for Linux will implement the
 * Gadget API, so that the existing Gadget drivers can be used.
 * (Gadget Driver is the Linux terminology for a Function Driver.)
 *
 * The Linux Gadget API is defined in the header file
 * <code><linux/usb_gadget.h></code>.  The USB EP operations API is
 * defined in the structure <code>usb_ep_ops</code> and the USB
 * Controller API is defined in the structure
 * <code>usb_gadget_ops</code>.
 *
 * An important function of the PCD is managing interrupts generated
 * by the DWC_otg controller. The implementation of the DWC_otg device
 * mode interrupt service routines is in dwc_otg_pcd_intr.c.
 *
 * @todo Add Device Mode test modes (Test J mode, Test K mode, etc).
 * @todo Does it work when the request size is greater than DEPTSIZ
 * transfer size
 *
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
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>

#include <mach/irqs.h>

#include "dwc_otg_driver.h"
#include "dwc_otg_pcd.h"

/**
 * Static PCD pointer for use in usb_gadget_register_driver and
 * usb_gadget_unregister_driver.  Initialized in dwc_otg_pcd_init.
 */
static dwc_otg_pcd_t *s_pcd = 0;

static int dwc_usb_gadget_unregister_driver(struct usb_gadget_driver *driver, void *priv);
static int dwc_usb_gadget_register_driver(struct usb_gadget_driver *driver, void *priv);



/* Display the contents of the buffer */
extern void dump_msg(const u8 *buf, unsigned int length);


/**
 * This function completes a request.  It call's the request call back.
 */
void request_done(dwc_otg_pcd_ep_t *ep, dwc_otg_pcd_request_t *req,
		  int status)
{
	unsigned stopped = ep->stopped;

	DWC_DEBUGPL(DBG_PCDV, "%s(%p)\n", __func__, ep);
	list_del_init(&req->queue);

	if (req->req.status == -EINPROGRESS) {
		req->req.status = status;
	} else {
		status = req->req.status;
	}

	/* don't modify queue heads during completion callback */
	ep->stopped = 1;
	SPIN_UNLOCK(&ep->pcd->lock);
	req->req.complete(&ep->ep, &req->req);
	SPIN_LOCK(&ep->pcd->lock);

	if (ep->pcd->request_pending > 0) {
		--ep->pcd->request_pending;
	}

	ep->stopped = stopped;
}

/**
 * This function terminates all the requsts in the EP request queue.
 */
void request_nuke(dwc_otg_pcd_ep_t *ep)
{
	dwc_otg_pcd_request_t *req;

	ep->stopped = 1;

	/* called with irqs blocked?? */
	while (!list_empty(&ep->queue)) {
		req = list_entry(ep->queue.next, dwc_otg_pcd_request_t,
				 queue);
		request_done(ep, req, -ESHUTDOWN);
	}
}

/* USB Endpoint Operations */
/*
 * The following sections briefly describe the behavior of the Gadget
 * API endpoint operations implemented in the DWC_otg driver
 * software. Detailed descriptions of the generic behavior of each of
 * these functions can be found in the Linux header file
 * include/linux/usb_gadget.h.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_ep_ops. The Gadget Driver calls the wrapper
 * function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper
 * functions. Within each section, the corresponding DWC_otg PCD
 * function name is specified.
 *
 */

/**
 * This function assigns periodic Tx FIFO to an periodic EP
 * in shared Tx FIFO mode
 */
static uint32_t assign_perio_tx_fifo(dwc_otg_core_if_t	*core_if)
{
	uint32_t PerTxMsk = 1;
	int i;
	for(i = 0; i < core_if->hwcfg4.b.num_dev_perio_in_ep; ++i)
	{
		if((PerTxMsk & core_if->p_tx_msk) == 0) {
			core_if->p_tx_msk |= PerTxMsk;
			return i + 1;
		}
		PerTxMsk <<= 1;
	}
	return 0;
}

/**
 * This function releases periodic Tx FIFO
 * in shared Tx FIFO mode
 */
static void release_perio_tx_fifo(dwc_otg_core_if_t *core_if, uint32_t fifo_num)
{
	core_if->p_tx_msk = (core_if->p_tx_msk & (1 << (fifo_num - 1))) ^ core_if->p_tx_msk;
}

/**
 * This function assigns periodic Tx FIFO to an periodic EP
 * in shared Tx FIFO mode
 */
static uint32_t assign_tx_fifo(dwc_otg_core_if_t *core_if)
{
	uint32_t TxMsk = 1;
	int i;

	for(i = 0; i < core_if->hwcfg4.b.num_in_eps; ++i)
	{
		if((TxMsk & core_if->tx_msk) == 0) {
			core_if->tx_msk |= TxMsk;
			return i + 1;
		}
		TxMsk <<= 1;
	}
	return 0;
}

/**
 * This function releases periodic Tx FIFO
 * in shared Tx FIFO mode
 */
static void release_tx_fifo(dwc_otg_core_if_t	*core_if, uint32_t fifo_num)
{
	core_if->tx_msk = (core_if->tx_msk & (1 << (fifo_num - 1))) ^ core_if->tx_msk;
}

/**
 * This function allocates a DMA Descriptor chain for the Endpoint
 * buffer to be used for a transfer to/from the specified endpoint.
 */
static dwc_otg_dma_desc_t* ep_alloc_desc(uint32_t * dma_desc_addr)
{

	return dma_alloc_coherent(NULL, sizeof(dwc_otg_dma_desc_t), dma_desc_addr, GFP_KERNEL);
}

/**
 * This function frees a DMA Descriptor chain that was allocated by ep_alloc_desc.
 */
static void ep_free_desc(dwc_otg_dma_desc_t* desc_addr, uint32_t dma_desc_addr)
{
	dma_free_coherent(NULL, sizeof(dwc_otg_dma_desc_t), desc_addr, dma_desc_addr);
}

#ifdef _EN_ISOC_
/**
 * This function allocates a DMA Descriptor chain for the Endpoint
 * buffer to be used for a transfer to/from the specified endpoint.
 */
static dwc_otg_iso_dma_desc_t* ep_alloc_iso_desc_chain(uint32_t * dma_desc_addr, uint32_t count)
{
	return dma_alloc_coherent(NULL, count * sizeof(dwc_otg_iso_dma_desc_t), dma_desc_addr, GFP_KERNEL);
}

/**
 * This function frees a DMA Descriptor chain that was allocated by ep_alloc_desc.
 */
static void ep_free_iso_desc_chain(dwc_otg_iso_dma_desc_t* desc_addr, uint32_t dma_desc_addr, uint32_t count)
{
	dma_free_coherent(NULL, count * sizeof(dwc_otg_dma_desc_t), desc_addr, dma_desc_addr);
}

#endif  //_EN_ISOC_

/**
 * This function is called by the Gadget Driver for each EP to be
 * configured for the current configuration (SET_CONFIGURATION).
 *
 * This function initializes the dwc_otg_ep_t data structure, and then
 * calls dwc_otg_ep_activate.
 */
static int dwc_otg_pcd_ep_enable(struct usb_ep *usb_ep,
				 const struct usb_endpoint_descriptor *ep_desc)
{
	dwc_otg_pcd_ep_t *ep = 0;
	dwc_otg_pcd_t *pcd = 0;
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, usb_ep, ep_desc);

	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);
	if (!usb_ep || !ep_desc || ep_desc->bDescriptorType != USB_DT_ENDPOINT) {
		DWC_WARN("%s, bad ep or descriptor\n", __func__);
		return -EINVAL;
	}
	if (ep == &ep->pcd->ep0) {
		DWC_WARN("%s, bad ep(0)\n", __func__);
		return -EINVAL;
	}

	/* Check FIFO size? */
	if (!ep_desc->wMaxPacketSize) {
		DWC_WARN("%s, bad %s maxpacket\n", __func__, usb_ep->name);
		return -ERANGE;
	}

	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) {
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);

	ep->desc = ep_desc;
	ep->ep.maxpacket = le16_to_cpu (ep_desc->wMaxPacketSize);

	/*
	 * Activate the EP
	 */
	ep->stopped = 0;

	ep->dwc_ep.is_in = (USB_DIR_IN & ep_desc->bEndpointAddress) != 0;
	ep->dwc_ep.maxpacket = ep->ep.maxpacket;

	ep->dwc_ep.type = ep_desc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;

	if(ep->dwc_ep.is_in) {
		if(!pcd->otg_dev->core_if->en_multiple_tx_fifo) {
			ep->dwc_ep.tx_fifo_num = 0;

			if (ep->dwc_ep.type == USB_ENDPOINT_XFER_ISOC) {
				/*
				 * if ISOC EP then assign a Periodic Tx FIFO.
				 */
				ep->dwc_ep.tx_fifo_num = assign_perio_tx_fifo(pcd->otg_dev->core_if);
			 }
		} else {
			/*
			 * if Dedicated FIFOs mode is on then assign a Tx FIFO.
			 */
			ep->dwc_ep.tx_fifo_num = assign_tx_fifo(pcd->otg_dev->core_if);

		}
	}
	/* Set initial data PID. */
	if (ep->dwc_ep.type == USB_ENDPOINT_XFER_BULK) {
		ep->dwc_ep.data_pid_start = 0;
	}

	/* Alloc DMA Descriptors */
	if(GET_CORE_IF(pcd)->dma_desc_enable) {
		if(ep->dwc_ep.type != USB_ENDPOINT_XFER_ISOC) {
			ep->dwc_ep.desc_addr = ep_alloc_desc(&ep->dwc_ep.dma_desc_addr);
			if(!ep->dwc_ep.desc_addr) {
				DWC_WARN("%s, can't allocate DMA descriptor\n", __func__);
				return -ENOMEM;
			}
		}
	}

	DWC_DEBUGPL(DBG_PCD, "Activate %s-%s: type=%d, mps=%d desc=%p\n",
					ep->ep.name, (ep->dwc_ep.is_in ?"IN":"OUT"),
					ep->dwc_ep.type, ep->dwc_ep.maxpacket, ep->desc);

	dwc_otg_ep_activate(GET_CORE_IF(pcd), &ep->dwc_ep);
	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	return 0;
}

/**
 * This function is called when an EP is disabled due to disconnect or
 * change in configuration. Any pending requests will terminate with a
 * status of -ESHUTDOWN.
 *
 * This function modifies the dwc_otg_ep_t data structure for this EP,
 * and then calls dwc_otg_ep_deactivate.
 */
static int dwc_otg_pcd_ep_disable(struct usb_ep *usb_ep)
{
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t *pcd = 0;
	unsigned long flags;
	dwc_otg_dma_desc_t* desc_addr;
	uint32_t dma_desc_addr;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, usb_ep);
	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);
	if (!usb_ep || !ep->desc) {
		DWC_DEBUGPL(DBG_PCD, "%s, %s not enabled\n", __func__,
			usb_ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);

	request_nuke(ep);

	dwc_otg_ep_deactivate(GET_CORE_IF(ep->pcd), &ep->dwc_ep);
	ep->desc = 0;
	ep->stopped = 1;

	if(ep->dwc_ep.is_in) {
		dwc_otg_flush_tx_fifo(GET_CORE_IF(ep->pcd), ep->dwc_ep.tx_fifo_num);
		release_perio_tx_fifo(GET_CORE_IF(ep->pcd), ep->dwc_ep.tx_fifo_num);
		release_tx_fifo(GET_CORE_IF(ep->pcd), ep->dwc_ep.tx_fifo_num);
	}

	/* Free DMA Descriptors */
	pcd = ep->pcd;
	if(GET_CORE_IF(pcd)->dma_desc_enable) {
		if(ep->dwc_ep.type != USB_ENDPOINT_XFER_ISOC) {
			desc_addr = ep->dwc_ep.desc_addr;
			dma_desc_addr = ep->dwc_ep.dma_desc_addr;

			/* Cannot call dma_free_coherent() with IRQs disabled */
			SPIN_UNLOCK_IRQRESTORE(&ep->pcd->lock, flags);
			ep_free_desc(desc_addr, dma_desc_addr);

			goto out_unlocked;
		}
	}

	SPIN_UNLOCK_IRQRESTORE(&ep->pcd->lock, flags);

out_unlocked:
	DWC_DEBUGPL(DBG_PCD, "%s disabled\n", usb_ep->name);
	return 0;
}


/**
 * This function allocates a request object to use with the specified
 * endpoint.
 *
 * @param ep The endpoint to be used with with the request
 * @param gfp_flags the GFP_* flags to use.
 */
static struct usb_request *dwc_otg_pcd_alloc_request(struct usb_ep *ep,
						     gfp_t gfp_flags)
{
	dwc_otg_pcd_request_t *req;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%d)\n", __func__, ep, gfp_flags);
	if (0 == ep) {
		DWC_WARN("%s() %s\n", __func__, "Invalid EP!\n");
		return 0;
	}
	req = kmalloc(sizeof(dwc_otg_pcd_request_t), gfp_flags);
	if (0 == req) {
		DWC_WARN("%s() %s\n", __func__,
				 "request allocation failed!\n");
		return 0;
	}
	memset(req, 0, sizeof(dwc_otg_pcd_request_t));
	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);
	return &req->req;
}

/**
 * This function frees a request object.
 *
 * @param ep The endpoint associated with the request
 * @param req The request being freed
 */
static void dwc_otg_pcd_free_request(struct usb_ep *ep,
					 struct usb_request *req)
{
	dwc_otg_pcd_request_t *request;
	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, ep, req);

	if (0 == ep || 0 == req) {
		DWC_WARN("%s() %s\n", __func__,
				 "Invalid ep or req argument!\n");
		return;
	}

	request = container_of(req, dwc_otg_pcd_request_t, req);
	kfree(request);
}


/**
 * This function is used to submit an I/O Request to an EP.
 *
 *	- When the request completes the request's completion callback
 *	  is called to return the request to the driver.
 *	- An EP, except control EPs, may have multiple requests
 *	  pending.
 *	- Once submitted the request cannot be examined or modified.
 *	- Each request is turned into one or more packets.
 *	- A BULK EP can queue any amount of data; the transfer is
 *	  packetized.
 *	- Zero length Packets are specified with the request 'zero'
 *	  flag.
 */
static int dwc_otg_pcd_ep_queue(struct usb_ep *usb_ep,
				struct usb_request *usb_req,
				gfp_t gfp_flags)
{
	int prevented = 0;
	dwc_otg_pcd_request_t *req;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t *pcd;
	unsigned long flags = 0;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p,%d)\n",
		    __func__, usb_ep, usb_req, gfp_flags);

	req = container_of(usb_req, dwc_otg_pcd_request_t, req);
	if (!usb_req || !usb_req->complete || !usb_req->buf ||
			!list_empty(&req->queue)) {
		DWC_WARN("%s, bad params\n", __func__);
		return -EINVAL;
	}

	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);
	if (!usb_ep || (!ep->desc && ep->dwc_ep.num != 0)/* || ep->stopped != 0*/) {
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}

	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) {
		DWC_DEBUGPL(DBG_PCDV, "gadget.speed=%d\n", pcd->gadget.speed);
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}


	DWC_DEBUGPL(DBG_PCD, "%s queue req %p, len %d buf %p\n",
			   usb_ep->name, usb_req, usb_req->length, usb_req->buf);

	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);

#if defined(DEBUG) & defined(VERBOSE)
	dump_msg(usb_req->buf, usb_req->length);
#endif

	usb_req->status = -EINPROGRESS;
	usb_req->actual = 0;

	/*
	 * For EP0 IN without premature status, zlp is required?
	 */
	if (ep->dwc_ep.num == 0 && ep->dwc_ep.is_in) {
		DWC_DEBUGPL(DBG_PCDV, "%s-OUT ZLP\n", usb_ep->name);
		//_req->zero = 1;
	}

	/* Start the transfer */
	if (list_empty(&ep->queue) && !ep->stopped) {
		/* EP0 Transfer? */
		if (ep->dwc_ep.num == 0) {
			switch (pcd->ep0state) {
			case EP0_IN_DATA_PHASE:
				DWC_DEBUGPL(DBG_PCD,
					    "%s ep0: EP0_IN_DATA_PHASE\n",
					    __func__);
				break;

			case EP0_OUT_DATA_PHASE:
				DWC_DEBUGPL(DBG_PCD,
					    "%s ep0: EP0_OUT_DATA_PHASE\n",
					    __func__);
				if (pcd->request_config) {
					/* Complete STATUS PHASE */
					ep->dwc_ep.is_in = 1;
					pcd->ep0state = EP0_IN_STATUS_PHASE;
				}
				break;

			case EP0_IN_STATUS_PHASE:
				DWC_DEBUGPL(DBG_PCD,
					    "%s ep0: EP0_IN_STATUS_PHASE\n",
					    __func__);
				break;

			default:
				DWC_DEBUGPL(DBG_ANY, "ep0: odd state %d\n",
						pcd->ep0state);
				SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
				return -EL2HLT;
			}

			/* configure DMA address */
			if (GET_CORE_IF(pcd)->dma_enable) {
				if (usb_req->dma == DMA_ADDR_INVALID) {
					ep->dwc_ep.dma_addr = dma_map_single(
						NULL,
						usb_req->buf,
						usb_req->length,
						ep->dwc_ep.is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
				} else {
					dma_sync_single_for_device(
						NULL,
						usb_req->dma, usb_req->length,
						ep->dwc_ep.is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
					ep->dwc_ep.dma_addr = usb_req->dma;
				}
			}

			ep->dwc_ep.start_xfer_buff = usb_req->buf;
			ep->dwc_ep.xfer_buff = usb_req->buf;
			ep->dwc_ep.xfer_len = usb_req->length;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = ep->dwc_ep.xfer_len;
			if(usb_req->zero) {
				if((ep->dwc_ep.xfer_len % ep->dwc_ep.maxpacket == 0)
						&& (ep->dwc_ep.xfer_len != 0)) {
					ep->dwc_ep.sent_zlp = 1;
				}
			}

			dwc_otg_ep0_start_transfer(GET_CORE_IF(pcd), &ep->dwc_ep);
		}
		else {
			/* Setup and start the Transfer */
			/* Configure DMA address */
			if (GET_CORE_IF(pcd)->dma_enable) {
				if (usb_req->dma == DMA_ADDR_INVALID) {
					ep->dwc_ep.dma_addr = dma_map_single(
						NULL,
						usb_req->buf,
						usb_req->length,
						ep->dwc_ep.is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
				} else {
					dma_sync_single_for_device(
						NULL,
						usb_req->dma, usb_req->length,
						ep->dwc_ep.is_in ? DMA_TO_DEVICE : DMA_FROM_DEVICE);
					ep->dwc_ep.dma_addr = usb_req->dma;
				}
			}

			ep->dwc_ep.start_xfer_buff = usb_req->buf;
			ep->dwc_ep.xfer_buff = usb_req->buf;
			ep->dwc_ep.xfer_len = usb_req->length;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = ep->dwc_ep.xfer_len;
			if(usb_req->zero) {
				if((ep->dwc_ep.xfer_len % ep->dwc_ep.maxpacket == 0)
						&& (ep->dwc_ep.xfer_len != 0)) {
					ep->dwc_ep.sent_zlp = 1;
				}
			}
			dwc_otg_ep_start_transfer(GET_CORE_IF(pcd), &ep->dwc_ep);
		}
	}

	if ((req != 0) || prevented) {
		++pcd->request_pending;
		list_add_tail(&req->queue, &ep->queue);
		if (ep->dwc_ep.is_in && ep->stopped && !(GET_CORE_IF(pcd)->dma_enable)) {
			/** @todo NGS Create a function for this. */
			diepmsk_data_t diepmsk = { .d32 = 0};
			diepmsk.b.intktxfemp = 1;
			dwc_modify_reg32(&GET_CORE_IF(pcd)->dev_if->dev_global_regs->diepmsk, 0, diepmsk.d32);
		}
	}

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
	return 0;
}

/**
 * This function cancels an I/O request from an EP.
 */
static int dwc_otg_pcd_ep_dequeue(struct usb_ep *usb_ep,
				  struct usb_request *usb_req)
{
	dwc_otg_pcd_request_t *req;
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t	*pcd;
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p,%p)\n", __func__, usb_ep, usb_req);

	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);
	if (!usb_ep || !usb_req || (!ep->desc && ep->dwc_ep.num != 0)) {
		DWC_WARN("%s, bad argument\n", __func__);
		return -EINVAL;
	}
	pcd = ep->pcd;
	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) {
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);
	DWC_DEBUGPL(DBG_PCDV, "%s %s %s %p\n", __func__, usb_ep->name,
					ep->dwc_ep.is_in ? "IN" : "OUT",
					usb_req);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue)
	{
		if (&req->req == usb_req) {
			break;
		}
	}

	if (&req->req != usb_req) {
		SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
		return -EINVAL;
	}

	if (!list_empty(&req->queue)) {
		request_done(ep, req, -ECONNRESET);
	}
	else {
		req = 0;
	}

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	return req ? 0 : -EOPNOTSUPP;
}

/**
 * usb_ep_set_halt stalls an endpoint.
 *
 * usb_ep_clear_halt clears an endpoint halt and resets its data
 * toggle.
 *
 * Both of these functions are implemented with the same underlying
 * function. The behavior depends on the value argument.
 *
 * @param[in] usb_ep the Endpoint to halt or clear halt.
 * @param[in] value
 *	- 0 means clear_halt.
 *	- 1 means set_halt,
 *	- 2 means clear stall lock flag.
 *	- 3 means set  stall lock flag.
 */
static int dwc_otg_pcd_ep_set_halt(struct usb_ep *usb_ep, int value)
{
	int retval = 0;
	unsigned long flags;
	dwc_otg_pcd_ep_t *ep = 0;


	DWC_DEBUGPL(DBG_PCD,"HALT %s %d\n", usb_ep->name, value);

	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);

	if (!usb_ep || (!ep->desc && ep != &ep->pcd->ep0) ||
			ep->desc->bmAttributes == USB_ENDPOINT_XFER_ISOC) {
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}

	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);
	if (!list_empty(&ep->queue)) {
		DWC_WARN("%s() %s XFer In process\n", __func__, usb_ep->name);
		retval = -EAGAIN;
	}
	else if (value == 0) {
		dwc_otg_ep_clear_stall(ep->pcd->otg_dev->core_if,
					&ep->dwc_ep);
	}
	else if(value == 1) {
		if (ep->dwc_ep.is_in == 1 && ep->pcd->otg_dev->core_if->dma_desc_enable) {
			dtxfsts_data_t txstatus;
			fifosize_data_t txfifosize;

			txfifosize.d32 = dwc_read_reg32(&ep->pcd->otg_dev->core_if->core_global_regs->dptxfsiz_dieptxf[ep->dwc_ep.tx_fifo_num]);
			txstatus.d32 = dwc_read_reg32(&ep->pcd->otg_dev->core_if->dev_if->in_ep_regs[ep->dwc_ep.num]->dtxfsts);

			if(txstatus.b.txfspcavail < txfifosize.b.depth) {
				DWC_WARN("%s() %s Data In Tx Fifo\n", __func__, usb_ep->name);
				retval = -EAGAIN;
			}
			else {
				if (ep->dwc_ep.num == 0) {
					ep->pcd->ep0state = EP0_STALL;
				}

				ep->stopped = 1;
				dwc_otg_ep_set_stall(ep->pcd->otg_dev->core_if,
							&ep->dwc_ep);
			}
		}
		else {
			if (ep->dwc_ep.num == 0) {
				ep->pcd->ep0state = EP0_STALL;
			}

			ep->stopped = 1;
			dwc_otg_ep_set_stall(ep->pcd->otg_dev->core_if,
						&ep->dwc_ep);
		}
	}
	else if (value == 2) {
		ep->dwc_ep.stall_clear_flag = 0;
	}
	else if (value == 3) {
		ep->dwc_ep.stall_clear_flag = 1;
	}

	SPIN_UNLOCK_IRQRESTORE(&ep->pcd->lock, flags);
	return retval;
}

#ifdef _EN_ISOC_

/**
 * This function is used to submit an ISOC Transfer Request to an EP.
 *
 *	- Every time a sync period completes the request's completion callback
 *	  is called to provide data to the gadget driver.
 *	- Once submitted the request cannot be modified.
 *	- Each request is turned into periodic data packets untill ISO
 *	  Transfer is stopped..
 */
static int dwc_otg_pcd_iso_ep_start(struct usb_ep *usb_ep, struct usb_iso_request *req,
				    gfp_t gfp_flags)
{
	dwc_otg_pcd_ep_t 	*ep;
	dwc_otg_pcd_t		*pcd;
	dwc_ep_t 		*dwc_ep;
	dsts_data_t 		dsts = { .d32 = 0};
	depctl_data_t 		depctl = { .d32 = 0 };
	volatile uint32_t 	*addr;
	unsigned long 		flags = 0;
	int32_t 		frm_data;
 	int 			i, j;
	dwc_otg_core_if_t	*core_if;
	dcfg_data_t		dcfg;

	if (!req || !req->process_buffer || !req->buf0 || !req->buf1) {
		DWC_WARN("%s, bad params\n", __func__);
		return -EINVAL;
	}

	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);

	if (!usb_ep || !ep->desc || ep->dwc_ep.num == 0) {
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}

	pcd = ep->pcd;
	core_if = GET_CORE_IF(pcd);

	dcfg.d32 = dwc_read_reg32(&core_if->dev_if->dev_global_regs->dcfg);

	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) {
		DWC_DEBUGPL(DBG_PCDV, "gadget.speed=%d\n", pcd->gadget.speed);
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	SPIN_LOCK_IRQSAVE(&ep->pcd->lock, flags);

	dwc_ep = &ep->dwc_ep;

	req->status = -EINPROGRESS;

	dwc_ep->dma_addr0 = req->dma0;
	dwc_ep->dma_addr1 = req->dma1;

	dwc_ep->xfer_buff0 = req->buf0;
	dwc_ep->xfer_buff1 = req->buf1;

	ep->iso_req = req;

	dwc_ep->data_per_frame = req->data_per_frame;

	/* todo - pattern data support is to be implemented in the future */
	dwc_ep->data_pattern_frame = req->data_pattern_frame;
	dwc_ep->sync_frame = req->sync_frame;

	dwc_ep->buf_proc_intrvl = req->buf_proc_intrvl;

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	dwc_ep->bInterval = 1 << (ep->desc->bInterval - 1);

	dwc_ep->proc_buf_num = 0;

	dwc_ep->pkt_per_frm = 0;
	frm_data = ep->dwc_ep.data_per_frame;

	while(frm_data > 0) {
		dwc_ep->pkt_per_frm++;
		frm_data -= ep->dwc_ep.maxpacket;
	}

	if(dwc_ep->is_in)
		dwc_ep->desc_cnt = dwc_ep->buf_proc_intrvl / dwc_ep->bInterval;
	else
		dwc_ep->desc_cnt = dwc_ep->buf_proc_intrvl * dwc_ep->pkt_per_frm / dwc_ep->bInterval;

	/** Allocate descriptors for double buffering */
	dwc_ep->iso_desc_addr = ep_alloc_iso_desc_chain(&dwc_ep->iso_dma_desc_addr,dwc_ep->desc_cnt*2);
	if(dwc_ep->desc_addr) {
		DWC_WARN("%s, can't allocate DMA descriptor chain\n", __func__);
		return -ENOMEM;
	}

	dsts.d32 = dwc_read_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dsts));

	/** ISO OUT EP */
	if(dwc_ep->is_in == 0) {
		iso_out_sts_data_t sts = { .d32 =0 };
		dwc_otg_iso_dma_desc_t* dma_desc = dwc_ep->iso_desc_addr;
		dma_addr_t dma_ad;
		uint32_t data_per_desc;
		dwc_otg_dev_out_ep_regs_t *out_regs =
			core_if->dev_if->out_ep_regs[dwc_ep->num];
		struct usb_gadget_iso_packet_descriptor *iso_packet;
		int	offset;

		addr = &GET_CORE_IF(pcd)->dev_if->out_ep_regs[dwc_ep->num]->doepctl;
		dma_ad = (dma_addr_t)dwc_read_reg32(&(out_regs->doepdma));

		/** Buffer 0 descriptors setup */
		dma_ad = dwc_ep->dma_addr0;

		sts.b.bs = BS_HOST_READY;
		sts.b.rxsts = 0;
		sts.b.l = 0;
		sts.b.sp = 0;
		sts.b.ioc = 0;
		sts.b.pid = 0;
		sts.b.framenum = 0;

		iso_packet = ep->iso_req->iso_packet_desc0;
		offset = 0;
		for(i = 0; i < dwc_ep->desc_cnt - dwc_ep->pkt_per_frm; i+= dwc_ep->pkt_per_frm)
		{
			for(j = 0; j < dwc_ep->pkt_per_frm; ++j)
			{
				data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
					dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;

				data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
				sts.b.rxbytes = data_per_desc;
				writel((uint32_t)dma_ad, &dma_desc->buf);
				writel(sts.d32, &dma_desc->status);
				iso_packet->offset = offset;

				offset += data_per_desc;
				dma_desc ++;
				(uint32_t)dma_ad += data_per_desc;
				++iso_packet;
			}
		}

		for(j = 0; j < dwc_ep->pkt_per_frm - 1; ++j)
		{
			data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
				dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;
			data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
			sts.b.rxbytes = data_per_desc;
			writel((uint32_t)dma_ad, &dma_desc->buf);
			writel(sts.d32, &dma_desc->status);
			iso_packet->offset = offset;

			offset += data_per_desc;
			dma_desc ++;
			(uint32_t)dma_ad += data_per_desc;
			++iso_packet;
		}

		sts.b.ioc = 1;
		data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
			dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;
		data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
		sts.b.rxbytes = data_per_desc;

		writel((uint32_t)dma_ad, &dma_desc->buf);
		writel(sts.d32, &dma_desc->status);
		iso_packet->offset = offset;
		dma_desc ++;

		/** Buffer 1 descriptors setup */
		sts.b.ioc = 0;
		dma_ad = dwc_ep->dma_addr1;

		iso_packet = ep->iso_req->iso_packet_desc1;
		offset = 0;
		for(i = 0; i < dwc_ep->desc_cnt - dwc_ep->pkt_per_frm; i+= dwc_ep->pkt_per_frm)
		{
			for(j = 0; j < dwc_ep->pkt_per_frm; ++j)
			{
				data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
					dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;
				data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
				sts.b.rxbytes = data_per_desc;
				writel((uint32_t)dma_ad, &dma_desc->buf);
				writel(sts.d32, &dma_desc->status);
				iso_packet->offset = offset;

				offset += data_per_desc;
				dma_desc ++;
				(uint32_t)dma_ad += data_per_desc;
				++iso_packet;
			}
		}
		for(j = 0; j < dwc_ep->pkt_per_frm - 1; ++j)
		{
			data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
				dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;
			data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
			sts.b.rxbytes = data_per_desc;
			writel((uint32_t)dma_ad, &dma_desc->buf);
			writel(sts.d32, &dma_desc->status);
			iso_packet->offset = offset;

			offset += data_per_desc;
			dma_desc ++;
			(uint32_t)dma_ad += data_per_desc;
			++iso_packet;
		}

		sts.b.ioc = 1;
		sts.b.l = 1;
		data_per_desc = ((j + 1) * dwc_ep->maxpacket > dwc_ep->data_per_frame) ?
			dwc_ep->data_per_frame - j * dwc_ep->maxpacket : dwc_ep->maxpacket;
		data_per_desc += (data_per_desc % 4) ? (4 - data_per_desc % 4):0;
		sts.b.rxbytes = data_per_desc;

		writel((uint32_t)dma_ad, &dma_desc->buf);
		writel(sts.d32, &dma_desc->status);
		iso_packet->offset = offset;

		dwc_ep->next_frame = 0;

		/** Write dma_ad into DOEPDMA register */
		dwc_write_reg32(&(out_regs->doepdma),(uint32_t)dwc_ep->iso_dma_desc_addr);

	}
	/** ISO IN EP */
	else {
		iso_in_sts_data_t sts = { .d32 =0 };
		dwc_otg_iso_dma_desc_t* dma_desc = dwc_ep->iso_desc_addr;
		dma_addr_t dma_ad;
		dwc_otg_dev_in_ep_regs_t *in_regs =
			core_if->dev_if->in_ep_regs[dwc_ep->num];
		unsigned int		   frmnumber;
		fifosize_data_t		txfifosize,rxfifosize;

		txfifosize.d32 = dwc_read_reg32(&core_if->dev_if->in_ep_regs[dwc_ep->num]->dtxfsts);
		rxfifosize.d32 = dwc_read_reg32(&core_if->core_global_regs->grxfsiz);


		addr = &GET_CORE_IF(pcd)->dev_if->in_ep_regs[dwc_ep->num]->diepctl;

		dma_ad = dwc_ep->dma_addr0;

		dsts.d32 = dwc_read_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dsts));

		sts.b.bs = BS_HOST_READY;
		sts.b.txsts = 0;
		sts.b.sp = (dwc_ep->data_per_frame % dwc_ep->maxpacket)? 1 : 0;
		sts.b.ioc = 0;
		sts.b.pid = dwc_ep->pkt_per_frm;

		if(req->flags & USB_REQ_ISO_ASAP) {
			frmnumber = dsts.b.soffn  + 1;
			if(dwc_ep->bInterval != 1){
				frmnumber = frmnumber + (dwc_ep->bInterval - 1 - frmnumber%dwc_ep->bInterval);
			}
		} else {
			frmnumber = req->start_frame;
		}

		sts.b.framenum = frmnumber;
		sts.b.txbytes = dwc_ep->data_per_frame;
		sts.b.l = 0;

		/** Buffer 0 descriptors setup */
		for(i = 0; i < dwc_ep->desc_cnt - 1; i++)
		{
			writel((uint32_t)dma_ad, &dma_desc->buf);
			writel(sts.d32, &dma_desc->status);
			dma_desc ++;

			(uint32_t)dma_ad += dwc_ep->data_per_frame;
			sts.b.framenum += dwc_ep->bInterval;
		}

		sts.b.ioc = 1;
		writel((uint32_t)dma_ad, &dma_desc->buf);
		writel(sts.d32, &dma_desc->status);
		++dma_desc;

		/** Buffer 1 descriptors setup */
		sts.b.ioc = 0;
		dma_ad = dwc_ep->dma_addr1;

		for(i = 0; i < dwc_ep->desc_cnt - dwc_ep->pkt_per_frm; i+= dwc_ep->pkt_per_frm)
		{
			writel((uint32_t)dma_ad, &dma_desc->buf);
			writel(sts.d32, &dma_desc->status);
			dma_desc ++;

			(uint32_t)dma_ad += dwc_ep->data_per_frame;
			sts.b.framenum += dwc_ep->bInterval;

			sts.b.ioc = 0;
		}
		sts.b.ioc = 1;
		sts.b.l = 1;

		writel((uint32_t)dma_ad, &dma_desc->buf);
		writel(sts.d32, &dma_desc->status);

		dwc_ep->next_frame = sts.b.framenum + dwc_ep->bInterval;

		/** Write dma_ad into diepdma register */
		dwc_write_reg32(&(in_regs->diepdma),(uint32_t)dwc_ep->iso_dma_desc_addr);
	}
	/** Enable endpoint, clear nak  */
	depctl.d32 = 0;
	depctl.b.epena = 1;
	depctl.b.usbactep = 1;
	depctl.b.cnak = 1;

	dwc_modify_reg32(addr, depctl.d32,depctl.d32);
	depctl.d32 = dwc_read_reg32(addr);
	return 0;
}

/**
 * This function stops ISO EP Periodic Data Transfer.
 */
static int dwc_otg_pcd_iso_ep_stop(struct usb_ep *usb_ep, struct usb_iso_request *req)
{
	dwc_otg_pcd_ep_t *ep;
	dwc_otg_pcd_t	*pcd;
	dwc_ep_t *dwc_ep;
	unsigned long flags;
	depctl_data_t depctl = { .d32 = 0 };
	volatile uint32_t *addr;


	ep = container_of(usb_ep, dwc_otg_pcd_ep_t, ep);

	if (!usb_ep || !ep->desc || ep->dwc_ep.num == 0) {
		DWC_WARN("%s, bad ep\n", __func__);
		return -EINVAL;
	}

	pcd = ep->pcd;

	if (!pcd->driver || pcd->gadget.speed == USB_SPEED_UNKNOWN) {
		DWC_DEBUGPL(DBG_PCDV, "gadget.speed=%d\n", pcd->gadget.speed);
		DWC_WARN("%s, bogus device state\n", __func__);
		return -ESHUTDOWN;
	}

	dwc_ep = &ep->dwc_ep;

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);

	if(ep->iso_req != req) {
		return -EINVAL;
	}

	req->status = -ECONNRESET;

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	if(ep->dwc_ep.is_in == 1) {
		addr = &GET_CORE_IF(pcd)->dev_if->in_ep_regs[dwc_ep->num]->diepctl;
	}
	else {
		addr = &GET_CORE_IF(pcd)->dev_if->out_ep_regs[dwc_ep->num]->doepctl;
	}

	depctl.d32 = dwc_read_reg32(addr);

	dwc_write_reg32(addr, depctl.d32);

	ep_free_iso_desc_chain(ep->dwc_ep.iso_desc_addr,ep->dwc_ep.iso_dma_desc_addr,ep->dwc_ep.desc_cnt * 2);

	/* reset varibales */
	dwc_ep->dma_addr0 = 0;
	dwc_ep->dma_addr1 = 0;
	dwc_ep->xfer_buff0 = 0;
	dwc_ep->xfer_buff1 = 0;
	dwc_ep->data_per_frame = 0;
	dwc_ep->data_pattern_frame = 0;
	dwc_ep->sync_frame = 0;
	dwc_ep->buf_proc_intrvl = 0;
	dwc_ep->bInterval = 0;
	dwc_ep->proc_buf_num = 0;
	dwc_ep->pkt_per_frm = 0;
	dwc_ep->pkt_per_frm = 0;
	dwc_ep->desc_cnt = 	0;
	dwc_ep->iso_desc_addr = 0;
	dwc_ep->iso_dma_desc_addr = 0;

	return 0;
}


static struct usb_iso_request *dwc_otg_pcd_alloc_iso_request(struct usb_ep *ep,int packets,
							     gfp_t gfp_flags)
{
	struct usb_iso_request	*pReq = NULL;
	uint32_t		req_size;


	req_size = sizeof(struct usb_iso_request);
	req_size += (2 * packets * (sizeof(struct usb_gadget_iso_packet_descriptor)));

	pReq = kmalloc(req_size, gfp_flags);
	if (!pReq) {
		DWC_WARN("%s, can't allocate Iso Request\n", __func__);
		return 0;
	}
	pReq->iso_packet_desc0 = (void*) (pReq +  1);
	pReq->iso_packet_desc1 = pReq->iso_packet_desc0 + packets;

	return pReq;
}

static void dwc_otg_pcd_free_iso_request(struct usb_ep *ep, struct usb_iso_request *req)
{
	kfree(req);
}

static struct usb_isoc_ep_ops dwc_otg_pcd_ep_ops =
{
	.ep_ops =
	{
		.enable		= dwc_otg_pcd_ep_enable,
		.disable	= dwc_otg_pcd_ep_disable,

		.alloc_request	= dwc_otg_pcd_alloc_request,
		.free_request	= dwc_otg_pcd_free_request,

		.alloc_buffer	= dwc_otg_pcd_alloc_buffer,
		.free_buffer	= dwc_otg_pcd_free_buffer,

		.queue		= dwc_otg_pcd_ep_queue,
		.dequeue	= dwc_otg_pcd_ep_dequeue,

		.set_halt	= dwc_otg_pcd_ep_set_halt,
		.fifo_status	= 0,
		.fifo_flush = 0,
	},
	.iso_ep_start		= dwc_otg_pcd_iso_ep_start,
	.iso_ep_stop		= dwc_otg_pcd_iso_ep_stop,
	.alloc_iso_request 	= dwc_otg_pcd_alloc_iso_request,
	.free_iso_request	= dwc_otg_pcd_free_iso_request,
};

#else


static struct usb_ep_ops dwc_otg_pcd_ep_ops =
{
	.enable		= dwc_otg_pcd_ep_enable,
	.disable	= dwc_otg_pcd_ep_disable,

	.alloc_request	= dwc_otg_pcd_alloc_request,
	.free_request	= dwc_otg_pcd_free_request,

	.queue		= dwc_otg_pcd_ep_queue,
	.dequeue	= dwc_otg_pcd_ep_dequeue,

	.set_halt	= dwc_otg_pcd_ep_set_halt,
	.fifo_status	= 0,
	.fifo_flush = 0,

};

#endif /* _EN_ISOC_ */

/* Gadget Operations */

/**
 * The following gadget operations will be implemented in the DWC_otg
 * PCD. Functions in the API that are not described below are not
 * implemented.
 *
 * The Gadget API provides wrapper functions for each of the function
 * pointers defined in usb_gadget_ops. The Gadget Driver calls the
 * wrapper function, which then calls the underlying PCD function. The
 * following sections are named according to the wrapper functions
 * (except for ioctl, which doesn't have a wrapper function). Within
 * each section, the corresponding DWC_otg PCD function name is
 * specified.
 *
 */

/**
 *Gets the USB Frame number of the last SOF.
 */
static int dwc_otg_pcd_get_frame(struct usb_gadget *gadget)
{
	dwc_otg_pcd_t *pcd;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, gadget);

	if (gadget == 0) {
		return -ENODEV;
	}
	else {
		pcd = container_of(gadget, dwc_otg_pcd_t, gadget);
		dwc_otg_get_frame_number(GET_CORE_IF(pcd));
	}

	return 0;
}


void dwc_otg_pcd_remote_wakeup(dwc_otg_pcd_t *pcd, int set)
{
	dctl_data_t dctl = {.d32=0};
	volatile uint32_t *addr = &(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dctl);

	if (dwc_otg_is_device_mode(GET_CORE_IF(pcd))) {
		if (pcd->remote_wakeup_enable) {
			if (set) {
				dctl.b.rmtwkupsig = 1;
				dwc_modify_reg32(addr, 0, dctl.d32);
				DWC_DEBUGPL(DBG_PCD, "Set Remote Wakeup\n");
				msleep(1);
				dwc_modify_reg32(addr, dctl.d32, 0);
				DWC_DEBUGPL(DBG_PCD, "Clear Remote Wakeup\n");
			}
			else {
			}
		}
		else {
			DWC_DEBUGPL(DBG_PCD, "Remote Wakeup is disabled\n");
		}
	}
	return;
}

/**
 * Initiates Session Request Protocol (SRP) to wakeup the host if no
 * session is in progress. If a session is already in progress, but
 * the device is suspended, remote wakeup signaling is started.
 *
 */
static int dwc_otg_pcd_wakeup(struct usb_gadget *gadget)
{
	unsigned long flags;
	dwc_otg_pcd_t *pcd;
	dsts_data_t dsts;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, gadget);

	if (gadget == 0) {
		return -ENODEV;
	}
	else {
		pcd = container_of(gadget, dwc_otg_pcd_t, gadget);
	}
	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);

	dsts.d32 = dwc_read_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dsts));
	if (dsts.b.suspsts) {
		dwc_otg_pcd_remote_wakeup(pcd, 1);
	}

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);
	return 0;
}


static int dwc_otg_pcd_pullup(struct usb_gadget *gadget, int is_on)
{
	dwc_otg_pcd_t *pcd;
	dctl_data_t dctl = {.d32=0};
	unsigned long flags;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, gadget);

	if (!gadget) {
		return -ENODEV;
	}

	pcd = container_of(gadget, dwc_otg_pcd_t, gadget);

	SPIN_LOCK_IRQSAVE(&pcd->lock, flags);

	if (is_on) {
		dctl.b.sftdiscon = 1;
		dwc_modify_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dctl), dctl.d32, 0);
	} else {
		dctl.b.sftdiscon = 1;
		dwc_modify_reg32(&(GET_CORE_IF(pcd)->dev_if->dev_global_regs->dctl), 0, dctl.d32);
	}

	SPIN_UNLOCK_IRQRESTORE(&pcd->lock, flags);

	return 0;
}

static const struct usb_gadget_ops dwc_otg_pcd_ops =
{
	.get_frame	 = dwc_otg_pcd_get_frame,
	.wakeup		 = dwc_otg_pcd_wakeup,
	// current versions must always be self-powered
	.pullup		 = dwc_otg_pcd_pullup,
};

/**
 * This function is the top level PCD interrupt handler.
 */
static irqreturn_t dwc_otg_pcd_irq(int irq, void *dev)
{
	dwc_otg_pcd_t *pcd = dev;
	int32_t retval = IRQ_NONE;

	retval = dwc_otg_pcd_handle_intr(pcd);
	return IRQ_RETVAL(retval);
}


/**
 * Tasklet
 *
 */
extern void start_next_request(dwc_otg_pcd_ep_t *ep);

static void start_xfer_tasklet_func (unsigned long data)
{
	dwc_otg_pcd_t *pcd = (dwc_otg_pcd_t*)data;
	dwc_otg_core_if_t *core_if = pcd->otg_dev->core_if;

	int i;
	depctl_data_t diepctl;

	DWC_DEBUGPL(DBG_PCDV, "Start xfer tasklet\n");

	diepctl.d32 = dwc_read_reg32(&core_if->dev_if->in_ep_regs[0]->diepctl);

	if (pcd->ep0.queue_sof) {
		pcd->ep0.queue_sof = 0;
		start_next_request (&pcd->ep0);
		// break;
	}

	for (i=0; i<core_if->dev_if->num_in_eps; i++)
	{
		depctl_data_t diepctl;
		diepctl.d32 = dwc_read_reg32(&core_if->dev_if->in_ep_regs[i]->diepctl);

		if (pcd->in_ep[i].queue_sof) {
			pcd->in_ep[i].queue_sof = 0;
			start_next_request (&pcd->in_ep[i]);
			// break;
		}
	}

	return;
}

static struct tasklet_struct start_xfer_tasklet[CTRL_NUM] =
{
	{
		.next = NULL,
		.state = 0,
		.count = ATOMIC_INIT(0),
		.func = start_xfer_tasklet_func,
		.data = 0,
	},
	{
		.next = NULL,
		.state = 0,
		.count = ATOMIC_INIT(0),
		.func = start_xfer_tasklet_func,
		.data = 0,
	},
};

/**
 * This function initialized the pcd Dp structures to there default
 * state.
 *
 * @param pcd the pcd structure.
 */
void dwc_otg_pcd_reinit(dwc_otg_pcd_t *pcd)
{
	static const char * names[] =
		{
			"ep0",
			"ep1in",
			"ep2in",
			"ep3in",
			"ep4in",
			"ep5in",
			"ep6in",
			"ep7in",
			"ep8in",
			"ep9in",
			"ep10in",
			"ep11in",
			"ep12in",
			"ep13in",
			"ep14in",
			"ep15in",
			"ep1out",
			"ep2out",
			"ep3out",
			"ep4out",
			"ep5out",
			"ep6out",
			"ep7out",
			"ep8out",
			"ep9out",
			"ep10out",
			"ep11out",
			"ep12out",
			"ep13out",
			"ep14out",
			"ep15out"
	};

	int i;
	int in_ep_cntr, out_ep_cntr;
	uint32_t hwcfg1;
	uint32_t num_in_eps = (GET_CORE_IF(pcd))->dev_if->num_in_eps;
	uint32_t num_out_eps = (GET_CORE_IF(pcd))->dev_if->num_out_eps;
	dwc_otg_pcd_ep_t *ep;

	DWC_DEBUGPL(DBG_PCDV, "%s(%p)\n", __func__, pcd);

	INIT_LIST_HEAD (&pcd->gadget.ep_list);
	pcd->gadget.ep0 = &pcd->ep0.ep;
	pcd->gadget.speed = USB_SPEED_UNKNOWN;

	INIT_LIST_HEAD (&pcd->gadget.ep0->ep_list);

	/**
	 * Initialize the EP0 structure.
	 */
	ep = &pcd->ep0;

	/* Init EP structure */
	ep->desc = 0;
	ep->pcd = pcd;
	ep->stopped = 1;

	/* Init DWC ep structure */
	ep->dwc_ep.num = 0;
	ep->dwc_ep.active = 0;
	ep->dwc_ep.tx_fifo_num = 0;
	/* Control until ep is actvated */
	ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
	ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
	ep->dwc_ep.dma_addr = 0;
	ep->dwc_ep.start_xfer_buff = 0;
	ep->dwc_ep.xfer_buff = 0;
	ep->dwc_ep.xfer_len = 0;
	ep->dwc_ep.xfer_count = 0;
	ep->dwc_ep.sent_zlp = 0;
	ep->dwc_ep.total_len = 0;
	ep->queue_sof = 0;

	/* Init the usb_ep structure. */
	ep->ep.name = names[0];
	ep->ep.ops = (struct usb_ep_ops*)&dwc_otg_pcd_ep_ops;

	/**
	 * @todo NGS: What should the max packet size be set to
	 * here?  Before EP type is set?
	 */
	ep->ep.maxpacket = MAX_PACKET_SIZE;

	list_add_tail (&ep->ep.ep_list, &pcd->gadget.ep_list);

	INIT_LIST_HEAD (&ep->queue);
	/**
	 * Initialize the EP structures.
	 */
	in_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 3;

	for (i = 1; in_ep_cntr < num_in_eps; i++)
	{
		if((hwcfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *ep = &pcd->in_ep[in_ep_cntr];
			in_ep_cntr ++;

			/* Init EP structure */
			ep->desc = 0;
			ep->pcd = pcd;
			ep->stopped = 1;

			/* Init DWC ep structure */
			ep->dwc_ep.is_in = 1;
			ep->dwc_ep.num = i;
			ep->dwc_ep.active = 0;
			ep->dwc_ep.tx_fifo_num = 0;

			/* Control until ep is actvated */
			ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
			ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
			ep->dwc_ep.dma_addr = 0;
			ep->dwc_ep.start_xfer_buff = 0;
			ep->dwc_ep.xfer_buff = 0;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = 0;
			ep->queue_sof = 0;

			/* Init the usb_ep structure. */
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			ep->ep.name = names[i];
			ep->ep.ops = (struct usb_ep_ops*)&dwc_otg_pcd_ep_ops;

			/**
			 * @todo NGS: What should the max packet size be set to
			 * here?  Before EP type is set?
			 */
			ep->ep.maxpacket = MAX_PACKET_SIZE;

			list_add_tail (&ep->ep.ep_list, &pcd->gadget.ep_list);
			INIT_LIST_HEAD (&ep->queue);
		}
		hwcfg1 >>= 2;
	}

	out_ep_cntr = 0;
	hwcfg1 = (GET_CORE_IF(pcd))->hwcfg1.d32 >> 2;

	for (i = 1; out_ep_cntr < num_out_eps; i++)
	{
		if((hwcfg1 & 0x1) == 0) {
			dwc_otg_pcd_ep_t *ep = &pcd->out_ep[out_ep_cntr];
			out_ep_cntr++;

			/* Init EP structure */
			ep->desc = 0;
			ep->pcd = pcd;
			ep->stopped = 1;

			/* Init DWC ep structure */
			ep->dwc_ep.is_in = 0;
			ep->dwc_ep.num = i;
			ep->dwc_ep.active = 0;
			ep->dwc_ep.tx_fifo_num = 0;
			/* Control until ep is actvated */
			ep->dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
			ep->dwc_ep.maxpacket = MAX_PACKET_SIZE;
			ep->dwc_ep.dma_addr = 0;
			ep->dwc_ep.start_xfer_buff = 0;
			ep->dwc_ep.xfer_buff = 0;
			ep->dwc_ep.xfer_len = 0;
			ep->dwc_ep.xfer_count = 0;
			ep->dwc_ep.sent_zlp = 0;
			ep->dwc_ep.total_len = 0;
			ep->queue_sof = 0;

			/* Init the usb_ep structure. */
			/**
			 * @todo NGS: Add direction to EP, based on contents
			 * of HWCFG1.  Need a copy of HWCFG1 in pcd structure?
			 * sprintf(";r
			 */
			ep->ep.name = names[15 + i];
			ep->ep.ops = (struct usb_ep_ops*)&dwc_otg_pcd_ep_ops;
			/**
			 * @todo NGS: What should the max packet size be set to
			 * here?  Before EP type is set?
			 */
			ep->ep.maxpacket = MAX_PACKET_SIZE;

			list_add_tail (&ep->ep.ep_list, &pcd->gadget.ep_list);
			INIT_LIST_HEAD (&ep->queue);
		}
		hwcfg1 >>= 2;
	}

	/* remove ep0 from the list.  There is a ep0 pointer.*/
	list_del_init (&pcd->ep0.ep.ep_list);

	pcd->ep0state = EP0_DISCONNECT;
	pcd->ep0.ep.maxpacket = MAX_EP0_SIZE;
	pcd->ep0.dwc_ep.maxpacket = MAX_EP0_SIZE;
	pcd->ep0.dwc_ep.type = DWC_OTG_EP_TYPE_CONTROL;
}

/**
 * This function releases the Gadget device.
 * required by device_unregister().
 *
 * @todo Should this do something?	Should it free the PCD?
 */
static void dwc_otg_pcd_gadget_release(struct device *dev)
{
	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, dev);
}


struct usb_gadget_udc dwc_driver = {
	.register_driver = dwc_usb_gadget_register_driver,
	.unregister_driver = dwc_usb_gadget_unregister_driver,
};
/**
 * This function initialized the PCD portion of the driver.
 *
 */
int dwc_otg_pcd_init(struct platform_device *pdev)
{
	static char pcd_name[] = "dwc_otg_pcd";
	dwc_otg_pcd_t *pcd;
	dwc_otg_core_if_t* core_if;
	dwc_otg_dev_if_t* dev_if;
	dwc_otg_device_t *otg_dev = platform_get_drvdata(pdev);
	int retval = 0;

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n",__func__, pdev);

	/* Allocate PCD structure */
	pcd = kmalloc(sizeof(dwc_otg_pcd_t), GFP_KERNEL);
	if (pcd == 0) {
		return -ENOMEM;
	}

	memset(pcd, 0, sizeof(dwc_otg_pcd_t));
	spin_lock_init(&pcd->lock);

	otg_dev->pcd = pcd;
	s_pcd = pcd;
	if (usb_gadget_register_udc(pcd, &dwc_driver)) {
		return -ENODEV;
	}
	pcd->gadget.name = pcd_name;
	strcpy(pcd->gadget.dev.bus_id, "gadget");

	pcd->otg_dev = otg_dev;

	pcd->gadget.dev.parent = &pdev->dev;
	pcd->gadget.dev.release = dwc_otg_pcd_gadget_release;
	pcd->gadget.ops = &dwc_otg_pcd_ops;

	core_if = GET_CORE_IF(pcd);
	dev_if = core_if->dev_if;

	if(core_if->hwcfg4.b.ded_fifo_en) {
		DWC_PRINT("Dedicated Tx FIFOs mode\n");
	}
	else {
		DWC_PRINT("Shared Tx FIFO mode\n");
	}

	/* If the module is set to FS or if the PHY_TYPE is FS then the gadget
	 * should not report as dual-speed capable.	 replace the following line
	 * with the block of code below it once the software is debugged for
	 * this.  If is_dualspeed = 0 then the gadget driver should not report
	 * a device qualifier descriptor when queried. */
	if ((GET_CORE_IF(pcd)->core_params->speed == DWC_SPEED_PARAM_FULL) ||
		((GET_CORE_IF(pcd)->hwcfg2.b.hs_phy_type == 2) &&
		 (GET_CORE_IF(pcd)->hwcfg2.b.fs_phy_type == 1) &&
		 (GET_CORE_IF(pcd)->core_params->ulpi_fs_ls))) {
		pcd->gadget.is_dualspeed = 0;
	}
	else {
		pcd->gadget.is_dualspeed = 1;
	}

	if ((otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_DEVICE) ||
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_NO_SRP_CAPABLE_HOST) ||
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_SRP_CAPABLE_DEVICE) ||
	(otg_dev->core_if->hwcfg2.b.op_mode == DWC_HWCFG2_OP_MODE_SRP_CAPABLE_HOST)) {
		pcd->gadget.is_otg = 0;
	}
	else {
		pcd->gadget.is_otg = 1;
	}

	pcd->driver = 0;

	/* Register the gadget device */
	retval = device_register(&pcd->gadget.dev);
	if (retval != 0) {
		kfree(pcd);
		return retval;
	}

	/* Initialized the Core for Device mode. */
	if (dwc_otg_is_device_mode(core_if)) {
		dwc_otg_core_dev_init(core_if);
	}

	/* Initialize EP structures */
	dwc_otg_pcd_reinit(pcd);

	/* Setup interupt handler */
	DWC_DEBUGPL(DBG_ANY, "registering handler for irq%d\n", platform_get_irq(pdev, 0));
	retval = request_irq(platform_get_irq(pdev, 0), dwc_otg_pcd_irq,
			     IRQF_SHARED, pcd->gadget.name, pcd);
	if (retval != 0) {
		DWC_ERROR("request of irq%d failed\n", platform_get_irq(pdev, 0));
		device_unregister(&pcd->gadget.dev);
		kfree(pcd);
		return -EBUSY;
	}

	/* Initialize the DMA buffer for SETUP packets */
	if (GET_CORE_IF(pcd)->dma_enable) {
		pcd->setup_pkt = dma_alloc_coherent (NULL, sizeof (*pcd->setup_pkt) * 5, &pcd->setup_pkt_dma_handle, 0);
		if (pcd->setup_pkt == 0) {
			free_irq(platform_get_irq(pdev, 0), pcd);
			device_unregister(&pcd->gadget.dev);
			kfree(pcd);
			return -ENOMEM;
		}

		pcd->status_buf = dma_alloc_coherent (NULL, sizeof (uint16_t), &pcd->status_buf_dma_handle, 0);
		if (pcd->status_buf == 0) {
			dma_free_coherent(NULL, sizeof(*pcd->setup_pkt), pcd->setup_pkt, pcd->setup_pkt_dma_handle);
			free_irq(platform_get_irq(pdev, 0), pcd);
			device_unregister(&pcd->gadget.dev);
			kfree(pcd);
			return -ENOMEM;
		}

		if (GET_CORE_IF(pcd)->dma_desc_enable) {
			dev_if->setup_desc_addr[0] = ep_alloc_desc(&dev_if->dma_setup_desc_addr[0]);
			dev_if->setup_desc_addr[1] = ep_alloc_desc(&dev_if->dma_setup_desc_addr[1]);
			dev_if->in_desc_addr = ep_alloc_desc(&dev_if->dma_in_desc_addr);
			dev_if->out_desc_addr = ep_alloc_desc(&dev_if->dma_out_desc_addr);

			if(dev_if->setup_desc_addr[0] == 0
			|| dev_if->setup_desc_addr[1] == 0
			|| dev_if->in_desc_addr == 0
			|| dev_if->out_desc_addr == 0 ) {

				if(dev_if->out_desc_addr)
					ep_free_desc(dev_if->out_desc_addr, dev_if->dma_out_desc_addr);
				if(dev_if->in_desc_addr)
					ep_free_desc(dev_if->in_desc_addr, dev_if->dma_in_desc_addr);
				if(dev_if->setup_desc_addr[1])
					ep_free_desc(dev_if->setup_desc_addr[1], dev_if->dma_setup_desc_addr[1]);
				if(dev_if->setup_desc_addr[0])
					ep_free_desc(dev_if->setup_desc_addr[0], dev_if->dma_setup_desc_addr[0]);

				dma_free_coherent(NULL, sizeof(*pcd->status_buf), pcd->status_buf, pcd->setup_pkt_dma_handle);
				dma_free_coherent(NULL, sizeof(*pcd->setup_pkt), pcd->setup_pkt, pcd->setup_pkt_dma_handle);

				free_irq(platform_get_irq(pdev, 0), pcd);
				device_unregister(&pcd->gadget.dev);
				kfree(pcd);

				return -ENOMEM;
			}
		}
	}
	else {
		pcd->setup_pkt = kmalloc (sizeof (*pcd->setup_pkt) * 5, GFP_KERNEL);
		if (pcd->setup_pkt == 0) {
			free_irq(platform_get_irq(pdev, 0), pcd);
			device_unregister(&pcd->gadget.dev);
			kfree(pcd);
			return -ENOMEM;
		}

		pcd->status_buf = kmalloc (sizeof (uint16_t), GFP_KERNEL);
		if (pcd->status_buf == 0) {
			kfree(pcd->setup_pkt);
			free_irq(platform_get_irq(pdev, 0), pcd);
			device_unregister(&pcd->gadget.dev);
			kfree(pcd);
			return -ENOMEM;
		}
	}

	/* Initialize tasklet */
	start_xfer_tasklet[pdev->id].data = (unsigned long)pcd;
	pcd->start_xfer_tasklet = &start_xfer_tasklet[pdev->id];

	return 0;
}

/**
 * Cleanup the PCD.
 */
void dwc_otg_pcd_remove(struct platform_device *pdev)
{
	dwc_otg_device_t *otg_dev = platform_get_drvdata(pdev);
	dwc_otg_pcd_t *pcd = otg_dev->pcd;
	dwc_otg_dev_if_t* dev_if = GET_CORE_IF(pcd)->dev_if;

	DWC_DEBUGPL(DBG_PCDV, "%s(%p)\n", __func__, pdev);

	/* Free the IRQ */
	free_irq(platform_get_irq(pdev, 0), pcd);

	 /* start with the driver above us */
	if (pcd->driver) {
		/* should have been done already by driver model core */
		DWC_WARN("driver '%s' is still registered\n",
				 pcd->driver->driver.name);
		usb_gadget_unregister_driver(pcd->driver);
	}
	device_unregister(&pcd->gadget.dev);

	if (GET_CORE_IF(pcd)->dma_enable) {
		dma_free_coherent (NULL, sizeof (*pcd->setup_pkt) * 5, pcd->setup_pkt, pcd->setup_pkt_dma_handle);
		dma_free_coherent (NULL, sizeof (uint16_t), pcd->status_buf, pcd->status_buf_dma_handle);
		if (GET_CORE_IF(pcd)->dma_desc_enable) {
			ep_free_desc(dev_if->setup_desc_addr[0], dev_if->dma_setup_desc_addr[0]);
			ep_free_desc(dev_if->setup_desc_addr[1], dev_if->dma_setup_desc_addr[1]);
			ep_free_desc(dev_if->in_desc_addr, dev_if->dma_in_desc_addr);
			ep_free_desc(dev_if->out_desc_addr, dev_if->dma_out_desc_addr);
		}
	}
	else {
		kfree(pcd->setup_pkt);
		kfree(pcd->status_buf);
	}

	kfree(pcd);
	otg_dev->pcd = 0;
}

/**
 * This function registers a gadget driver with the PCD.
 *
 * When a driver is successfully registered, it will receive control
 * requests including set_configuration(), which enables non-control
 * requests.  then usb traffic follows until a disconnect is reported.
 * then a host may connect again, or the driver might get unbound.
 *
 * @param driver The driver being registered
 */
static int dwc_usb_gadget_register_driver(struct usb_gadget_driver *driver, void *priv)
{
	dwc_otg_core_if_t* core_if;
	dctl_data_t dctl = {.d32 = 0};
	int retval;

	DWC_DEBUGPL(DBG_PCD, "registering gadget driver '%s'\n", driver->driver.name);

	if (!driver || driver->speed == USB_SPEED_UNKNOWN ||
	    !driver->bind ||
	    !driver->disconnect ||
	    !driver->setup) {
		DWC_DEBUGPL(DBG_PCDV,"EINVAL\n");
		return -EINVAL;
	}
	if (s_pcd == 0) {
		DWC_DEBUGPL(DBG_PCDV,"ENODEV\n");
		return -ENODEV;
	}
	if (s_pcd->driver != 0) {
		DWC_DEBUGPL(DBG_PCDV,"EBUSY (%p)\n", s_pcd->driver);
		return -EBUSY;
	}

	/* hook up the driver */
	s_pcd->driver = driver;
	s_pcd->gadget.dev.driver = &driver->driver;

	DWC_DEBUGPL(DBG_PCD, "bind to driver %s\n", driver->driver.name);
	retval = driver->bind(&s_pcd->gadget);
	if (retval) {
		DWC_ERROR("bind to driver %s --> error %d\n",
					driver->driver.name, retval);
		s_pcd->driver = 0;
		s_pcd->gadget.dev.driver = 0;
		return retval;
	}

	/* disable Soft Disconnect */
	core_if = GET_CORE_IF(s_pcd);
	dctl.b.sftdiscon = 1;
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dctl, dctl.d32, 0);

	DWC_DEBUGPL(DBG_ANY, "registered gadget driver '%s'\n",
					driver->driver.name);
	return 0;
}

/**
 * This function unregisters a gadget driver
 *
 * @param driver The driver being unregistered
 */
static int dwc_usb_gadget_unregister_driver(struct usb_gadget_driver *driver, void *priv)
{
	dwc_otg_core_if_t* core_if;
	dctl_data_t dctl = {.d32 = 0};

	DWC_DEBUGPL(DBG_PCDV,"%s(%p)\n", __func__, driver);

	if (s_pcd == 0) {
		DWC_DEBUGPL(DBG_ANY, "%s Return(%d): s_pcd==0\n", __func__,
				-ENODEV);
		return -ENODEV;
	}
	if (driver == 0 || driver != s_pcd->driver) {
		DWC_DEBUGPL(DBG_ANY, "%s Return(%d): driver?\n", __func__,
				-EINVAL);
		return -EINVAL;
	}

	if (driver->disconnect)
		driver->disconnect(&s_pcd->gadget);

	if (driver->unbind)
		driver->unbind(&s_pcd->gadget);

	s_pcd->driver = 0;

	/* disable Soft Disconnect */
	core_if = GET_CORE_IF(s_pcd);
	dctl.b.sftdiscon = 1;
	dwc_modify_reg32(&core_if->dev_if->dev_global_regs->dctl, 0, dctl.d32);

	DWC_DEBUGPL(DBG_ANY, "unregistered driver '%s'\n",
			driver->driver.name);
	return 0;
}

