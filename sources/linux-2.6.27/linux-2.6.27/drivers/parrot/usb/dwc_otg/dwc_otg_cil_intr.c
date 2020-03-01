/* ==========================================================================
 * $File: //dwh/usb_iip/dev/software/otg_ipmate/linux/drivers/dwc_otg_cil_intr.c $
 * $Revision: #15 $
 * $Date: 2008/04/17 $
 * $Change: 1017879 $
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
 * This file contains the Common Interrupt handlers.
 */
#include "linux/dwc_otg_plat.h"
#include "dwc_otg_regs.h"
#include "dwc_otg_cil.h"

/** This function will log a debug message
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_mode_mismatch_intr (dwc_otg_core_if_t *core_if)
{
	gintsts_data_t gintsts;
	DWC_WARN("Mode Mismatch Interrupt: currently in %s mode\n",
		 dwc_otg_mode(core_if) ? "Host" : "Device");

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.modemismatch = 1;
	dwc_write_reg32 (&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}

/**
 * This function handles the OTG Interrupts. It reads the OTG
 * Interrupt Register (GOTGINT) to determine what interrupt has
 * occurred.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_otg_intr(dwc_otg_core_if_t *core_if)
{
        dwc_otg_core_global_regs_t *global_regs = core_if->core_global_regs;
	gotgint_data_t gotgint;

	gotgint.d32 = dwc_read_reg32(&global_regs->gotgint);

	DWC_DEBUGPL(DBG_CIL, "++OTG Interrupt gotgint=%0x [%s]\n",
		    gotgint.d32, dwc_otg_mode(core_if) ? "Host" : "Device");

	/* Clear GOTGINT */
	dwc_write_reg32 (&core_if->core_global_regs->gotgint, gotgint.d32);

	return 1;
}


/**
 * This function handles the Connector ID Status Change Interrupt.  It
 * reads the OTG Interrupt Register (GOTCTL) to determine whether this
 * is a Device to Host Mode transition or a Host Mode to Device
 * Transition.
 *
 * This only occurs when the cable is connected/removed from the PHY
 * connector.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_conn_id_status_change_intr(dwc_otg_core_if_t *core_if)
{
	gintsts_data_t gintsts = { .d32 = 0 };

	DWC_WARN(" ++Connector ID Status Change Interrupt++  (%s)\n",
		 (dwc_otg_is_host_mode(core_if)?"Host":"Device"));

	/* Set flag and clear interrupt */
	gintsts.b.conidstschng = 1;
	dwc_write_reg32 (&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 * This interrupt indicates that a device is initiating the Session
 * Request Protocol to request the host to turn on bus power so a new
 * session can begin. The handler responds by turning on bus power. If
 * the DWC_otg controller is in low power mode, the handler brings the
 * controller out of low power mode before turning on bus power.
 *
 * @param core_if Programming view of DWC_otg controller.
 */
int32_t dwc_otg_handle_session_req_intr(dwc_otg_core_if_t *core_if)
{
    	gintsts_data_t gintsts;

	DWC_DEBUGPL(DBG_ANY, "++Session Request Interrupt++\n");

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.sessreqintr = 1;
	dwc_write_reg32 (&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}


/**
 * This interrupt indicates that the DWC_otg controller has detected a
 * resume or remote wakeup sequence. If the DWC_otg controller is in
 * low power mode, the handler must brings the controller out of low
 * power mode. The controller automatically begins resume
 * signaling. The handler schedules a time to stop resume signaling.
 */
int32_t dwc_otg_handle_wakeup_detected_intr(dwc_otg_core_if_t *core_if)
{
	gintsts_data_t gintsts;

	DWC_DEBUGPL(DBG_ANY, "++Resume and Remote Wakeup Detected Interrupt++\n");

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.wkupintr = 1;
	dwc_write_reg32 (&core_if->core_global_regs->gintsts, gintsts.d32);

	return 1;
}

/**
 * This interrupt indicates that a device has been disconnected from
 * the root port.
 */
int32_t dwc_otg_handle_disconnect_intr(dwc_otg_core_if_t *core_if)
{
	gintsts_data_t gintsts;

	DWC_DEBUGPL(DBG_ANY, "++Disconnect Detected Interrupt++ (%s)\n",
                    (dwc_otg_is_host_mode(core_if)?"Host":"Device"));

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.disconnect = 1;
	dwc_write_reg32 (&core_if->core_global_regs->gintsts, gintsts.d32);
	return 1;
}
/**
 * This interrupt indicates that SUSPEND state has been detected on
 * the USB.
 *
 * For HNP the USB Suspend interrupt signals the change from
 * "a_peripheral" to "a_host".
 *
 * When power management is enabled the core will be put in low power
 * mode.
 */
int32_t dwc_otg_handle_usb_suspend_intr(dwc_otg_core_if_t *core_if)
{
	gintsts_data_t gintsts;

        DWC_DEBUGPL(DBG_ANY,"USB SUSPEND\n");

	/* Clear interrupt */
	gintsts.d32 = 0;
	gintsts.b.usbsuspend = 1;
	dwc_write_reg32(&core_if->core_global_regs->gintsts, gintsts.d32);

        return 1;
}


/**
 * This function returns the Core Interrupt register.
 */
static inline uint32_t dwc_otg_read_common_intr(dwc_otg_core_if_t *core_if)
{
        gintsts_data_t gintsts;
        gintmsk_data_t gintmsk;
        gintmsk_data_t gintmsk_common = {.d32=0};
	gintmsk_common.b.wkupintr = 1;
	gintmsk_common.b.sessreqintr = 0;
	gintmsk_common.b.conidstschng = 0;
	gintmsk_common.b.otgintr = 0;
	gintmsk_common.b.modemismatch = 0;
        gintmsk_common.b.disconnect = 1;
        gintmsk_common.b.usbsuspend = 1;
        /** @todo: The port interrupt occurs while in device
         * mode. Added code to CIL to clear the interrupt for now!
         */
        gintmsk_common.b.portintr = 1;

        gintsts.d32 = dwc_read_reg32(&core_if->core_global_regs->gintsts);
        gintmsk.d32 = dwc_read_reg32(&core_if->core_global_regs->gintmsk);
#ifdef DEBUG
        /* if any common interrupts set */
        if (gintsts.d32 & gintmsk_common.d32) {
                DWC_DEBUGPL(DBG_ANY, "gintsts=%08x  gintmsk=%08x\n",
                            gintsts.d32, gintmsk.d32);
        }
#endif

        return ((gintsts.d32 & gintmsk.d32) & gintmsk_common.d32);

}

/**
 * Common interrupt handler.
 *
 * The common interrupts are those that occur in both Host and Device mode.
 * This handler handles the following interrupts:
 * - Mode Mismatch Interrupt
 * - Disconnect Interrupt
 * - OTG Interrupt
 * - Connector ID Status Change Interrupt
 * - Session Request Interrupt.
 * - Resume / Remote Wakeup Detected Interrupt.
 *
 */
int32_t dwc_otg_handle_common_intr(dwc_otg_core_if_t *core_if)
{
	int retval = 0;
        gintsts_data_t gintsts;

        gintsts.d32 = dwc_otg_read_common_intr(core_if);

        if (gintsts.b.modemismatch) {
                retval |= dwc_otg_handle_mode_mismatch_intr(core_if);
        }
        if (gintsts.b.otgintr) {
                retval |= dwc_otg_handle_otg_intr(core_if);
        }
        if (gintsts.b.conidstschng) {
                retval |= dwc_otg_handle_conn_id_status_change_intr(core_if);
        }
        if (gintsts.b.disconnect) {
                retval |= dwc_otg_handle_disconnect_intr(core_if);
        }
        if (gintsts.b.sessreqintr) {
                retval |= dwc_otg_handle_session_req_intr(core_if);
        }
        if (gintsts.b.wkupintr) {
                retval |= dwc_otg_handle_wakeup_detected_intr(core_if);
        }
        if (gintsts.b.usbsuspend) {
                retval |= dwc_otg_handle_usb_suspend_intr(core_if);
        }
        if (gintsts.b.portintr && dwc_otg_is_device_mode(core_if)) {
                /* The port interrupt occurs while in device mode with HPRT0
                 * Port Enable/Disable.
                 */
                gintsts.d32 = 0;
                gintsts.b.portintr = 1;
                dwc_write_reg32(&core_if->core_global_regs->gintsts,
                                gintsts.d32);
                retval |= 1;

        }
        return retval;
}
