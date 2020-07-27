/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2011 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <string.h>
#include <libopencm3/cm3/common.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_common.h>
#include "usb_private.h"
#include "usb_dwc_common.h"

#include <libopencm3/stm32/timer.h>

/* The FS core and the HS core have the same register layout.
 * As the code can be used on both cores, the registers offset is modified
 * according to the selected cores base address. */
#define dev_base_address (usbd_dev->driver->base_address)
#define REBASE(x)        MMIO32((x) + (dev_base_address))

enum
{
	FRAME_COUNT	= 1024,
};
static volatile struct trace_frame
{
	enum TRACE_FRAME_TYPE
	{
		TR_INVALID = 0,
		TR_IRQ_ENTER,
		TR_IRQ_EXIT,
		TR_PACKET_DATA_AVAILABLE,
		TR_IN_PACKET_SENT,
		TR_OUT_PACKET_COMPLETE,
		TR_PACKET_RECEIVED,
		TR_GENERIC,
	}
	type;
	uint32_t	time;
	uint32_t	data;
}
trace_frames[FRAME_COUNT];
static int frame_index;

void log_trace_frame(enum TRACE_FRAME_TYPE type, uint32_t data)
{
	if (frame_index >= FRAME_COUNT)
		return;
	trace_frames[frame_index ++] = (struct trace_frame) { .type = type, .time = TIM2_CNT, .data = data, };
}

void dwc_set_address(usbd_device *usbd_dev, uint8_t addr)
{
	REBASE(OTG_DCFG) = (REBASE(OTG_DCFG) & ~OTG_DCFG_DAD) | (addr << 4);
}

void dwc_ep_setup(usbd_device *usbd_dev, uint8_t addr, uint8_t type,
			uint16_t max_size,
			void (*callback) (usbd_device *usbd_dev, uint8_t ep))
{
	/*
	 * Configure endpoint address and type. Allocate FIFO memory for
	 * endpoint. Install callback function.
	 */
	uint8_t dir = addr & 0x80;
	addr &= 0x7f;

	if (addr == 0) { /* For the default control endpoint */
		/* Configure IN part. */
		if (max_size >= 64) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_64;
		} else if (max_size >= 32) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_32;
		} else if (max_size >= 16) {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_16;
		} else {
			REBASE(OTG_DIEPCTL0) = OTG_DIEPCTL0_MPSIZ_8;
		}

		REBASE(OTG_DIEPTSIZ0) =
			(max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DIEPCTL0) |=
			OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_SNAK;

		/* Configure OUT part. */
		usbd_dev->doeptsiz[0] = OTG_DIEPSIZ0_STUPCNT_1 |
			OTG_DIEPSIZ0_PKTCNT |
			(max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DOEPTSIZ(0)) = usbd_dev->doeptsiz[0];
		REBASE(OTG_DOEPCTL(0)) |=
		    OTG_DOEPCTL0_EPENA | OTG_DIEPCTL0_SNAK;

		REBASE(OTG_GNPTXFSIZ) = ((max_size / 4) << 16) |
					 usbd_dev->driver->rx_fifo_size;
		usbd_dev->fifo_mem_top += max_size / 4;
		usbd_dev->fifo_mem_top_ep0 = usbd_dev->fifo_mem_top;

		return;
	}

	if (dir) {
		REBASE(OTG_DIEPTXF(addr)) = ((max_size / 4) << 16) |
					     usbd_dev->fifo_mem_top;
		usbd_dev->fifo_mem_top += max_size / 4;

		REBASE(OTG_DIEPTSIZ(addr)) =
		    (max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DIEPCTL(addr)) |=
		    OTG_DIEPCTL0_EPENA | OTG_DIEPCTL0_SNAK | (type << 18)
		    | OTG_DIEPCTL0_USBAEP | OTG_DIEPCTLX_SD0PID
		    | (addr << 22) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_IN] =
			    (void *)callback;
		}
	}

	if (!dir) {
		usbd_dev->doeptsiz[addr] = OTG_DIEPSIZ0_PKTCNT |
				 (max_size & OTG_DIEPSIZ0_XFRSIZ_MASK);
		REBASE(OTG_DOEPTSIZ(addr)) = usbd_dev->doeptsiz[addr];
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_EPENA |
		    OTG_DOEPCTL0_USBAEP | OTG_DIEPCTL0_CNAK |
		    OTG_DOEPCTLX_SD0PID | (type << 18) | max_size;

		if (callback) {
			usbd_dev->user_callback_ctr[addr][USB_TRANSACTION_OUT] =
			    (void *)callback;
		}
	}
}

void dwc_endpoints_reset(usbd_device *usbd_dev)
{
	int i;
	/* The core resets the endpoints automatically on reset. */
	usbd_dev->fifo_mem_top = usbd_dev->fifo_mem_top_ep0;

	/* Disable any currently active endpoints */
	for (i = 1; i < 4; i++) {
		if (REBASE(OTG_DOEPCTL(i)) & OTG_DOEPCTL0_EPENA) {
			REBASE(OTG_DOEPCTL(i)) |= OTG_DOEPCTL0_EPDIS;
		}
		if (REBASE(OTG_DIEPCTL(i)) & OTG_DIEPCTL0_EPENA) {
			REBASE(OTG_DIEPCTL(i)) |= OTG_DIEPCTL0_EPDIS;
		}
	}

	/* Flush all tx/rx fifos */
	REBASE(OTG_GRSTCTL) = OTG_GRSTCTL_TXFFLSH | OTG_GRSTCTL_TXFNUM_ALL
			      | OTG_GRSTCTL_RXFFLSH;
}

void dwc_ep_stall_set(usbd_device *usbd_dev, uint8_t addr, uint8_t stall)
{
	if (addr == 0) {
		if (stall) {
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_STALL;
		} else {
			REBASE(OTG_DIEPCTL(addr)) &= ~OTG_DIEPCTL0_STALL;
		}
	}

	if (addr & 0x80) {
		addr &= 0x7F;

		if (stall) {
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_STALL;
		} else {
			REBASE(OTG_DIEPCTL(addr)) &= ~OTG_DIEPCTL0_STALL;
			REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTLX_SD0PID;
		}
	} else {
		if (stall) {
			REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_STALL;
		} else {
			REBASE(OTG_DOEPCTL(addr)) &= ~OTG_DOEPCTL0_STALL;
			REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTLX_SD0PID;
		}
	}
}

uint8_t dwc_ep_stall_get(usbd_device *usbd_dev, uint8_t addr)
{
	/* Return non-zero if STALL set. */
	if (addr & 0x80) {
		return (REBASE(OTG_DIEPCTL(addr & 0x7f)) &
				OTG_DIEPCTL0_STALL) ? 1 : 0;
	} else {
		return (REBASE(OTG_DOEPCTL(addr)) &
				OTG_DOEPCTL0_STALL) ? 1 : 0;
	}
}

void dwc_ep_nak_set(usbd_device *usbd_dev, uint8_t addr, uint8_t nak)
{
	/* It does not make sense to force NAK on IN endpoints. */
	if (addr & 0x80) {
		return;
	}

	usbd_dev->force_nak[addr] = nak;

	if (nak) {
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_SNAK;
	} else {
		REBASE(OTG_DOEPCTL(addr)) |= OTG_DOEPCTL0_CNAK;
	}
}

uint16_t dwc_ep_write_packet(usbd_device *usbd_dev, uint8_t addr,
			      const void *buf, uint16_t len)
{
	const uint32_t *buf32 = buf;
#if defined(__ARM_ARCH_6M__)
	const uint8_t *buf8 = buf;
	uint32_t word32;
#endif /* defined(__ARM_ARCH_6M__) */
	int i;

	addr &= 0x7F;

	/* Return if endpoint is already enabled. */
	if (REBASE(OTG_DIEPTSIZ(addr)) & OTG_DIEPSIZ0_PKTCNT) {
		return 0;
	}

	/* Enable endpoint for transmission. */
	REBASE(OTG_DIEPTSIZ(addr)) = OTG_DIEPSIZ0_PKTCNT | len;
	REBASE(OTG_DIEPCTL(addr)) |= OTG_DIEPCTL0_EPENA |
				     OTG_DIEPCTL0_CNAK;

	/* Copy buffer to endpoint FIFO, note - memcpy does not work.
	 * ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i > 0; i -= 4) {
		REBASE(OTG_FIFO(addr)) = *buf32++;
	}
#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i > 0; i -= 4) {
			REBASE(OTG_FIFO(addr)) = *buf32++;
		}
	} else {
		for (i = len; i > 0; i -= 4) {
			memcpy(&word32, buf8, 4);
			REBASE(OTG_FIFO(addr)) = word32;
			buf8 += 4;
		}
	}
#endif /* defined(__ARM_ARCH_6M__) */

	return len;
}

uint16_t dwc_ep_read_packet(usbd_device *usbd_dev, uint8_t addr,
				  void *buf, uint16_t len)
{
	int i;
	uint32_t *buf32 = buf;
#if defined(__ARM_ARCH_6M__)
	uint8_t *buf8 = buf;
	uint32_t word32;
#endif /* defined(__ARM_ARCH_6M__) */
	uint32_t extra;

	/* We do not need to know the endpoint address since there is only one
	 * receive FIFO for all endpoints.
	 */
	(void) addr;
	len = MIN(len, usbd_dev->rxbcnt);

	/* ARMv7M supports non-word-aligned accesses, ARMv6M does not. */
#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__)
	for (i = len; i >= 4; i -= 4) {
		*buf32++ = REBASE(OTG_FIFO(0));
		usbd_dev->rxbcnt -= 4;
	}
#endif /* defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) */

#if defined(__ARM_ARCH_6M__)
	/* Take care of word-aligned and non-word-aligned buffers */
	if (((uint32_t)buf8 & 0x3) == 0) {
		for (i = len; i >= 4; i -= 4) {
			*buf32++ = REBASE(OTG_FIFO(0));
			usbd_dev->rxbcnt -= 4;
		}
	} else {
		for (i = len; i >= 4; i -= 4) {
			word32 = REBASE(OTG_FIFO(0));
			memcpy(buf8, &word32, 4);
			usbd_dev->rxbcnt -= 4;
			buf8 += 4;
		}
		/* buf32 needs to be updated as it is used for extra */
		buf32 = (uint32_t *)buf8;
	}
#endif /* defined(__ARM_ARCH_6M__) */

	if (i) {
		extra = REBASE(OTG_FIFO(0));
		/* we read 4 bytes from the fifo, so update rxbcnt */
		if (usbd_dev->rxbcnt < 4) {
			/* Be careful not to underflow (rxbcnt is unsigned) */
			usbd_dev->rxbcnt = 0;
		} else {
			usbd_dev->rxbcnt -= 4;
		}
		memcpy(buf32, &extra, i);
	}

	return len;
}

static void dwc_flush_txfifo(usbd_device *usbd_dev, int ep)
{
	uint32_t fifo;
	/* set IN endpoint NAK */
	REBASE(OTG_DIEPCTL(ep)) |= OTG_DIEPCTL0_SNAK;
	/* wait for core to respond */
	while (!(REBASE(OTG_DIEPINT(ep)) & OTG_DIEPINTX_INEPNE)) {
		/* idle */
	}
	/* get fifo for this endpoint */
	fifo = (REBASE(OTG_DIEPCTL(ep)) & OTG_DIEPCTL0_TXFNUM_MASK) >> 22;
	/* wait for core to idle */
	while (!(REBASE(OTG_GRSTCTL) & OTG_GRSTCTL_AHBIDL)) {
		/* idle */
	}
	/* flush tx fifo */
	REBASE(OTG_GRSTCTL) = (fifo << 6) | OTG_GRSTCTL_TXFFLSH;
	/* reset packet counter */
	REBASE(OTG_DIEPTSIZ(ep)) = 0;
	while ((REBASE(OTG_GRSTCTL) & OTG_GRSTCTL_TXFFLSH)) {
		/* idle */
	}
}

volatile xxx;
volatile uint32_t mask;
volatile uint32_t xmask;
void dwc_poll(usbd_device *usbd_dev)
{
	/* Read interrupt status register. */
	uint32_t intsts = REBASE(OTG_GINTSTS);
	mask = REBASE(OTG_GINTMSK);
	xmask = intsts & mask;

log_trace_frame(TR_IRQ_ENTER, xmask);

	int i;

	if (intsts)
		xxx ++;

	if (intsts & OTG_GINTSTS_USBRST) {
		REBASE(OTG_DCTL) &=~ OTG_DCTL_RWUSIG;
		REBASE(OTG_DAINTMSK) = 0xffffffff;

		REBASE(OTG_DOEPMSK) |= OTG_DOEPMSK_STUPM
			| OTG_DOEPMSK_XFRCM
			| OTG_DOEPMSK_EPDM
			| (1 << 5) //OTG_DOEPMSK_OTEPSPRM
			| (1 << 13) //OTG_DOEPMSK_NAKM
			;
		REBASE(OTG_DIEPMSK) |= OTG_DIEPMSK_TOM
			| OTG_DIEPMSK_XFRCM
			| OTG_DIEPMSK_EPDM
			;

		REBASE(OTG_DCFG) &=~ OTG_DCFG_DAD;
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_USBRST;
	}

	if (intsts & OTG_GINTSTS_ENUMDNE) {
		/* Handle USB RESET condition. */
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_ENUMDNE;
		usbd_dev->fifo_mem_top = usbd_dev->driver->rx_fifo_size;
		_usbd_reset(usbd_dev);
log_trace_frame(TR_IRQ_EXIT, __LINE__);
		return;
	}

	/*
	 * There is no global interrupt flag for transmit complete.
	 * The XFRC bit must be checked in each OTG_DIEPINT(x).
	 */
	for (i = 0; i < 4; i++) { /* Iterate over endpoints. */
		if (REBASE(OTG_DIEPINT(i)) & OTG_DIEPINTX_XFRC) {
			/* Transfer complete. */
log_trace_frame(TR_IN_PACKET_SENT, i);
			if (usbd_dev->user_callback_ctr[i]
						       [USB_TRANSACTION_IN]) {
				usbd_dev->user_callback_ctr[i]
					[USB_TRANSACTION_IN](usbd_dev, i);
			}

			REBASE(OTG_DIEPINT(i)) = OTG_DIEPINTX_XFRC;
		}
	}

	/* Note: RX and TX handled differently in this device. */
	if (intsts & OTG_GINTSTS_RXFLVL) {
log_trace_frame(TR_PACKET_DATA_AVAILABLE, i);

		REBASE(OTG_GINTMSK) &=~ OTG_GINTMSK_RXFLVLM;

		/* Receive FIFO non-empty. */
		uint32_t rxstsp = REBASE(OTG_GRXSTSP);
		uint32_t pktsts = rxstsp & OTG_GRXSTSP_PKTSTS_MASK;
		uint8_t ep = rxstsp & OTG_GRXSTSP_EPNUM_MASK;
		// shopov: arm next out transaction - THIS SHOULD NOT BE DONE HERE!!!
		if (pktsts == OTG_GRXSTSP_PKTSTS_OUT_COMP /* 3 */
				|| pktsts == OTG_GRXSTSP_PKTSTS_SETUP_COMP /* 4 */)  {
log_trace_frame(TR_OUT_PACKET_COMPLETE, ep);

#if 1			
			REBASE(OTG_DOEPTSIZ(ep)) = usbd_dev->doeptsiz[ep];
			REBASE(OTG_DOEPCTL(ep)) |= OTG_DOEPCTL0_EPENA |

				(usbd_dev->force_nak[ep] ?
				 OTG_DOEPCTL0_SNAK : OTG_DOEPCTL0_CNAK);
			REBASE(OTG_GINTMSK) |= OTG_GINTMSK_RXFLVLM;
#endif
log_trace_frame(TR_IRQ_EXIT, __LINE__);
			return;
		}

		if ((pktsts != OTG_GRXSTSP_PKTSTS_OUT /* 2 */) &&
		    (pktsts != OTG_GRXSTSP_PKTSTS_SETUP /* 6 */)) {
		REBASE(OTG_GINTMSK) |= OTG_GINTMSK_RXFLVLM;
log_trace_frame(TR_IRQ_EXIT, __LINE__);
			return;
		}

		uint8_t type;
		if (pktsts == OTG_GRXSTSP_PKTSTS_SETUP /* 6 */) {
			type = USB_TRANSACTION_SETUP;
		} else {
			type = USB_TRANSACTION_OUT;
		}

		if (type == USB_TRANSACTION_SETUP
			&& (REBASE(OTG_DIEPTSIZ(ep)) & OTG_DIEPSIZ0_PKTCNT)) {
			/* SETUP received but there is still something stuck
			 * in the transmit fifo.  Flush it.
			 */
			while (1);
			dwc_flush_txfifo(usbd_dev, ep);
		}

		/* Save packet size for dwc_ep_read_packet(). */
		usbd_dev->rxbcnt = (rxstsp & OTG_GRXSTSP_BCNT_MASK) >> 4;

		if (usbd_dev->user_callback_ctr[ep][type]) {
			usbd_dev->user_callback_ctr[ep][type] (usbd_dev, ep);
		}

		if (usbd_dev->rxbcnt)
			while (1);
		/* Discard unread packet data. */
		for (i = 0; i < usbd_dev->rxbcnt; i += 4) {
			/* There is only one receive FIFO, so use OTG_FIFO(0) */
			(void)REBASE(OTG_FIFO(0));
		}

		usbd_dev->rxbcnt = 0;
REBASE(OTG_GINTMSK) |= OTG_GINTMSK_RXFLVLM;
	}

	if (intsts & OTG_GINTSTS_USBSUSP) {
		if (usbd_dev->user_callback_suspend) {
			usbd_dev->user_callback_suspend();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_USBSUSP;
	}

	if (intsts & OTG_GINTSTS_WKUPINT) {
		if (usbd_dev->user_callback_resume) {
			usbd_dev->user_callback_resume();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_WKUPINT;
	}

	if (intsts & OTG_GINTSTS_SOF) {
		if (usbd_dev->user_callback_sof) {
			usbd_dev->user_callback_sof();
		}
		REBASE(OTG_GINTSTS) = OTG_GINTSTS_SOF;
	}

	if (usbd_dev->user_callback_sof) {
		REBASE(OTG_GINTMSK) |= OTG_GINTMSK_SOFM;
	} else {
		REBASE(OTG_GINTMSK) &= ~OTG_GINTMSK_SOFM;
	}

#if 1
	// This is very evil... Black magic from the st sources; some of the bits are not even documented; this must go away
	for (i = 0U; i < 9; i ++)
		REBASE(OTG_DOEPINT(i)) |= 0xFB7FU;
#endif
log_trace_frame(TR_IRQ_EXIT, __LINE__);
}

void dwc_disconnect(usbd_device *usbd_dev, bool disconnected)
{
	if (disconnected) {
		REBASE(OTG_DCTL) |= OTG_DCTL_SDIS;
	} else {
		REBASE(OTG_DCTL) &= ~OTG_DCTL_SDIS;
	}
}
