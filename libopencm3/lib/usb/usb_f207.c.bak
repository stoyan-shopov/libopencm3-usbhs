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
#include <libopencm3/stm32/tools.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/dwc/otg_hs.h>
#include "usb_private.h"
#include "usb_dwc_common.h"

/* Receive FIFO size in 32-bit words. */
#define RX_FIFO_SIZE 512

static usbd_device *stm32f207_usbd_init(void);

static struct _usbd_device usbd_dev;

const struct _usbd_driver stm32f207_usb_driver = {
	.init = stm32f207_usbd_init,
	.set_address = dwc_set_address,
	.ep_setup = dwc_ep_setup,
	.ep_reset = dwc_endpoints_reset,
	.ep_stall_set = dwc_ep_stall_set,
	.ep_stall_get = dwc_ep_stall_get,
	.ep_nak_set = dwc_ep_nak_set,
	.ep_write_packet = dwc_ep_write_packet,
	.ep_read_packet = dwc_ep_read_packet,
	.poll = dwc_poll,
	.disconnect = dwc_disconnect,
	.base_address = USB_OTG_HS_BASE,
	.set_address_before_status = 1,
	.rx_fifo_size = RX_FIFO_SIZE,
};

/** Initialize the USB device controller hardware of the STM32. */
static usbd_device *stm32f207_usbd_init(void)
{
	//return &usbd_dev;
	int i = 0;
	xHAL_PCD_MspInit();

	OTG_HS_GAHBCFG &=~ OTG_GAHBCFG_GINT; 

	rcc_periph_clock_enable(RCC_OTGHS); //???
	OTG_HS_GINTSTS = OTG_GINTSTS_MMIS; //???

	OTG_HS_GCCFG &=~ OTG_GCCFG_PWRDWN;
	OTG_HS_GUSBCFG &=~ (OTG_GUSBCFG_TSDPS | OTG_GUSBCFG_ULPIFSLS | OTG_GUSBCFG_PHYSEL);
	OTG_HS_GUSBCFG &=~ (OTG_GUSBCFG_ULPIEVBUSD | OTG_GUSBCFG_ULPIEVBUSI);
	OTG_HS_GUSBCFG &=~ OTG_GUSBCFG_ULPISEL;
	OTG_HS_GCCFG |= OTG_GCCFG_PHYHSEN;

	/* ??? The st header files have this:
	 * #define USB_HS_PHYC_LDO_ENABLE                   USB_HS_PHYC_LDO_DISABLE
	 * ...go figure...
	 */
	OTG_HS_PHYC_LDO |= OTG_PHYC_LDO_DISABLE;
	while (!(OTG_HS_PHYC_LDO & OTG_PHYC_LDO_STATUS))
		;
	/* This setting is for a HSE clock of 25 MHz. */
	OTG_HS_PHYC_PLL1 = 5 << 1;
	OTG_HS_PHYC_TUNE |= 0x00000F13U;
	OTG_HS_PHYC_PLL1 |= OTG_PHYC_PLL1_ENABLE;
	/* 2ms Delay required to get internal phy clock stable */
	HAL_Delay(2U);

	while (!(OTG_HS_GRSTCTL & OTG_GRSTCTL_AHBIDL))
		;

	OTG_HS_GRSTCTL |= OTG_GRSTCTL_CSRST;
	while (OTG_HS_GRSTCTL & OTG_GRSTCTL_CSRST);

	OTG_HS_GUSBCFG &=~ (OTG_GUSBCFG_FHMOD | OTG_GUSBCFG_FDMOD);

	////OTG_HS_GCCFG |= OTG_GCCFG_PWRDWN;//???
	OTG_HS_GUSBCFG |= OTG_GUSBCFG_FDMOD;
	HAL_Delay(50U);

	for (i = 0U; i < 15U; i++)
		OTG_HS_DIEPTXF(i) = 0;

	OTG_HS_DCTL |= OTG_DCTL_SDIS;
	OTG_HS_GCCFG &=~ OTG_GCCFG_VBDEN;

	OTG_HS_GOTGCTL |= OTG_GOTGCTL_BVALOEN;
	OTG_HS_GOTGCTL |= OTG_GOTGCTL_BVALOVAL;

	OTG_HS_PCGCCTL = 0;

	/* remove these after debugging is done */
	OTG_HS_DCFG |= 0;
	OTG_HS_DCFG |= 0;

	OTG_HS_GRSTCTL = OTG_GRSTCTL_TXFFLSH | (0x10 << 6);
	while (OTG_HS_GRSTCTL & OTG_GRSTCTL_TXFFLSH);
	OTG_HS_GRSTCTL = OTG_GRSTCTL_RXFFLSH;
	while (OTG_HS_GRSTCTL & OTG_GRSTCTL_RXFFLSH);

	/* Clear all pending Device Interrupts */
	OTG_HS_DIEPMSK = 0;
	OTG_HS_DOEPMSK = 0;
	OTG_HS_DAINTMSK = 0;

	for (i = 0U; i < 9; i++)
	{
		OTG_HS_DIEPCTL(i) = 0;
		OTG_HS_DIEPTSIZ(i) = 0;
		/* ??? what is this value ??? */
		OTG_HS_DIEPINT(i) = 0xFB7FU;
	}
	for (i = 0U; i < 9; i++)
	{
		OTG_HS_DOEPCTL(i) = 0;
		OTG_HS_DOEPTSIZ(i) = 0;
		OTG_HS_DOEPINT(i) = 0;
	}

	OTG_HS_GINTMSK = 0;
	/* Clear any pending interrupts */
	OTG_HS_GINTSTS = 0xBFFFFFFFU;
#if 0
	OTG_HS_GINTMSK |= OTG_GINTMSK_RXFLVLM
		| OTG_GINTMSK_USBSUSPM
		| OTG_GINTMSK_USBRST
		| OTG_GINTMSK_ENUMDNEM
		| OTG_GINTMSK_IEPINT
		| OTG_GINTMSK_OEPINT
		| OTG_GINTMSK_IISOOXFRM
		| OTG_GINTMSK_WUIM
		;
#else 

	OTG_HS_GINTMSK = OTG_GINTMSK_ENUMDNEM |
			 OTG_GINTMSK_RXFLVLM |
			 OTG_GINTMSK_IEPINT |
			 OTG_GINTMSK_OEPINT |
			 OTG_GINTMSK_USBSUSPM |
			 OTG_GINTMSK_USBRST |
			 OTG_GINTMSK_WUIM;
#endif

	/* this is redundant - already set above */
	OTG_HS_DCTL |= OTG_DCTL_SDIS;
	HAL_Delay(3U);

	/* maybe redundant - this is the reset value */
	//OTG_HS_GRXFSIZ = 0x400;
	/* try later with a value of 0x400 */
	OTG_HS_GRXFSIZ = 0x200;

	OTG_HS_GNPTXFSIZ = (0x80 << 16) | OTG_HS_GRXFSIZ;
	OTG_HS_DIEPTXF(1) = (0x174 << 16) | (OTG_HS_GRXFSIZ + 0x80);


	OTG_HS_DCTL &=~ OTG_DCTL_SDIS;
	HAL_Delay(3U);
	OTG_HS_GAHBCFG |= OTG_GAHBCFG_GINT;

	return &usbd_dev;

#if ORIGINAL_INIT
	rcc_periph_clock_enable(RCC_OTGHS);
	OTG_HS_GINTSTS = OTG_GINTSTS_MMIS;

	OTG_HS_GUSBCFG |= OTG_GUSBCFG_PHYSEL;
	/* Enable VBUS sensing in device mode and power down the PHY. */
	OTG_HS_GCCFG |= OTG_GCCFG_VBUSBSEN | OTG_GCCFG_PWRDWN;

	/* Wait for AHB idle. */
	while (!(OTG_HS_GRSTCTL & OTG_GRSTCTL_AHBIDL));
	/* Do core soft reset. */
	OTG_HS_GRSTCTL |= OTG_GRSTCTL_CSRST;
	while (OTG_HS_GRSTCTL & OTG_GRSTCTL_CSRST);

	/* Force peripheral only mode. */
	OTG_HS_GUSBCFG |= OTG_GUSBCFG_FDMOD | OTG_GUSBCFG_TRDT_MASK;

	/* Full speed device. */
	OTG_HS_DCFG |= OTG_DCFG_DSPD;

	/* Restart the PHY clock. */
	OTG_HS_PCGCCTL = 0;

	OTG_HS_GRXFSIZ = stm32f207_usb_driver.rx_fifo_size;
	usbd_dev.fifo_mem_top = stm32f207_usb_driver.rx_fifo_size;

	/* Unmask interrupts for TX and RX. */
	OTG_HS_GAHBCFG |= OTG_GAHBCFG_GINT;
	OTG_HS_GINTMSK = OTG_GINTMSK_ENUMDNEM |
			 OTG_GINTMSK_RXFLVLM |
			 OTG_GINTMSK_IEPINT |
			 OTG_GINTMSK_USBSUSPM |
			 OTG_GINTMSK_WUIM;
	OTG_HS_DAINTMSK = 0xF;
	OTG_HS_DIEPMSK = OTG_DIEPMSK_XFRCM;

	return &usbd_dev;
#endif
}


#if CUBE_INITIALIZATION
  hpcd_USB_OTG_HS.pData = pdev;
  pdev->pData = &hpcd_USB_OTG_HS;

  hpcd_USB_OTG_HS.Instance = USB_OTG_HS;
  hpcd_USB_OTG_HS.Init.dev_endpoints = 9;
  hpcd_USB_OTG_HS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.phy_itface = USB_OTG_HS_EMBEDDED_PHY;
  hpcd_USB_OTG_HS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_HS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_HS.Init.use_external_vbus = DISABLE;

  xHAL_PCD_MspInit();

  USBx->GAHBCFG &= ~USB_OTG_GAHBCFG_GINT;

  // if (cfg.phy_itface == USB_OTG_HS_EMBEDDED_PHY)
  {
    USBx->GCCFG &= ~(USB_OTG_GCCFG_PWRDWN);

    /* Init The UTMI Interface */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_TSDPS | USB_OTG_GUSBCFG_ULPIFSLS | USB_OTG_GUSBCFG_PHYSEL);

    /* Select vbus source */
    USBx->GUSBCFG &= ~(USB_OTG_GUSBCFG_ULPIEVBUSD | USB_OTG_GUSBCFG_ULPIEVBUSI);

    /* Select UTMI Interace */
    USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_ULPI_UTMI_SEL;
    USBx->GCCFG |= USB_OTG_GCCFG_PHYHSEN;
  }
++++++++++++++++++++++++++++++++++++++++++++++++

	// Enable ldo
  USB_HS_PHYC->USB_HS_PHYC_LDO |= USB_HS_PHYC_LDO_ENABLE;

  /* wait for LDO Ready */
  while ((USB_HS_PHYC->USB_HS_PHYC_LDO & USB_HS_PHYC_LDO_STATUS) == 0U)
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }


  //else if (HSE_VALUE == 25000000U) /* HSE = 25MHz */
  {
    USB_HS_PHYC->USB_HS_PHYC_PLL = (0x5U << 1);
  }

++++++++++++++++++++++++++++++++++++++++++++++++

  /* Control the tuning interface of the High Speed PHY */
  USB_HS_PHYC->USB_HS_PHYC_TUNE |= USB_HS_PHYC_TUNE_VALUE;

  /* Enable PLL internal PHY */
  USB_HS_PHYC->USB_HS_PHYC_PLL |= USB_HS_PHYC_PLL_PLLEN;

  /* 2ms Delay required to get internal phy clock stable */
  HAL_Delay(2U);

++++++++++++++++++++++++++++++++++++++++++++++++


  /* Wait for AHB master IDLE state. */
  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_AHBIDL) == 0U);

++++++++++++++++++++++++++++++++++++++++++++++++

  /* Core Soft Reset */
  count = 0U;
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_CSRST) == USB_OTG_GRSTCTL_CSRST);

++++++++++++++++++++++++++++++++++++++++++++++++

    /* Activate the USB Transceiver */
    USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;

    USBx->GUSBCFG |= USB_OTG_GUSBCFG_FDMOD;
    HAL_Delay(50U)

++++++++++++++++++++++++++++++++++++++++++++++++
	  /* Init endpoints structures */
  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    /* Init ep structure */
    hpcd->IN_ep[i].is_in = 1U;
    hpcd->IN_ep[i].num = i;
    hpcd->IN_ep[i].tx_fifo_num = i;
    /* Control until ep is activated */
    hpcd->IN_ep[i].type = EP_TYPE_CTRL;
    hpcd->IN_ep[i].maxpacket = 0U;
    hpcd->IN_ep[i].xfer_buff = 0U;
    hpcd->IN_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < hpcd->Init.dev_endpoints; i++)
  {
    hpcd->OUT_ep[i].is_in = 0U;
    hpcd->OUT_ep[i].num = i;
    /* Control until ep is activated */
    hpcd->OUT_ep[i].type = EP_TYPE_CTRL;
    hpcd->OUT_ep[i].maxpacket = 0U;
    hpcd->OUT_ep[i].xfer_buff = 0U;
    hpcd->OUT_ep[i].xfer_len = 0U;
  }

  for (i = 0U; i < 15U; i++)
  {
    USBx->DIEPTXF[i] = 0U;
  }

++++++++++++++++++++++++++++++++++++++++++++++++

    USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;

    /* Deactivate VBUS Sensing B */
    USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

    /* B-peripheral session valid override enable */
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
    USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;


  /* Restart the Phy Clock */
  USBx_PCGCCTL = 0U;

  /* Device mode configuration */
  USBx_DEVICE->DCFG |= DCFG_FRAME_INTERVAL_80;

   USBx_DEVICE->DCFG |= USB_OTG_SPEED_HIGH;

++++++++++++++++++++++++++++++++++++++++++++++++

  uint32_t count = 0U;

  USBx->GRSTCTL = (USB_OTG_GRSTCTL_TXFFLSH | (0x10 << 6));

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_TXFFLSH) == USB_OTG_GRSTCTL_TXFFLSH);


    USBx->GRSTCTL = USB_OTG_GRSTCTL_RXFFLSH;

  do
  {
    if (++count > 200000U)
    {
      return HAL_TIMEOUT;
    }
  }
  while ((USBx->GRSTCTL & USB_OTG_GRSTCTL_RXFFLSH) == USB_OTG_GRSTCTL_RXFFLSH);

++++++++++++++++++++++++++++++++++++++++++++++++

  /* Clear all pending Device Interrupts */
  USBx_DEVICE->DIEPMSK = 0U;
  USBx_DEVICE->DOEPMSK = 0U;
  USBx_DEVICE->DAINTMSK = 0U;
++++++++++++++++++++++++++++++++++++++++++++++++

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_INEP(i)->DIEPCTL & USB_OTG_DIEPCTL_EPENA) == USB_OTG_DIEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_SNAK;
      }
      else
      {
        USBx_INEP(i)->DIEPCTL = USB_OTG_DIEPCTL_EPDIS | USB_OTG_DIEPCTL_SNAK;
      }
    }
    else
    {
      USBx_INEP(i)->DIEPCTL = 0U;
    }

    USBx_INEP(i)->DIEPTSIZ = 0U;
    USBx_INEP(i)->DIEPINT  = 0xFB7FU;
  }

  for (i = 0U; i < cfg.dev_endpoints; i++)
  {
    if ((USBx_OUTEP(i)->DOEPCTL & USB_OTG_DOEPCTL_EPENA) == USB_OTG_DOEPCTL_EPENA)
    {
      if (i == 0U)
      {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_SNAK;
      }
      else
      {
        USBx_OUTEP(i)->DOEPCTL = USB_OTG_DOEPCTL_EPDIS | USB_OTG_DOEPCTL_SNAK;
      }
    }
    else
    {
      USBx_OUTEP(i)->DOEPCTL = 0U;
    }

    USBx_OUTEP(i)->DOEPTSIZ = 0U;
    USBx_OUTEP(i)->DOEPINT  = 0xFB7FU;
  }
++++++++++++++++++++++++++++++++++++++++++++++++

  USBx_DEVICE->DIEPMSK &= ~(USB_OTG_DIEPMSK_TXFURM);

  /* Disable all interrupts. */
  USBx->GINTMSK = 0U;

  /* Clear any pending interrupts */
  USBx->GINTSTS = 0xBFFFFFFFU;

  /* Enable the common interrupts */
  if (cfg.dma_enable == 0U)
  {
    USBx->GINTMSK |= USB_OTG_GINTMSK_RXFLVLM;
  }

  /* Enable interrupts matching to the Device mode ONLY */
  USBx->GINTMSK |= USB_OTG_GINTMSK_USBSUSPM | USB_OTG_GINTMSK_USBRST |
                   USB_OTG_GINTMSK_ENUMDNEM | USB_OTG_GINTMSK_IEPINT |
                   USB_OTG_GINTMSK_OEPINT   | USB_OTG_GINTMSK_IISOIXFRM |
                   USB_OTG_GINTMSK_PXFRM_IISOOXFRM | USB_OTG_GINTMSK_WUIM;


  USBx_DEVICE->DCTL |= USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);

++++++++++++++++++++++++++++++++++++++++++++++++

//HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_HS, 0x200);
  hpcd->Instance->GRXFSIZ = size;
++++++++++++++++++++++++++++++++++++++++++++++++
//HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 0, 0x80);
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size)
{
  uint8_t i;
  uint32_t Tx_Offset;

  /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows:
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

  Tx_Offset = hpcd->Instance->GRXFSIZ;

  if (fifo == 0U)
  {
    hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
  }
  else
  {
    Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ) >> 16;
    for (i = 0U; i < (fifo - 1U); i++)
    {
      Tx_Offset += (hpcd->Instance->DIEPTXF[i] >> 16);
    }

    /* Multiply Tx_Size by 2 to get higher performance */
    hpcd->Instance->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
  }

  return HAL_OK;
}

//HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_HS, 1, 0x174);
HAL_StatusTypeDef HAL_PCDEx_SetTxFiFo(PCD_HandleTypeDef *hpcd, uint8_t fifo, uint16_t size)
{
  uint8_t i;
  uint32_t Tx_Offset;

  /*  TXn min size = 16 words. (n  : Transmit FIFO index)
      When a TxFIFO is not used, the Configuration should be as follows:
          case 1 :  n > m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txm can use the space allocated for Txn.
         case2  :  n < m    and Txn is not used    (n,m  : Transmit FIFO indexes)
         --> Txn should be configured with the minimum space of 16 words
     The FIFO is used optimally when used TxFIFOs are allocated in the top
         of the FIFO.Ex: use EP1 and EP2 as IN instead of EP1 and EP3 as IN ones.
     When DMA is used 3n * FIFO locations should be reserved for internal DMA registers */

  Tx_Offset = hpcd->Instance->GRXFSIZ;

  if (fifo == 0U)
  {
    hpcd->Instance->DIEPTXF0_HNPTXFSIZ = ((uint32_t)size << 16) | Tx_Offset;
  }
  else
  {
    Tx_Offset += (hpcd->Instance->DIEPTXF0_HNPTXFSIZ) >> 16;
    for (i = 0U; i < (fifo - 1U); i++)
    {
      Tx_Offset += (hpcd->Instance->DIEPTXF[i] >> 16);
    }

    /* Multiply Tx_Size by 2 to get higher performance */
    hpcd->Instance->DIEPTXF[fifo - 1U] = ((uint32_t)size << 16) | Tx_Offset;
  }

  return HAL_OK;
}
++++++++++++++++++++++++++++++++++++++++++++++++


  USBx_DEVICE->DCTL &= ~USB_OTG_DCTL_SDIS;
  HAL_Delay(3U);

  USBx->GAHBCFG |= USB_OTG_GAHBCFG_GINT;




#endif
