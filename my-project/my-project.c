#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>

void xSystemClock_Config(void)
{
	rcc_clock_setup_hse(rcc_3v3 + RCC_CLOCK_3V3_216MHZ, 25000000);
}

void xHAL_PCD_MspInit(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO14 | GPIO15);
	gpio_set_output_options(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO14 | GPIO15);
	gpio_set_af(GPIOB, GPIO_AF12, GPIO14 | GPIO15);

	rcc_periph_clock_enable(RCC_OTGPHYC);
	rcc_periph_clock_enable(RCC_OTGHS);
	rcc_periph_clock_enable(RCC_OTGHSULPI);

	// TODO - check the preemption and subpriority, they are unified here
	nvic_set_priority(NVIC_OTG_HS_IRQ, 0);
	//nvic_set_priority(NVIC_OTG_HS_IRQ, 64);
}

void xEnableUsbInterrupt()
{
	nvic_enable_irq(NVIC_OTG_HS_IRQ);
}


void sys_tick_handler(void)
{
  HAL_IncTick();
}

extern unsigned hpcd_USB_OTG_HS;
extern HAL_PCD_IRQHandler(unsigned);


bool use_libopencm3_usb_code = true;

extern volatile int CUBE;
void otg_hs_isr(void)
{
	//use_libopencm3_usb_code ? xusb_poll() : HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
#if 0
	if (CUBE)
		HAL_PCD_IRQHandler(&hpcd_USB_OTG_HS);
	else
#endif
		xusb_poll();
}

void xHAL_MspInit(void)
{
	rcc_periph_clock_enable(RCC_APB1ENR_PWREN);
	rcc_periph_clock_enable(RCC_APB2ENR_SYSCFGEN);
}

void xMX_GPIO_Init(void)
{
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOH);
}

