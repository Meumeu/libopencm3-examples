/*
 * This file is part of the libopencm3 project.
 *
 * Copyright (C) 2019 Guillaume Meunier <guillaume.meunier@centraliens.net>
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

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/l4plus/nvic.h>
#include <libopencm3/cm3/cortex.h>

/* --- Convenience macros -------------------------------------------------- */

/* Low-power timer register base addresses (for convenience) */
/****************************************************************************/
/** @defgroup lptim_reg_base Low-power timer register base addresses
@{*/
#define LPTIM1				LPTIM1_BASE
#define LPTIM2				LPTIM2_BASE
/**@}*/

/* --- Low-power timer registers ------------------------------------------- */

/* Interrupt and status register (LPTIMx_ISR) */
#define LPTIM_ISR(lptim_base)		MMIO32((lptim_base) + 0x00)
#define LPTIM1_ISR			LPTIM_ISR(LPTIM1)
#define LPTIM2_ISR			LPTIM_ISR(LPTIM2)

/* interrupt clear register (LPTIMx_ICR) */
#define LPTIM_ICR(lptim_base)		MMIO32((lptim_base) + 0x04)
#define LPTIM1_ICR			LPTIM_ICR(LPTIM1)
#define LPTIM2_ICR			LPTIM_ICR(LPTIM2)

/* interrupt enable register  (LPTIMx_IER) */
#define LPTIM_IER(lptim_base)		MMIO32((lptim_base) + 0x08)
#define LPTIM1_IER			LPTIM_IER(LPTIM1)
#define LPTIM2_IER			LPTIM_IER(LPTIM2)

/* Configuration register (LPTIMx_CFGR) */
#define LPTIM_CFGR(lptim_base)		MMIO32((lptim_base) + 0x0c)
#define LPTIM1_CFGR			LPTIM_CFGR(LPTIM1)
#define LPTIM2_CFGR			LPTIM_CFGR(LPTIM2)

/* Control register (LPTIMx_CR) */
#define LPTIM_CR(lptim_base)		MMIO32((lptim_base) + 0x10)
#define LPTIM1_CR			LPTIM_CR(LPTIM1)
#define LPTIM2_CR			LPTIM_CR(LPTIM2)

/* Compare register (LPTIMx_CMP) */
#define LPTIM_CMP(lptim_base)		MMIO32((lptim_base) + 0x14)
#define LPTIM1_CMP			LPTIM_CMP(LPTIM1)
#define LPTIM2_CMP			LPTIM_CMP(LPTIM2)

/* Autoreload register (LPTIMx_ARR) */
#define LPTIM_ARR(lptim_base)		MMIO32((lptim_base) + 0x18)
#define LPTIM1_ARR			LPTIM_ARR(LPTIM1)
#define LPTIM2_ARR			LPTIM_ARR(LPTIM2)

/* Counter register (LPTIMx_CNT) */
#define LPTIM_CNT(lptim_base)		MMIO32((lptim_base) + 0x1c)
#define LPTIM1_CNT			LPTIM_CNT(LPTIM1)
#define LPTIM2_CNT			LPTIM_CNT(LPTIM2)

/* Option register (LPTIMx_OR) */
#define LPTIM_OR(lptim_base)		MMIO32((lptim_base) + 0x20)
#define LPTIM1_OR			LPTIM_OR(LPTIM1)
#define LPTIM2_OR			LPTIM_OR(LPTIM2)


static void clock_setup(void)
{
	rcc_periph_clock_enable(RCC_PWR);
    
	// Enable LSE
	pwr_disable_backup_domain_write_protect(); // LSEON is read-only by default
	rcc_osc_on(RCC_LSE);
	rcc_wait_for_osc_ready(RCC_LSE);
	pwr_enable_backup_domain_write_protect();
	
	// Enable HSE
	rcc_osc_on(RCC_HSE);
	rcc_wait_for_osc_ready(RCC_HSE);
	
	// Enable HSI
	rcc_osc_on(RCC_HSI16);
	rcc_wait_for_osc_ready(RCC_HSI16);
}

static void clock_set_msi(void)
{
	rcc_set_sysclk_source(RCC_CFGR_SW_MSI);
	rcc_wait_for_sysclk_status(RCC_MSI);
}

static void clock_set_hsi16(void)
{
	rcc_set_sysclk_source(RCC_CFGR_SW_HSI16);
	rcc_wait_for_sysclk_status(RCC_HSI16);
}

static void clock_set_speed(int freq_mhz)
{
	if (freq_mhz < 1 || freq_mhz > 120)
	    return;

	if ((PWR_CR5 & PWR_CR5_R1MODE) && (freq_mhz > 80))
	{
	    // Switch power to range 1 boost (§5.1.8, p 172)
	    rcc_set_hpre(RCC_CFGR_HPRE_DIV2);
	    pwr_set_vos_scale(PWR_SCALE1BOOST);
	}
	else if (!(PWR_CR5 & PWR_CR5_R1MODE) && (freq_mhz <= 80))
	{
	    // Switch power to range 1 normal (§5.1.8, p 172)
	    pwr_set_vos_scale(PWR_SCALE1);
	}
	
	// Switch to HSI16 before disabling the PLL
	clock_set_hsi16();
    
	// Disable PLL
	rcc_osc_off(RCC_PLL);
	while(rcc_is_osc_ready(RCC_PLL));

	// Input frequency: f=16MHz
	// f / PLLM1 must be >= 2.66 and <= 8
	// f / PLLM1 * N >= 96 and <= 344
	// f / PLLM1 * N / R >= 8 and <= 120

	// CPU frequency must also be >= 14.2 MHz for USB

	// So: 2 <= PLLM1 <= 6
	//     6 <= N / PLLM1 <= 21.5
	//     0.5 <= N / PLLM1 / R <= 7.5

	if (freq_mhz <= 20)
		flash_set_ws(0);
	else if (freq_mhz <= 40)
		flash_set_ws(1);
	else if (freq_mhz <= 60)
		flash_set_ws(2);
	else if (freq_mhz <= 80)
		flash_set_ws(3);
	else if (freq_mhz <= 100)
		flash_set_ws(4);
	else // freq_mhz <= 120
		flash_set_ws(5);

	if (freq_mhz <= 80)
	{
		// PLLM1 = 4, R = 4, N = freq_mhz
		rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSE, 4 /* PLLM */, freq_mhz /* PLLN */, RCC_PLLCFGR_PLLP_DIV2 /* PLLP */, RCC_PLLCFGR_PLLQ_DIV2 /* PLLQ */, RCC_PLLCFGR_PLLR_DIV4 /* PLLR */);
	}
	else
	{
		// PLLM1 = 4, R = 2, N = freq_mhz / 2
		rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSE, 4 /* PLLM */, freq_mhz / 2 /* PLLN */, RCC_PLLCFGR_PLLP_DIV2 /* PLLP */, RCC_PLLCFGR_PLLQ_DIV2 /* PLLQ */, RCC_PLLCFGR_PLLR_DIV2 /* PLLR */);
	}

	// Enable PLL
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);

	// Switch to PLL
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	while(rcc_system_clock_source() != RCC_CFGR_SW_PLL);

	// End of switch to range 1 boost, does nothing in range 1 normal
	for(int i = 0; i < 100; ++i)
		__asm__ volatile("nop");

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
}


static void led_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOH);
	gpio_mode_setup(GPIO_PORT_H_BASE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO4);
	gpio_set(GPIO_PORT_H_BASE, GPIO4);
}

static void lptim1_setup(void)
{
	// Use LSE as clock source for LPTIM1
	RCC_CCIPR |= RCC_CCIPR_LPTIMxSEL_LSE << RCC_CCIPR_LPTIM1SEL_SHIFT;

	rcc_periph_clock_enable(RCC_LPTIM1);
	
	LPTIM1_IER = 1 << 0; //  Compare match Interrupt Enable
	LPTIM1_CFGR = 0;
	
	LPTIM1_CR |= 1 << 0; // enable

	LPTIM1_CMP = 0x7fff; // set reload value

	LPTIM1_CR |= 1 << 2; // start continuous
	
	nvic_enable_irq(NVIC_LPTIM1_IRQ);
}

volatile int time;
volatile int measured_frequency;

void lptim1_isr(void)
{
	static int last_timer2;
	
	LPTIM1_ICR = 1 << 0;

	time++;
	
	int timer2 = timer_get_counter(TIM2);
	measured_frequency = timer2 - last_timer2;
	last_timer2 = timer2;
}

static int blink(void)
{
	time = 0;

	while(time < 5)
	{
		for(int i = 0; i < 1000000; ++i)
			__asm__ volatile("nop");
		gpio_clear(GPIO_PORT_H_BASE, GPIO4);
		
		for(int i = 0; i < 1000000; ++i)
			__asm__ volatile("nop");
		gpio_set(GPIO_PORT_H_BASE, GPIO4);
	}
	
	return measured_frequency;
}

int frequencies[12];
int main(void)
{
	clock_setup();
	led_setup();
	lptim1_setup();
	
	rcc_periph_clock_enable(RCC_TIM2);
	timer_direction_up(TIM2);
	timer_set_period(TIM2, 0xffffffff);
	timer_continuous_mode(TIM2);
	timer_enable_counter(TIM2);

	cm_enable_interrupts();
	
	while(1)
	{
		clock_set_msi();
		frequencies[0] = blink();
		
		for(int f = 20; f <= 120; f += 10)
		{
			clock_set_speed(f);
			frequencies[(f - 10) / 10] = blink();
		}
	}
}
