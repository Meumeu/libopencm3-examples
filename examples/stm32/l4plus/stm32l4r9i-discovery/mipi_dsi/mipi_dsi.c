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
#include <libopencm3/stm32/pwr.h>
#include <libopencm3/stm32/l4plus/nvic.h>
#include <libopencm3/cm3/cortex.h>
#include <libopencm3/stm32/flash.h>

static void clock_setup_120mhz(void)
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
	
	// Switch power to range 1 boost (§5.1.8, p 172)
	rcc_set_hpre(RCC_CFGR_HPRE_DIV2);
	pwr_set_vos_scale(PWR_SCALE1BOOST);
	
	flash_set_ws(5);
	flash_icache_enable();
	flash_dcache_enable();
	flash_prefetch_enable();
	
	// CPU frequency = HSE(16MHz) * PLLN / (PLLM*PLLR) = 120MHz
	rcc_set_main_pll(RCC_PLLCFGR_PLLSRC_HSE, 4 /* PLLM */, 60 /* PLLN */, RCC_PLLCFGR_PLLP_DIV2 /* PLLP */, RCC_PLLCFGR_PLLQ_DIV2 /* PLLQ */, RCC_PLLCFGR_PLLR_DIV2 /* PLLR */);
	
	// Enable PLL
	rcc_osc_on(RCC_PLL);
	rcc_wait_for_osc_ready(RCC_PLL);
	
	// Switch to PLL
	rcc_set_sysclk_source(RCC_CFGR_SW_PLL);
	while(rcc_system_clock_source() != RCC_CFGR_SW_PLL);
	
	// End of switch to range 1 boost
	for(int i = 0; i < 100; ++i)
		__asm__ volatile("nop");

	rcc_set_hpre(RCC_CFGR_HPRE_NODIV);
}

static void display_setup(void)
{
	// See stm32l4r9i_discovery_lcd.c, LCD_PowerOn()
	
	
	// See stm32l4r9i_discovery_lcd.c, BSP_LCD_MspInit()
	rcc_periph_clock_enable(RCC_GFXMMU);
	rcc_periph_reset_pulse(RST_GFXMMU);
	
	rcc_periph_clock_enable(RCC_LTDC);
	rcc_periph_reset_pulse(RST_LTDC);
	
	rcc_periph_clock_enable(RCC_DMA2D);
	rcc_periph_reset_pulse(RST_DMA2D);
	
	rcc_periph_clock_enable(RCC_DSI);
	rcc_periph_reset_pulse(RST_DSI);
	
	// See stm32l4xx_hal_rcc_ex.c, HAL_RCCEx_PeriphCLKConfig
// 	rcc_osc_off(RCC_PLLSAI2);
	RCC_CR &= ~RCC_CR_PLLSAI2ON;
// 	rcc_is_osc_ready(RCC_PLLSAI2);
	while(RCC_CR & RCC_CR_PLLSAI2RDY);
	
	// LTDC clock = PLLSAI2R / 4
	RCC_CCIPR2 = (RCC_CCIPR2 & ~(RCC_CCIPR2_PLLSAI2DIVR_MASK << RCC_CCIPR2_PLLSAI2DIVR_SHIFT)) | (RCC_CCIPR2_PLLSAI2DIVR_DIV4 << RCC_CCIPR2_PLLSAI2DIVR_SHIFT);
	
	// Configure PLLSAI2R to output 60 MHz, to have LTDC running at 15 MHz
	// ST uses the HSI, this uses the HSE with:
	// PLLM3 = /4
	// N = *60
	// R = /4
	RCC_PLLSAI2_CFGR = (RCC_PLLCFGR_PLLM(4) << RCC_PLLCFGR_PLLM_SHIFT) | (RCC_PLLCFGR_PLLR_DIV4 << RCC_PLLCFGR_PLLR_SHIFT) | (60 << RCC_PLLCFGR_PLLN_SHIFT) | RCC_PLLCFGR_PLLREN;

	// TBC, ST uses priority group, preempt priority, subpriority
	nvic_set_priority(NVIC_LCD_TFT_IRQ, 3);
	nvic_set_priority(NVIC_LCD_TFT_ER_IRQ, 3);
	
	// LTDC configuration, stm32l4r9i_discovery_lcd.c, BSP_LCD_Init()
	
	
}

int main(void)
{
	clock_setup_120mhz();
	display_setup();
}
