/*
 * Copyright (c) 2024 Semios Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>
#include <soc.h>

#include <am_mcu_apollo.h>

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(power, CONFIG_PM_LOG_LEVEL);

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(substate_id);

	__disable_irq();
	irq_unlock(0);

	switch (state) {
	case PM_STATE_RUNTIME_IDLE:
		/* Sys sleep mode 1 */
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
		k_cpu_idle();
		break;
	case PM_STATE_SUSPEND_TO_IDLE:
		/* Sys deep sleep mode 1 */
		am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_CACHE);
		am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_FLASH_2M);
		am_hal_pwrctrl_memory_deepsleep_powerdown(AM_HAL_PWRCTRL_MEM_SRAM_MAX);
		am_hal_pwrctrl_memory_deepsleep_retain(AM_HAL_PWRCTRL_MEM_SRAM_768K);
		am_hal_sysctrl_control(AM_HAL_SYSCTRL_CONTROL_DEEPSLEEP_MINPWR_EN, 0);
		am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_DEEP);
		k_cpu_idle();
		break;
	default:
		break;
	}
}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{
	ARG_UNUSED(state);
	ARG_UNUSED(substate_id);

	__enable_irq();
}
