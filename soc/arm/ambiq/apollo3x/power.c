/* HOLLY NOTE: we should be writing here... */
#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/pm/state.h>
#include <zephyr/pm/pm.h>
#include <hal/am_hal_sysctrl.h>
#include <hal/am_hal_pwrctrl.h>

#include <zephyr/logging/log.h>

#define LOG_LEVEL CONFIG_LOG_DEFAULT_LEVEL
LOG_MODULE_REGISTER(power);

static int ambiq_power_init(void);

static int ambiq_power_init(void)
{
    am_hal_pwrctrl_low_power_init();
    return 0;
}

void pm_state_set(enum pm_state state, uint8_t substate_id)
{
    /*switch (state) {
    case PM_STATE_SUSPEND_TO_IDLE:
        am_hal_sysctrl_sleep(AM_HAL_SYSCTRL_SLEEP_NORMAL);
        break;
    default:
        LOG_DBG("Unsupported power substate-id %u",
							substate_id);
        break;
    }*/

}

void pm_state_exit_post_ops(enum pm_state state, uint8_t substate_id)
{

}

SYS_INIT(ambiq_power_init, PRE_KERNEL_1, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);
