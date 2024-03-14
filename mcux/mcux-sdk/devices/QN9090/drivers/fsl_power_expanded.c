//
// Created by ijsbrand on 13-3-24.
//

#include "fsl_power.h"

#include "fsl_iocon.h"

#include "rom_mpu.h"
#include "rom_api.h"
#include <fsl_wtimer.h>

/*beginning additions*/

/*Code block for saving and restoring CPU context. Unused in main code, but might come in useful to implement deep sleep into zephyr at one point*/

#define PWR_JUMP_BUF_SIZE         10

/*Reserve a part of memory so that it can be used as a boot stack during rebooting.*/

 uint8_t reserve_rom_stack[1024];
 uint8_t* reserve_rom_stack_end = reserve_rom_stack+1024;

#define END_BOOT_ROM (uint32_t)&reserve_rom_stack_end

typedef unsigned long PWR_Register;
typedef PWR_Register PWR_jmp_buf[PWR_JUMP_BUF_SIZE];

void PWR_longjmp(PWR_jmp_buf __jmpb, int __retval);
int PWR_setjmp(PWR_jmp_buf __jmpb);

PWR_jmp_buf                   pwr_CPUContext;

/*!
 * brief Function for restoring cpu context
 *
 * return none
 */

void GO_JUMP(){
  PWR_longjmp(pwr_CPUContext, true);
}


bool POWER_EnterPowerDownMode(pm_power_config_t *pm_power_config)
{
  LPC_LOWPOWER_T lp_config;
  bool entered_sleep;

  /* get lp_config */
  POWER_GetPowerDownConfig(pm_power_config, (void*)&lp_config);

  BOOT_SetResumeStackPointer(END_BOOT_ROM);

  SYSCON->CPSTACK = END_BOOT_ROM;

  if ( 0 == PWR_setjmp(pwr_CPUContext))
  {
    POWER_GoToPowerDown(&lp_config);
    entered_sleep = true;
  }else{
    entered_sleep = false;
  }

  return entered_sleep;
}

/*!
 * brief Function for managing configs and putting system to sleep
 *
 * param *bpm_power_config containing configuration structure
 * return true
 */

bool POWER_EnterDeepSleepMode(pm_power_config_t *pm_power_config)
{
  LPC_LOWPOWER_T lp_config;

  /* get lp_config */
  POWER_GetDeepSleepConfig(pm_power_config, (void*)&lp_config);

  POWER_GoToDeepSleep(&lp_config);

  return true;
}

/*!
 * brief Function for converting user-specified configs to config structure usable by power management API in ROM.
 *
 * param *pm_power_config struct containing user-specified configs
 * param *pm_config struct containing converted configs
 *
 * return none
 */

void POWER_GetDeepSleepConfig(pm_power_config_t *pm_power_config, void* pm_config)
{

  /*
     * TODO: Add additional configuration options.
     *
     * This function can be expanded on, but it's hard due to a lack of insight as to the behind-the-scenes functionality of the power management system
     * and ROM functions. Which configurations are supported by which modes isn't entirely clear, but approximations can be made through trial and error.
     *
     * */

  int wakeup_src0;
  int wakeup_src1;

  LPC_LOWPOWER_T* lp_config = (LPC_LOWPOWER_T*)pm_config;

  memset(lp_config, 0, sizeof(LPC_LOWPOWER_T));

  wakeup_src0 = (int)pm_power_config->pm_wakeup_src & 0xFFFFFFFF;

  wakeup_src1 = (int)(pm_power_config->pm_wakeup_src >> 32) & 0xFFFFFFFF;

  /*Passes config for putting system into deep sleep*/
  lp_config->CFG = LOWPOWER_CFG_MODE_DEEPSLEEP;

  /* PDRUNCFG : on ES2, flag discard to keep the same configuration than active */
  /* Example of something that is unclear. This writes to a reserved register. */
  lp_config->CFG |= LOWPOWER_CFG_PDRUNCFG_DISCARD_MASK;

  lp_config->SLEEPPOSTPONE = 0U;
  lp_config->GPIOLATCH     = 0U;

  lp_config->WAKEUPSRCINT0 = wakeup_src0;
  lp_config->WAKEUPSRCINT1 = wakeup_src1;

  /* Configure IO wakeup source */
  if (lp_config->WAKEUPSRCINT1 & LOWPOWER_WAKEUPSRCINT1_IO_IRQ)
  {
    lp_config->WAKEUPIOSRC = pm_power_config->pm_wakeup_io;
  }

  /*maintains GPIO state during sleep*/
  lp_config->GPIOLATCH = POWER_GetIoClampConfig();

}

/*!
 * brief Function for passing configuration structure to ROM API which puts system into sleep.
 *
 * param *pm_config struct containing converted configs
 *
 * return none
 */

void POWER_GoToDeepSleep( void* pm_config )
{

  LPC_LOWPOWER_T* lp_config = (LPC_LOWPOWER_T*)pm_config;

  if ( (void*)power_hook_fn != NULL )
  {
    /* Call hook function */
    (*power_hook_fn)();
  }

  Chip_LOWPOWER_SetLowPowerMode( lp_config );
}

/*!
 * courtesy of PWR_vColdStart in PWC.c in mcux library
 *
 * brief Function for resetting the wakeup timer.
 *
 * return none
 */

void reset_wkt(void) {

  RESET_SetPeripheralReset(kWKT_RST_SHIFT_RSTn);
  RESET_ClearPeripheralReset(kWKT_RST_SHIFT_RSTn);

}

/*!
 * brief Power Library API to enter different power mode.
 *
 * If requested mode is PM_POWER_DOWN, the API will perform the clamping of the DIOs
 * if the PIO register has the bit IO_CLAMPING set: SYSCON->RETENTIONCTRL.IOCLAMP
 * will be set
 *
 * return false if chip could not go to sleep. Configuration structure is incorrect
 *
 * Updated to include PM_DEEP_SLEEP
 */

bool POWER_EnterPowerMode(pm_power_mode_t pm_power_mode, pm_power_config_t *pm_power_config)
{
  bool ret;
  switch (pm_power_mode)
  {
  case PM_DEEP_SLEEP:
    ret = POWER_EnterDeepSleepMode(pm_power_config);
    break;
  case PM_POWER_DOWN:
    ret = POWER_EnterPowerDownMode(pm_power_config);
    break;
  case PM_DEEP_DOWN:
    ret = POWER_EnterDeepDownMode(pm_power_config);
    break;
  default:
    ret = false;
    break;
  }

  return ret;
}

/*!
 * brief Function for initializing wakeup timer 0.
 *
 * param duration in seconds after which wakeup timer will fire
 *
 * return none
 */

void init_config_timer(double time_s){
  CLOCK_EnableClock(kCLOCK_Xtal32k);
  WTIMER_Init();
  WTIMER_EnableInterrupts(WTIMER_TIMER0_ID);
  WTIMER_StartTimer(WTIMER_TIMER0_ID,  (uint32_t)(time_s * 32768U));
}

/*!
 * brief Function uninitializing wakeup timer 0
 *
 * return none
 */

void deinit_config_timer(void){

  WTIMER_StopTimer(WTIMER_TIMER0_ID);
  WTIMER_DisableInterrupts(WTIMER_TIMER0_ID);
  WTIMER_DeInit(); //disable clocks to wakeup timer
  CLOCK_DisableClock(kCLOCK_Xtal32k);
}