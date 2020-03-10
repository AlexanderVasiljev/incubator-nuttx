/****************************************************************************
 * boards/arm/stm32f7/stm32f746-som/src/stm32f746-ws.h
 * ****************************************************************************/

#ifndef __BOARDS_ARM_STM32F746_SOM_SRC_STM32F746_WS_H
#define __BOARDS_ARM_STM32F746_SOM_SRC_STM32F746_WS_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

#define GPIO_PHY_ENABLE			(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | GPIO_OUTPUT_CLEAR | GPIO_PORTG | GPIO_PIN6)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bringup
 *
 * Description:
 *   Perform architecture-specific initialization
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=y :
 *     Called from board_late_initialize().
 *
 *   CONFIG_BOARD_LATE_INITIALIZE=n && CONFIG_LIB_BOARDCTL=y :
 *     Called from the NSH library
 *
 ****************************************************************************/

int stm32_bringup(void);

/****************************************************************************

 * Name: stm32_enablefmc
 *
 * Description:
 *  enable clocking to the FMC module
 *
 ****************************************************************************/

#ifdef CONFIG_STM32F7_FMC
void stm32_enablefmc(void);
#endif

#endif /* __BOARDS_ARM_STM32F746_SOM_SRC_STM32F746_WS_H */
