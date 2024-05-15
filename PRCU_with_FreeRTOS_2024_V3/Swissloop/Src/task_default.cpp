/**
 *
 * @file        task_default.cpp
 * @brief
 *
 * @author      Philip Wiese, philip.wiese@swissloop.ch
 * @date        04.10.2022
 *
 */

/* Includes ------------------------------------------------------------------*/
// Includes global configuration and all necessary files
#include "../Inc/Project.hpp"

/* Private typedef  ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/**
 * @brief   Initialize the default task.
 * @param   params   Task parameters to be passed during initialization.
 *
 * @return  None
 */
void _task_default_init(void *params);

/* Private variables ---------------------------------------------------------*/

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#endif

/* Exported functions --------------------------------------------------------*/
void task_default(void *params) {

  _task_default_init(params);
  LOG_Printf("Task started\r\n");

  status.task_init_bools.task_default_initialized = true;

  for (;;) {
    osDelay(TASK_DEFAULT_DELAY);

#if CONFIG_ENABLE_WATCHDOG
    // Watchdog update for task default
    status.watchdog_status.task_default = true;
#endif
  }

  // should not get here
  configASSERT(0)
}
#ifdef __clang__
#pragma clang diagnostic pop
#endif

/* Private functions ---------------------------------------------------------*/
void _task_default_init(void *params) {
  UNUSED(params);
}
