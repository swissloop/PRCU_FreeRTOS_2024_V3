/**
 *
 * @file        configurations.hpp
 * @brief
 *
 * @author      Brian Schnider, brian.schnider@swisslooop.ch
 * @date        07.12.2023
 *
 */

#pragma once
#include "cppapi.h"

/* Includes ------------------------------------------------------------------*/
#include "Project.hpp"

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

// @todo customize these and put them into
enum class Subsystem_States_t : uint8_t {
  Emergency = 1,
  Reset,
  Idle,
  TemplateState,
};

// Bools which are used to check if task is still running and being scheduled, task_safety is not necessary as it is controlling the watchdog
struct WatchdogStatus_t {
  bool task_default;
  bool task_network;
  uint32_t task_logging_last_refresh;
};

struct TaskInitBools_t {
  bool task_default_initialized;
  bool task_network_initialized;
};

// Status of the operation of the microcontroller, can be expanded
struct Status_t {
  Subsystem_States_t state;
  Subsystem_States_t state_before_emergency;
  uint32_t state_transition_time_us;
  uint32_t first_error_time_us;
  Error first_error;
  uint8_t error_flags[(((size_t) Error::ErrorCount) + 7) / 8];
  uint8_t warning_flags[(((size_t) Warning::WarningCount) + 7) / 8];
  volatile WatchdogStatus_t watchdog_status;
  volatile TaskInitBools_t task_init_bools;
};

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/
extern Status_t status;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief initializes the status of the subsystem to default values
 */
EXTERN_C void init_status();
