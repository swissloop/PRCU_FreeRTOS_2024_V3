/**
 *
 * @file      Project.h
 * @brief     Main project configuration file
 *
 * @author    Philip Wiese, philip.wiese@swissloop.ch
 * @date      04.10.2022
 */

#pragma once
#include "cppapi.h"


/* Global Includes -----------------------------------------------------------*/

/* STL Includes */
#include <cctype>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

/* RTOS Includes */
#include "../../Middlewares/Third_Party/FreeRTOS/Source/include/FreeRTOS.h" //"FreeRTOS.h"
#include "cmsis_os2.h"
#include "queue.h"
#include "semphr.h"
#include "task.h"
#include "timers.h"

/* HAL Includes */
// #include "fdcan.h"

/* External libraries */
//#include "mrrb.h"

/* Global project configuration files */
#include "config.h"

/* Global Exported constants -------------------------------------------------*/

/* Global Exported types -----------------------------------------------------*/

/* Global Exported macros ----------------------------------------------------*/
#include "macros.h"

/* Project Includes ----------------------------------------------------------*/
#include "main.h"
#include "c_to_cpp.h"
#include "notifications.hpp"
#include "status.hpp"
#include "network.hpp"

#include "task_safety.hpp"
#include "task_default.hpp"
#include "task_network.hpp"