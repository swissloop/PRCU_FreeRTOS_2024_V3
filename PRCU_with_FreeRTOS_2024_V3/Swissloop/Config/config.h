/**
 *
 * @file       config.h
 * @brief      Configuration header of the project
 *
 * @author     Philip Wiese, philip.wiese@swissloop.ch
 * @author     Luca Rufer, luca.rufer@swissloop.ch
 * @date       25.02.2024
 */

#pragma once
#include "cppapi.h"

/** TODO change this
 * Use preconfigure GPPCB pin mapping
 * @see https://docs.google.com/spreadsheets/d/1hxmHWxa9WQs33SJgPaI9UFUHfWk33p8maKQU9AS5swE/edit#gid=110301558
 * @ref version.h
 */
#define USE_USER_PIN_MAPPING

/*************************************\
 *          General Options          *
\*************************************/


/*************************************\
 *           Task Options            *
\*************************************/

/**
 * Notification wait time
 */
#define TASK_SAFETY_DELAY        5
#define TASK_DEFAULT_DELAY       500

/**
 * LED Delays (in ms)
 */
#define LED_PERIOD_SAFETY       500
#define LED_PERIOD_EMERGENCY    500

/*************************************\
 *        Safety Task Options        *
\*************************************/

/**
 * Number of elements in the Error queue
 */
#define CONFIG_SAFETY_ERROR_QUEUE_LENGTH 16

/**
 * Number of elements in the 'other' queue.
 * This queue handles all messages to the safety task that are not errors.
 */
#define CONFIG_SAFETY_OTHER_QUEUE_LENGTH 16

/*************************************\
 *        Network Task Options       *
\*************************************/

/**
 * Node ID of the network node.
*/
#define CONFIG_NETWORK_NODE_ID 1

/**
 * UDP port used by the network task.
*/
#define CONFIG_NETWORK_PORT 12345

/**
 * Delay for the network task in ms.
*/
#define CONFIG_NETWORK_TASK_DELAY 1

/**
 * Maximum number of topics that can be used in the network.
*/
#define CONFIG_NETWORK_TOPICS_LENGTH 32

/**
 * Maximum number of channels that can be used in the network.
*/
#define CONFIG_NETWORK_CHANNELS_LENGTH 32

/**
 * Maximum delay for sending topics with priority 'high' in ms.
*/
#define CONFIG_NETWORK_HIGH_PRIORITY_DELAY 5

/**
 * Maximum delay for sending topics with priority 'medium' in ms.
*/
#define CONFIG_NETWORK_MEDIUM_PRIORITY_DELAY 20

/**
 * Maximum delay for sending topics with priority 'low' in ms.
*/
#define CONFIG_NETWORK_LOW_PRIORITY_DELAY 100

/**
 * Number of elements in the 'very high priority' event queue.
 * This queue is used to send events to the network task that require immediate attention, thus have very high priority.
*/
#define CONFIG_NETWORK_QUEUE_VERY_HIGH_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'high priority' event queue.
 * This queue is used to send events to the network task that require quick attention, thus have high priority.
*/
#define CONFIG_NETWORK_QUEUE_HIGH_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'medium priority' event queue.
 * This queue is used to send events to the network task that do not require quick attention, thus have medium priority.
*/
#define CONFIG_NETWORK_QUEUE_MEDIUM_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'low priority' event queue.
 * This queue is used to send events to the network task that can wait to be sent, thus have medium priority.
*/
#define CONFIG_NETWORK_QUEUE_LOW_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'high priority' reoccuring event array.
 * This array is used to store reoccuring events that should be sent very often.
*/
#define CONFIG_NETWORK_REOCC_HIGH_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'medium priority' reoccuring event array.
 * This array is used to store reoccuring events that should be sent often.
*/
#define CONFIG_NETWORK_REOCC_MEDIUM_PRIORITY_LENGTH 16

/**
 * Number of elements in the 'low priority' reoccuring event array.
 * This array is used to store reoccuring events that should be sent sparingly.
*/
#define CONFIG_NETWORK_REOCC_LOW_PRIORITY_LENGTH 16

/**
 * Maximum number of topics that can be sent per channel.
*/
#define CONFIG_NETWORK_MAX_TOPICS_TO_BE_SENT_PER_CHANNEL 16

/**
 * Maximum number of messages that can be sent per channel.
*/
#define CONFIG_NETWORK_MAX_MESSAGES_PER_CHANNEL 16

#define CONFIG_NETWORK_USE_FDCAN1 1
#define CONFIG_NETWORK_USE_FDCAN2 0
#define CONFIG_NETWORK_USE_FDCAN3 0

/*************************************\
 *         Reset and Tracing         *
\*************************************/

/**
 *  Behaviour on system exception:
 *  0: Infinite Loop
 *  1: CPU Reset
 */
#define CONFIG_RESET_ON_EXCEPTION 0

/**
 *  Enable the Independent Watchdog.
 *  0: Disabled
 *  1: Enabled
 *  The Watchdog is managed by the safety task.
 */
#define CONFIG_ENABLE_WATCHDOG 1
/**
 * Timeout for the Logging Task
*/
#define WATCHDOG_TASK_LOGGING_TIMEOUT 2000

/**
 *  Enable the Tracealyzer
 *  0: Disabled
 *  1: Enabled
 *  When enabled, the SEGGER Debugger can be used to trace the performance of the Microcontroller.
 *  Note: To enable Tracealyzer, the appropriate include and source files must be included in the 
 *        build process. When building using CMake, set the option 'TRACEALYZER_ENABLE' to 'ON'.
 */
#define CONFIG_ENABLE_TRACEALYZER 0
