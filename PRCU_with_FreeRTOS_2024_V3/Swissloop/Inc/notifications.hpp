/**
 *
 * @file        notification.hpp
 * @brief       Notification definitions including errors and warnings and inter-task requests
 *
 * @author      Philip Wiese, philip.wiese@swissloop.ch
 * @author      Luca Rufer, luca.rufer@swissloop.ch
 * @date        09.03.2024
 *
 */

#pragma once
#include "cppapi.h"

/* Includes ------------------------------------------------------------------*/

/* Other libraries */
#include "../../Extern/magic_enum/magic_enum.hpp" //"magic_enum/magic_enum.hpp"
using namespace magic_enum::bitwise_operators;

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/*************************************\
 *        Global Notifications       *
\*************************************/

/**
 * @brief Definition of all errors. Errors are defined globally across all tasks.
 * @note Errors should be used if a problem occurs that may have safety-critical 
 * side effects. If the error is not critical (e.g. lost data), use a Warning instead
 */
enum class Error : uint32_t {
  None = 0,        /*!< No Error */

  // General Errors
  HAL,             /*!< Error from the HAL driver */
  Timer,           /*!< Timer failed */
  Abort,           /*!< Action aborted due to Error */
  NotImplemented,  /*!< Action not implemented */
  InvalidArgument, /*!< Invalid argument provided */

  // Inter-Task Communication
  TaskNotify,      /*!< Task Notification failed */

  // Safety Task
  ErrorOverrun,                  /*!< The Safety Task was overrun with errors */
  SafetyNotificationOverrun,     /*!< The Safety Task was overrun with notifications */
  IllegalSafetyNotificationType, /*!< A notification sent to the safety task has an illegal type */

  // State Machine
  IllegalTransition, /*!< Illegal transition has been tried */

  // SD Card Errors
  SdMount,         /*!< SD card mounting */
  SdOpenDirectory, /*!< Open directory on SD card */
  SdOpenFile,      /*!< Open file on SD card */
  SdWriteFile,     /*!< Write file on SD card */
  SdSync,          /*!< Sync file system to SD card */
  SdClose,         /*!< Close file on SD card */

  // Telemetry
  TelemetryDownLinkSocket, /*!< An Error occurred while trying to create a socket for telemetry control messages */
  TelemetryDownLinkBind, /*!< An Error occurred while trying to bind a socket for telemetry control messages */

  // FreeRTOS
  OutOfMemory, /*!< FreeRTOS ran out of Heap memory. Newly requested memory could not be allocated. */

  // Watchdog
  LoggingTimeout, /*!< The Logging Task did not refresh the watchdog in time */

  // Number of possible errors. MUST be the last enumerator!
  ErrorCount
};

/**
 * @brief Definition of all warnings. Warnings are defined globally across all tasks.
 * @note Warnings should be used if a problem occurs that has no safety-critical side
 * effects. If this is not the case, use an Error instead.
 */
enum class Warning : uint32_t {
  None = 0,

  // Network
  CANStartFailed,           /*!< CAN start failed */
  ToBeSentArrayFull,        /*!< The to be sent array of a channel is full */
  NoDataToSend,             /*!< No data to send in the array of the channel */
  TopicNotFound,            /*!< Topic was not found in the send/receive topics array */
  InvalidPriority,          /*!< Invalid priority used */
  InvalidChannelType,       /*!< Channel type used does not exist */
  InvalidMessageLength,     /*!< The message length is an invalid value for the given type */
  InvalidTopicLength,       /*!< The topic length doesn't fit the actual length */
  ReoccuringListFull,       /*!< A reoccuring list is full */
  TopciAlreadyInList,       /*!< The topic is already in the reoccuring list */
  EventNotFound,            /*!< Event was not found in the event array */
  NetworkSocketFailed,      /*!< Network socket creation failed. */
  NoDataReceived,           /*!< No data received from the network */
  MemoryAllocationFailed,   /*!< Memory allocation failed */

  // Error Callbacks (Interrupts)
  ADCErrorCallback,      /*!< ADC Error Callback */
  EthernetErrorCallback, /*!< Ethernet Error Callback */
  FDCANErrorCallback,    /*!< FDCAN Error Callback */
  FlashErrorCallback,    /*!< Flash Error Callback */
  I2CErrorCallback,      /*!< I2C Error Callback */
  OSPIErrorCallback,     /*!< OSPI Error Callback */
  RCCErrorCallback,      /*!< RCC Error Callback */
  SDErrorCallback,       /*!< SD Error Callback */
  SPIErrorCallback,      /*!< SPI Error Callback */
  TimerErrorCallback,    /*!< Timer Error Callback */
  UARTErrorCallback,     /*!< UART Error Callback */

  // Other / Misc

  // Number of possible warnings. MUST be the last enumerator!
  WarningCount
};

/*************************************\
 *     Safety Task Notifications     *
\*************************************/

/**
 * @brief Definition of requests to the Safety Task
 */
enum class TaskSafetyRequest : uint32_t {
  None = 0,
  RESET,            /*!< Reset the state machine */
  TEMPLATE_STATE,   /*!< Go to 'Template' State */
};

/**
 * @brief Definitions of Infos for the safety Tasks
 * @note Infos are used to inform the safety task of actions that other tasks have taken,
 * which might be important for some states or requests of the safety task.
 */
enum class TaskSafetyInfo : uint32_t {
  None = 0,
  FILES_CLOSED,     /*!< The Logging task closed the log files. */
};

/* Exported macros -----------------------------------------------------------*/

#ifdef USE_FULL_ASSERT
#define ERROR(error) User_Error_Handler(error, #error, __FILE__, __LINE__)
#else
#define ERROR(error) User_Error_Handler(error, NULL, NULL, 0)
#endif

#ifdef USE_FULL_ASSERT
#define WARNING(warning) User_Warning_Handler(warning, #warning, __FILE__, __LINE__)
#else
#define WARNING(warning) User_Warning_Handler(warning, NULL, NULL, 0)
#endif

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief User error handler.
 * @note  This function will notify the safety task about the error and log the error.
 */
Error User_Error_Handler(Error err, const char *name, const char *file, uint32_t line);

/**
 * @brief User warning handler.
 * @note  This function will notify the safety task about the warning and log the warning.
 */
Error User_Warning_Handler(Warning warning, const char *name, const char *file, uint32_t line);

/*************************************\
 *    Task Notification Functions    *
\*************************************/

// Functions implemented in the safety task
Error Safety_Notify_Error(Error error);
Error Safety_Notify_Warning(Warning warning);
Error Safety_Notify_Info(TaskSafetyInfo info);
Error Safety_Notify_Request(TaskSafetyRequest request);