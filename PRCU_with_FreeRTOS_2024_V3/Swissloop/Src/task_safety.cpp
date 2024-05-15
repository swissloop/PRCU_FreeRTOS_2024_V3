/**
 *
 * @file        task_safety.cpp
 * @brief       the safety task handles the state machine and all safety related checks
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @author      Luca Rufer, luca.rufer@swissloop.ch
 * @date        04.12.2023
 *
 */

/* Includes ------------------------------------------------------------------*/

// Includes global configuration and all necessary files
#include "Project.hpp"

// State machine
#include "state_machine.hpp"

// Additional includes
#include "iwdg.h"

/* Private typedef  ----------------------------------------------------------*/

enum class SafetyNotificationFlags : uint32_t {
  None                = 0,           /*!< No new notification */
  NewError            = (1UL <<  0), /*!< New item in the error Queue */
  ErrorOverrun        = (1UL <<  1), /*!< The Error Queue was overrun with errors */
  NewNotification     = (1UL <<  2), /*!< New item in the 'other' notification Queue */
  NotificationOverrun = (1UL <<  3), /*!< The 'other' notification Queue was overrun with notifications */
};

enum class SafetyNotificationType : uint32_t {
  Warning,
  Request,
  Info,
};

typedef struct {
  SafetyNotificationType type;
  union {
    Warning warning;
    TaskSafetyRequest request;
    TaskSafetyInfo info;
  };
} safety_notification_other_t;

/* Private defines -----------------------------------------------------------*/

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/**
 * @brief   Initialize the safety task.
 * @param   params   Task parameters to be passed during initialization.
 *
 * @return  None
 */
void _task_safety_init(void *params);

/**
 * @brief Notify the safety task of any non-error notification
 * 
 * @param notification The notification for the safety task
 * @return Error The error that occurred during notification, if any.
 */
Error _Safety_Notify_Other(safety_notification_other_t *notification);

/**
 * @brief Internally process a request to the safety task
 * 
 * @param request The request to be processed
 */
void _process_request(TaskSafetyRequest request);

/**
 * @brief Internally process a info notification to the safety task
 * 
 * @param info The info to be processed
 */
void _process_info(TaskSafetyInfo info);

inline bool _any_error() {
  for (uint32_t i = 0; i < sizeof(status.error_flags); i++) {
    if (status.error_flags[i] != 0) return true;
  }
  return false;
}

inline void _add_error(Error error) {
  assert_param(error <= Error::ErrorCount);
  // Check if this is the first error
  if (!_any_error()) {
    status.first_error = error;
    status.first_error_time_us = getRunTimeCounterValue();
  }
  // Set the error flag
  uint32_t error_idx = (uint32_t) error;
  status.error_flags[error_idx / 8] |= 1 << (error_idx % 8);
}

inline bool _is_error_set(Error error) {
  assert_param(error <= Error::ErrorCount);
  uint32_t error_idx = (uint32_t) error;
  return (status.error_flags[error_idx / 8] & (1 << (error_idx % 8)));
}

inline void _add_warning(Warning warning) {
  assert_param(warning <= Warning::WarningCount);
  uint32_t warning_idx = (uint32_t) warning;
  status.warning_flags[warning_idx / 8] |= 1 << (warning_idx % 8);
}

/* Private variables ---------------------------------------------------------*/
/*! Storing timestamp of the task safety init for use in the watchdog */
uint32_t timestamp_safety_init;

/*! Queues used for passing errors, warnings, requests etc to the safety task. */
QueueHandle_t _queue_safety_error = NULL;
QueueHandle_t _queue_safety_other = NULL;

/*!< The handle of the safety task */
extern osThreadId_t task_safetyHandle;

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#endif

/* Exported functions --------------------------------------------------------*/

void safety_queues_init(void) {
  // Initialize the queues for message passing to the safety task
  _queue_safety_error = xQueueCreate(CONFIG_SAFETY_ERROR_QUEUE_LENGTH, sizeof(Error));
  _queue_safety_other = xQueueCreate(CONFIG_SAFETY_OTHER_QUEUE_LENGTH, sizeof(safety_notification_other_t));
}

/**
 * @brief Main of the safety task
 * @param params
 */
void task_safety_start(void *params) {
  uint32_t notificationFlags;
  uint32_t last_led_blink = 0;
  Error error;
  safety_notification_other_t notification;

  // Give the logging task some time to initialize
  osDelay(10);

  _task_safety_init(params);
  LOG_Printf("Task started\r\n");

  for (;;) {

    // Blinking LED periodically to visually check whether task safety is still running
    if(osKernelGetTickCount() - last_led_blink > pdMS_TO_TICKS(LED_PERIOD_SAFETY / 2)) {
      HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
      last_led_blink = osKernelGetTickCount();
    }

    // Periodically get the new notifications from other tasks
    if(xTaskNotifyWait(0, 0xffffffff, &notificationFlags, pdMS_TO_TICKS(TASK_SAFETY_DELAY)) == pdTRUE) {
      // First, check if the error queue was overrun
      if (notificationFlags & ((uint32_t) SafetyNotificationFlags::ErrorOverrun)) {
        _add_error(Error::ErrorOverrun);
      }
      // Check if the notification queue was overrun
      if (notificationFlags & ((uint32_t) SafetyNotificationFlags::NotificationOverrun)) {
        _add_error(Error::SafetyNotificationOverrun);
      }
    }

    // Set the errors in the error queue (if any)
    while (uxQueueMessagesWaiting(_queue_safety_error) > 0) {
      if (xQueueReceive(_queue_safety_error, (void *) &error, 0) == pdTRUE) {
        _add_error(error);
      } 
    }

    // If any errors are set, transition to emergency
    if (_any_error()) {
      state_machine_trigger_emergency();
    }

    // Run safety checks
    state_machine_safety_checks();

    // Perform automatic transition checks
    state_machine_automatic_transition_checks();

    // Check if there are any notifications in the notification queue
    while (uxQueueMessagesWaiting(_queue_safety_other) > 0) {
      if (xQueueReceive(_queue_safety_other, (void *) &notification, 0) == pdTRUE) {
        switch (notification.type) {
          case SafetyNotificationType::Warning:
            _add_warning(notification.warning);  
            break;
          case SafetyNotificationType::Request:
            _process_request(notification.request);
            break;
          case SafetyNotificationType::Info:
            _process_info(notification.info);
            break;
          default:
            // Illegal notification type. Ignore the notification and report the error
            ERROR(Error::IllegalSafetyNotificationType);
            break;
        }
      } 
    }

#if CONFIG_ENABLE_WATCHDOG
    // Refreshing the watchdog timer for critical tasks like the communication task
    // TODO: Add critical tasks to this section (and maybe remove default task) and set their watchdog to true in the respective task
    if(status.watchdog_status.task_default) {
      HAL_IWDG_Refresh(&hiwdg1);
      status.watchdog_status.task_default = false;
    }

    // // Refreshing the "watchdog" timer for non-critical tasks like logging
    // // TODO: (optional) add more non-critical tasks to this section and set their watchdog to true in the respective task
    // if(status.watchdog_status.task_logging) {
    //   status.watchdog_status.task_logging = false;
    //   status.watchdog_status.task_logging_last_refresh = osKernelGetTickCount();
    // } else if((osKernelGetTickCount() - status.watchdog_status.task_logging_last_refresh >= pdMS_TO_TICKS(WATCHDOG_TASK_LOGGING_TIMEOUT)) \
    //            && osKernelGetTickCount() - timestamp_safety_init >= pdMS_TO_TICKS(WATCHDOG_TASK_LOGGING_TIMEOUT)) {
    //   _add_error(Error::LoggingTimeout);
    //   state_machine_trigger_emergency();
    //   LOG_Printf("Logging task timed out!\r\n");
    //   LOG_Printf("Time since last refresh: %lu, current time: %lu\r\n", status.watchdog_status.task_logging_last_refresh, osKernelGetTickCount());
    // }
#endif /* CONFIG_ENABLE_WATCHDOG */

  }

  // should not get here
  configASSERT(0)
}

#ifdef __clang__
#pragma clang diagnostic pop
#endif

Error Safety_Notify_Error(Error error) {
  BaseType_t xReturn, yield;
  SafetyNotificationFlags notification_flag;

  // Put the Error into the error Queue
  if (IS_IRQ()) {
    yield = pdFALSE;
    xReturn = xQueueSendFromISR(_queue_safety_error, &error, &yield);
    portYIELD_FROM_ISR(yield);
  } else {
    xReturn = xQueueSend(_queue_safety_error, &error, 0);
  }

  // Notify the safety task depending on the queue status
  notification_flag = (xReturn == pdPASS) ? SafetyNotificationFlags::NewError : SafetyNotificationFlags::ErrorOverrun;

  // Notify the safety Task
  if (IS_IRQ()) {
    yield = pdFALSE;
    xReturn = xTaskNotifyFromISR((TaskHandle_t) task_safetyHandle, (uint32_t) notification_flag, eSetBits, &yield);
    portYIELD_FROM_ISR(yield);
  } else {
    xReturn = xTaskNotify((TaskHandle_t) task_safetyHandle, (uint32_t) notification_flag, eSetBits);
  }

  // Return while preventing recursive error throwing
  if (xReturn == pdPASS || error == Error::TaskNotify || notification_flag == SafetyNotificationFlags::ErrorOverrun) {
    return Error::None;
  } else {
    return ERROR(Error::TaskNotify);
  }
}

Error Safety_Notify_Warning(Warning warning) {
  safety_notification_other_t notification = {
    .type = SafetyNotificationType::Warning,
    .warning = warning,
  };
  return _Safety_Notify_Other(&notification);
}

Error Safety_Notify_Info(TaskSafetyInfo info) {
  safety_notification_other_t notification = {
    .type = SafetyNotificationType::Info,
    .info = info,
  };
  return _Safety_Notify_Other(&notification);
}

Error Safety_Notify_Request(TaskSafetyRequest request) {
  safety_notification_other_t notification = {
    .type = SafetyNotificationType::Request,
    .request = request,
  };
  return _Safety_Notify_Other(&notification);
}

/* Private functions ---------------------------------------------------------*/

void _task_safety_init(void *params) {
  UNUSED(params);

  // Wait for the other tasks to finish initialization
  while(!status.task_init_bools.task_default_initialized) {
    osDelay(1);
  }

#if CONFIG_ENABLE_WATCHDOG
  // Allow the Debugger to freeze the watchdog timer while debugging
#if DEBUG
  __HAL_DBGMCU_FREEZE_IWDG1();
#endif /* DEBUG */

  // Initialize the Watchdog
  MX_IWDG1_Init();
#endif /* CONFIG_ENABLE_WATCHDOG */

  // Safe timestamp of task safety init
  timestamp_safety_init = osKernelGetTickCount();
}

Error _Safety_Notify_Other(safety_notification_other_t *notification) {
  BaseType_t xReturn, yield;
  SafetyNotificationFlags notification_flag;

  // Put the Request into the Queue
  if (IS_IRQ()) {
    yield = pdFALSE;
    xReturn = xQueueSendFromISR(_queue_safety_other, notification, &yield);
    portYIELD_FROM_ISR(yield);
  } else {
    xReturn = xQueueSend(_queue_safety_other, notification, 0);
  }

  // Notify the safety task depending on the queue status
  notification_flag = (xReturn == pdPASS) ? SafetyNotificationFlags::NewNotification : 
                                            SafetyNotificationFlags::NotificationOverrun;

  // Notify the safety Task
  if (IS_IRQ()) {
    yield = pdFALSE;
    xReturn = xTaskNotifyFromISR((TaskHandle_t) task_safetyHandle, (uint32_t) notification_flag, eSetBits, &yield);
    portYIELD_FROM_ISR(yield);
  } else {
    xReturn = xTaskNotify((TaskHandle_t) task_safetyHandle, (uint32_t) notification_flag, eSetBits);
  }

  // Return
  if (xReturn == pdPASS) {
    return Error::None;
  } else {
    return ERROR(Error::TaskNotify);
  }
}

void _process_request(TaskSafetyRequest request) {
  // No default case on purpose: Compiler complains if any request is not handled
  switch (request) {
    case TaskSafetyRequest::None:
      break;
    case TaskSafetyRequest::RESET:
      state_machine_transition_request(Subsystem_States_t::Reset);
      break;
    case TaskSafetyRequest::TEMPLATE_STATE:
      state_machine_transition_request(Subsystem_States_t::TemplateState);
      break;
  }
}

void _process_info(TaskSafetyInfo info) {
  // No default case on purpose: Compiler complains if any info is not handled
  switch (info) {
    case TaskSafetyInfo::None:
      break;
    case TaskSafetyInfo::FILES_CLOSED:
      break;
  }
}
