/**
 *
 * @file        error_handler.cpp
 * @brief
 *
 * @author      Philip Wiese, philip.wiese@swissloop.ch
 * @date        10.10.2022
 *
 */

/* Includes ------------------------------------------------------------------*/
// Includes global configuration and all necessary files
#include "Project.hpp"

/* Private typedef  ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

Error User_Error_Handler(Error err, const char *name, const char *file, uint32_t line) {
  // Only handle non-None errors
  if (err == Error::None) return Error::None;

  // Notify the safety task about the error
  Error err_notify = Safety_Notify_Error(err);

#ifdef USE_FULL_ASSERT
  // Print the error value and description if desired
  if (name != nullptr && !IS_IRQ()) LOG_PrintError(err, file, line);
#endif

  return (err_notify == Error::None) ? err : err_notify;
}

Error User_Warning_Handler(Warning warning, const char *name, const char *file, uint32_t line) {
  // Only handle non-None warnings
  if (warning == Warning::None) return Error::None;

  // Notify the safety task about the error
  Error err_notify = Safety_Notify_Warning(warning);

#ifdef USE_FULL_ASSERT
  // Print the warning value and description if desired
  if (name != nullptr && !IS_IRQ()) LOG_PrintWarning(warning, file, line);
#endif

  return err_notify;
}

/* Private functions ---------------------------------------------------------*/
