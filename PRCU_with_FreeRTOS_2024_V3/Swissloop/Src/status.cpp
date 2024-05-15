/**
 *
 * @file        configurations.cpp
 * @brief
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @date        07.12.23
 *
 */

/* Includes ------------------------------------------------------------------*/
// Includes global configuration and all necessary files
#include "Project.hpp"

// Additional includes

/* Private typedef  ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

Status_t status = {};

/* Private variables ---------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

void init_status() {
  // Initialize selected fields
  status.state = Subsystem_States_t::Reset;
  status.state_before_emergency = Subsystem_States_t::Reset;
}
/* Private functions ---------------------------------------------------------*/
