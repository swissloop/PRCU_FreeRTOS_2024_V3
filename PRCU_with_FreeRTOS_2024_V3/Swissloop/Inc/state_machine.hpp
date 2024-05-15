/**
 *
 * @file      state_machine.hpp
 * @brief     Implements the basic state transition functions
 *
 * @author    Brian Schnider, brian.schnider@swissloop.ch
 * @author    Luca Rufer, luca.rufer@swissloop.ch
 * 
 * @date      08.03.2024
 */

#pragma once
#include "cppapi.h"

/* Includes ------------------------------------------------------------------*/
#include "Project.hpp"

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/

/**
 * @brief Trigger the safety checks.
 * 
 * @return true safety checks passed.
 * @return false safety checks failed.
 */
bool state_machine_safety_checks();

/**
 * @brief Trigger a transition into the emergency state.
 */
void state_machine_trigger_emergency();

/**
 * @brief Request a transition into a specific state.
 * 
 * @param requested_state The requested state.
 * @return true  The state machine transitioned into the requested state.
 * @return false The state machine did not transition into the requested state.
 */
bool state_machine_transition_request(Subsystem_States_t requested_state);

/**
 * @brief checks for automatic state transitions
 */
void state_machine_automatic_transition_checks();
