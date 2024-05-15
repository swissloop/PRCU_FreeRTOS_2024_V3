/**
 *
 * @file        state_machine.cpp
 * @brief       Implements all state check and transition check functions
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @author      Luca Rufer, luca.rufer@swissloop.ch
 * @date        04.12.2023
 *
 */

// TODO test state machine!!!

/* Includes ------------------------------------------------------------------*/

// Includes global configuration and all necessary files
#include "Project.hpp"

// State machine
#include "state_machine.hpp"

/* Private typedef  ----------------------------------------------------------*/

/* A function prototype for all state transition functions */
typedef void (*_state_transition_fn)(void);

/* Private defines -----------------------------------------------------------*/

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

inline Subsystem_States_t _get_state() {
  return status.state;
}

inline uint32_t _get_time_since_state_transition() {
  uint32_t current_time = getRunTimeCounterValue();
  return current_time - status.state_transition_time_us;
}

inline void _set_state(Subsystem_States_t state) {
  if (state != _get_state()) {
    status.state_transition_time_us = getRunTimeCounterValue();
  }
  status.state = state;
}

/**
 * @brief handles transition to state EMERGENCY.
 */
void _transition_to_emergency();

/**
 * @brief handles transition to state reset.
 */
void _transition_to_reset();

/**
 * @brief handles transition to state idle.
 */
void _transition_to_idle();

/**
 * @brief handles transition to state template state.
 */
void _transition_to_template_state();

/**
 * @brief checks safety conditions for state RESET
 * @return returns true if all safety values are ok, false if one is violated
 */
bool _reset_safety_checks();

/**
 * @brief checks safety conditions for state IDLE
 * @return returns true if all safety values are ok, false if one is violated
 */
bool _idle_safety_checks();

/**
 * @brief checks safety conditions for state template state
 * @return returns true if all safety values are ok, false if one is violated
 */
bool _template_state_safety_checks();

/**
 * @brief checks for additional errors while already in emergency state
 * @return always returns false
 */
bool _emergency_safety_checks();

/* Private variables ---------------------------------------------------------*/

extern osThreadId_t task_safetyHandle;

/* Exported functions --------------------------------------------------------*/

bool state_machine_safety_checks() {
  // Ensure only the safety task may call this function
  assert_param(xTaskGetCurrentTaskHandle() == (TaskHandle_t) task_safetyHandle);

  // Check if the safety checks for the current state pass
  bool safety_checks_passed = false;
  switch(_get_state()) {
    case Subsystem_States_t::Emergency:
      safety_checks_passed = _emergency_safety_checks();
      break;
    case Subsystem_States_t::Reset:
      safety_checks_passed = _reset_safety_checks();
      break;
    case Subsystem_States_t::Idle:
      safety_checks_passed = _idle_safety_checks();
      break;
    case Subsystem_States_t::TemplateState:
      safety_checks_passed = _template_state_safety_checks();
      break;
  }
  // Safety checks failed. Transition into emergency.
  if (safety_checks_passed == false) {
    _transition_to_emergency();
  }
  return safety_checks_passed;
}

void state_machine_trigger_emergency() {
  // Ensure only the safety task may call this function
  assert_param(xTaskGetCurrentTaskHandle() == (TaskHandle_t) task_safetyHandle);

  // Check if already in emergency
  if (_get_state() == Subsystem_States_t::Emergency) return;

  LOG_Printf("Transitioning to Emergency.\r\n");

  // Transition to emergency
  _transition_to_emergency();

  // Notify the communication task of the transition
  Communication_Notify_Request(TaskCommunicationRequest::SendHeartbeat);
}

bool state_machine_transition_request(Subsystem_States_t requested_state) {
  // Ensure only the safety task may call this function
  assert_param(xTaskGetCurrentTaskHandle() == (TaskHandle_t) task_safetyHandle);

  // Save the current state
  Subsystem_States_t current_state = _get_state();

  // Extract state names
  auto current_state_name = magic_enum::enum_name(current_state);
  auto requested_state_name = magic_enum::enum_name(requested_state);

  // Check if already in the requested state
  if (requested_state == current_state) {
    LOG_Printf("Already in requested state %s.\r\n", std::string(current_state_name).c_str());
    return true;
  }

  _state_transition_fn transition_fn;
  bool transition_allowed = false;

  // Attempt to transition to the requested state.
  // Here, it is only checked if a transition per-se is allowed. 
  // Other checks are performed in the respective functions.
  switch (requested_state) {
    case Subsystem_States_t::Emergency:
      transition_fn = _transition_to_emergency;
      // Always allowed
      transition_allowed = true;
      break;
    case Subsystem_States_t::Reset:
      transition_fn = _transition_to_reset;
      // Always allowed
      transition_allowed = true;
      break;
    case Subsystem_States_t::Idle:
      transition_fn = _transition_to_idle;
      // Only allowed from Reset or template state
      switch (current_state) {
        case Subsystem_States_t::Reset:
        case Subsystem_States_t::TemplateState:
          transition_allowed = true;
          break;
        default:
          transition_allowed = false;
          break;
      }
      break;
    case Subsystem_States_t::TemplateState:
      transition_fn = _transition_to_template_state;
      // Only allowed from Idle state
      switch (current_state) {
        case Subsystem_States_t::Idle:
          transition_allowed = true;
          break;
        default:
          transition_allowed = false;
          break;
      }
      break;
  }

  if (transition_allowed == false) {
    LOG_Printf("State transition from %s to %s not allowed!\r\n", 
                std::string(current_state_name).c_str(), 
                std::string(requested_state_name).c_str());
    ERROR(Error::IllegalTransition);
    return false;
  }

  // Call the selected transition function
  transition_fn();

  // Notify the communication task of the transition
  Communication_Notify_Request(TaskCommunicationRequest::SendHeartbeat);

  Subsystem_States_t transitioned_state = _get_state();
  auto transitioned_state_name = magic_enum::enum_name(transitioned_state);

  // Check if the state was changed to the requested state
  if (transitioned_state == requested_state) {
    LOG_Printf("State transitioned from %s to %s\r\n", 
                std::string(current_state_name).c_str(), 
                std::string(requested_state_name).c_str());
    return true;
  } else {
    LOG_Printf("State did not transition from state %s to %s, but entered into %s instead.\r\n", 
                std::string(current_state_name).c_str(), 
                std::string(requested_state_name).c_str(),
                std::string(transitioned_state_name).c_str());
    return false;
  }
}

void state_machine_automatic_transition_checks() {
  // Ensure only the safety task may call this function
  assert_param(xTaskGetCurrentTaskHandle() == (TaskHandle_t) task_safetyHandle);

  Subsystem_States_t old_state = _get_state();

  switch (_get_state()) {
    case Subsystem_States_t::Emergency:
      break;
    case Subsystem_States_t::Reset:
      // Check for automatic transition to idle state
      if((status.task_init_bools.task_logging_initialized && 
          status.task_init_bools.task_default_initialized &&
          status.task_init_bools.task_communication_initialized &&
          status.task_init_bools.task_telemetry_up_initialized &&
          status.task_init_bools.task_telemetry_down_initialized)) {
        _transition_to_idle();
      }
      break;
    case Subsystem_States_t::Idle:
      break;
    case Subsystem_States_t::TemplateState:
      // Check for automatic transition to Idle state (as example)
      _transition_to_idle();
      break;
    default:
      // Any non-handled state automatically goes into emergency
      LOG_Printf("Automatic transition into emergency because of unknown state %hu.\r\n", (uint8_t) _get_state());
      _transition_to_emergency();
  }

  // In case of an automatic transition, report it.
  if (old_state != _get_state()) {
    auto old_state_name = magic_enum::enum_name(old_state);
    auto new_state_name = magic_enum::enum_name(_get_state());
    LOG_Printf("Automatic state transition from %s to %s\r\n", 
                std::string(old_state_name).c_str(), 
                std::string(new_state_name).c_str());
  }
}

/* Private functions ---------------------------------------------------------*/

/*************************************\
 *     State Transition Functions    *
\*************************************/

void _transition_to_emergency() {
  // Change state
  if (_get_state() != Subsystem_States_t::Emergency) {
    status.state_before_emergency = _get_state();
  }
  _set_state(Subsystem_States_t::Emergency);
  
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_RESET);
}

void _transition_to_reset() {
  // Change state
  _set_state(Subsystem_States_t::Reset);

  // Reset the red emergency LED
  HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET);

  // Clear error and warning flags
  memset(status.error_flags, 0, sizeof(status.error_flags));  
  memset(status.warning_flags, 0, sizeof(status.warning_flags));
}

void _transition_to_idle() {
  // Check if transition allowed
  if (!_idle_safety_checks()) return;

  // Change state
  _set_state(Subsystem_States_t::Idle);

  // @todo do stuff (for example set subsystem ready pin)
}

void _transition_to_template_state() {
  // Check if transition allowed
  if (!_template_state_safety_checks()) return;

  // Change state
  _set_state(Subsystem_States_t::TemplateState);

  // @todo do stuff (for example set an LED to indicate this state)
  HAL_GPIO_WritePin(LED_BLUE_GPIO_Port, LED_BLUE_Pin, GPIO_PIN_SET);
}

/*************************************\
 *        State Safety Checks        *
\*************************************/

bool _emergency_safety_checks() {
  // @todo add your safety checks that should still be checked if already in emergency state (two examples from LCU last year below)

  /*
  //connection with VCU (or ICU) lost
  if(!is_flag(status.err, Error::HeartbeatVCU) && status.use.pod)_check_vcu_heartbeat();

  //pod is not in ready state
  if(!is_flag(status.err, Error::Pod) && status.use.pod && HAL_GPIO_ReadPin(Pod_Ready_GPIO_Port, Pod_Ready_Pin) == GPIO_PIN_RESET) status.err |= Error::Pod;
   */
  return false;
}

bool _reset_safety_checks() {
  //check safety conditions
  bool return_val = true;

  // @todo add your safety checks
  // Check safety value
  if(false /* check safety value*/) return_val = false;

  return return_val;
}

bool _idle_safety_checks() {
  //check safety conditions
  bool return_val = true;

  // @todo add your safety checks
  // Check safety value
  if(false /* check safety value*/) return_val = false;

  return return_val;
}

bool _template_state_safety_checks() {
  //check safety conditions
  bool return_val = true;

  // @todo add your safety checks
  // Check safety value
  if(false /* check safety value*/) return_val = false;

  return return_val;
}
