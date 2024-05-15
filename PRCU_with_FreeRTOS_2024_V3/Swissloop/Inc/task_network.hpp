/**
 *
 * @file        task_network.hpp
 * @brief       implements the task handling everthing on the network
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @date        30.03.2024
 *
 */

#pragma once
#include "cppapi.h"

/* Includes ------------------------------------------------------------------*/
#include "Project.hpp"

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct {
  uint8_t private_send_topic_ID;
  TopicID public_topic_ID;
} network_event_t;

enum class SendPriority : uint8_t {
  VERY_HIGH,
  HIGH,
  MEDIUM,
  LOW,
};

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

extern QueueHandle_t _queue_network_event_very_high;
extern QueueHandle_t _queue_network_event_high;
extern QueueHandle_t _queue_network_event_medium;
extern QueueHandle_t _queue_network_event_low;

/* Exported functions --------------------------------------------------------*/

/**
 * @brief   Sends a topic once with given priority
 * @param   public_topic_ID  The public topic ID
 * @param   priority         The priority of the message
 *
 * @return  None
*/
void send_topic(TopicID public_topic_ID, SendPriority priority);

/**
 * @brief   Adds a topic to the reoccuring list of a priority
 * @param   public_topic_ID  The public topic ID
 * @param   priority         The priority/reoccurance of the message
 *
 * @return  True if the topic was added successfully, false otherwise
*/
bool add_topic_to_reoccuring_list(TopicID public_topic_ID, SendPriority priority);

/**
 * @brief   Removes a topic from the reoccuring list of a priority
 * @param   public_topic_ID  The public topic ID
 * @param   priority         The priority/reoccurance of the message
 *
 * @return  True if the topic was removed successfully, false otherwise
*/
bool remove_topic_from_reoccuring_list(TopicID public_topic_ID, SendPriority priority);