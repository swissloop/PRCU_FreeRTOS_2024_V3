/**
 *
 * @file        network.hpp
 * @brief
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @date        \today
 *
 */

#pragma once
#include "cppapi.h"

/* Includes ------------------------------------------------------------------*/
#include "Project.hpp"

/* Exported constants --------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Topic typedefs*/

enum class TopicID : uint8_t {
  Example1 = 0,
  Example2,
};

typedef void (*parser_function_t)(uint8_t*, uint16_t);

typedef struct {
  TopicID public_topic_ID;
  uint8_t *data;
  uint8_t data_length;
} topic_t;

typedef struct {
  topic_t topic;
  uint32_t channels_bitmap;
} send_topic_t;

typedef struct {
  topic_t topic;
  parser_function_t parser;
} receive_topic_t;

/* Channel typedefs*/

enum class ChannelType : uint8_t {
  CAN,
  UDP,
  TCP,
};

typedef struct {
  FDCAN_HandleTypeDef hfdcan;
  uint16_t destination;
  FDCAN_TxHeaderTypeDef header;
} CAN_details_t;

typedef struct {
  uint32_t destination;
} UDP_details_t;

typedef struct {
  uint8_t handle; //placeholder
} TCP_details_t;

typedef struct {
  TopicID public_topic_ID;
  uint8_t *data;
  uint8_t data_length;
} topic_to_be_sent_t;

typedef struct {
  uint8_t* data;
  uint8_t data_length;
} message_t;

typedef struct {
  ChannelType type;
  union {
    CAN_details_t CAN_details;
    UDP_details_t UDP_details;
    TCP_details_t TCP_details;
  };
  topic_to_be_sent_t topics_to_be_sent[CONFIG_NETWORK_MAX_TOPICS_TO_BE_SENT_PER_CHANNEL];
  uint8_t topics_to_be_sent_counter;
  message_t messages[CONFIG_NETWORK_MAX_MESSAGES_PER_CHANNEL];
  uint8_t messages_counter;
} network_channel_t;

/* Exported macros -----------------------------------------------------------*/

/* Exported variables --------------------------------------------------------*/

/* Arrays of Topics */
extern send_topic_t send_topics[CONFIG_NETWORK_TOPICS_LENGTH];
extern receive_topic_t receive_topics[CONFIG_NETWORK_TOPICS_LENGTH];

/* Array of Channels */
extern network_channel_t network_channels[CONFIG_NETWORK_CHANNELS_LENGTH];

extern int network_sd_udp;
extern sockaddr_in network_addresses_UDP[4];

/* Exported functions --------------------------------------------------------*/
void init_topics();
void init_channels();