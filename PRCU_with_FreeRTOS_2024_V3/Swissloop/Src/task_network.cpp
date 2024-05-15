/**
 *
 * @file        task_network.cpp
 * @brief
 *
 * @author      Brian Schnider, brian.schnider@swissloop.ch
 * @date        \today
 *
 */

/* Includes ------------------------------------------------------------------*/
// Includes global configuration and all necessary files
#include "Project.hpp"

// Additional includes

/* Private typedef  ----------------------------------------------------------*/

/* Private defines -----------------------------------------------------------*/

#define CAN_MAX_MESSAGE_LENGTH 64
#define UDP_MAX_MESSAGE_LENGTH 512
#define TCP_MAX_MESSAGE_LENGTH 512

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/**
 * @brief   Initialize the <name> task.
 * @param   params   Task parameters to be passed during initialization.
 *
 * @return  None
 */
void _task_network_init(void *params);

/**
 * @brief   Checks if a topic is in the reoccuring list of a priority.
 * @param   public_topic_ID  The public topic ID of the topic.
 * @param   priority         The priority of the topic.
 *
 * @return  True if the topic is in the reoccuring list, false otherwise.
 */
bool _check_reocc_list_for_topic(TopicID public_topic_ID, SendPriority priority);

/**
 * @brief   Adds an event to the to be sent arrays of each channel of the topic.
 * @param   event   The event to be added to the channels.
 * 
 * @return  None
 */
void _add_event_to_channels(network_event_t event);

/**
 * @brief   Creates messages of a channel.
 * @details The messages are created by adding topics to the messages until the message would be full.
 * @param   private_channel_ID   The private ID of the channel to create messages for.
 * 
 * @return  None
 */
void _create_messages(uint8_t private_channel_ID);

/**
 * @brief   Sends all channels that have data to be sent.
 * 
 * @return  None
 */
void _send_channels();

/**
 * @brief   Sends the data of a channel.
 * @details The data of a channel is sent over the channel if there is data otherwise nothing is done.
 * @param   private_channel_ID   The private ID of the channel to be sent.
 * 
 * @warning The channel needs to have data to be sent. Otherwise a warning is issued.
 * 
 * @return  None
 */
void _send_channel(uint8_t private_channel_ID);

/**
 * @brief   Gets the private topic ID of a public topic ID.
 * @details Iterates over all topics to find the private topic ID of a public topic ID.
 * @param   public_topic_ID  The public topic ID of the topic.
 * @param   is_send_topic    True if the topic is a send topic, false if it is a receive topic.
 * 
 * @warning If the topic is not found a warning is issued.
 * 
 * @return  The private topic ID of the public topic ID.
 */
uint8_t _get_private_topic_ID(TopicID public_topic_ID, bool is_send_topic);

/**
 * @brief Gets the FDCAN_DLC-style data length from the data length in bytes.
 * @param data_length The data length in bytes.
 * 
 * @return The FDCAN_DLC-style data length.
 */
uint32_t _get_CAN_length_from_datalength(uint8_t data_length);

/**
 * @brief   Receives data from the network.
 * 
 * @return  None
 */
void _receive_from_network();

/**
 * @brief   Checks if a medium (type of channel) has data to be sent.
 * @param   type   The type of the medium to be checked.
 * 
 * @return  True if the medium has data to be sent, false otherwise.
 */
bool _medium_has_data(ChannelType type);

/**
 * @brief   Processes a CAN message.
 * @details Processes a CAN message by getting the data and parsing it.
 * 
 * @return  None
 */
void _process_CAN_message(uint32_t FIFO);

/**
 * @brief   Gets the data length of a CAN message in bytes from the FDCAN_DLC-style length.
 * @param   CANDataLength   The FDCAN_DLC-style length.
 * 
 * @return  The data length in bytes.
 */
uint8_t _get_datalength_from_CAN_length(uint32_t CANDataLength);

/**
 * @brief   Parses the received data.
 * @param   received_data         The received data.
 * @param   received_data_length  The length of the received data.
 * 
 * @return  None
 */
void _parse_data(uint8_t *received_data, uint16_t received_data_length);

/* Private variables ---------------------------------------------------------*/

/* Queues used to receive single events of different priorities */
QueueHandle_t _queue_network_event_very_high = NULL;
QueueHandle_t _queue_network_event_high = NULL;
QueueHandle_t _queue_network_event_medium = NULL;
QueueHandle_t _queue_network_event_low = NULL;

// TODO: Protect these with mutexes
/* Arrays used to store reoccuring events of different priorities */
network_event_t _reocc_network_event_high[CONFIG_NETWORK_REOCC_HIGH_PRIORITY_LENGTH];
network_event_t _reocc_network_event_medium[CONFIG_NETWORK_REOCC_MEDIUM_PRIORITY_LENGTH];
network_event_t _reocc_network_event_low[CONFIG_NETWORK_REOCC_LOW_PRIORITY_LENGTH];
uint8_t _reocc_network_event_high_counter = 0;
uint8_t _reocc_network_event_medium_counter = 0;
uint8_t _reocc_network_event_low_counter = 0;

// Event which was received
network_event_t received_event;

uint32_t _timestamp_last_high_prio_send;
uint32_t _timestamp_last_medium_prio_send;
uint32_t _timestamp_last_low_prio_send;

#ifdef __clang__
#pragma clang diagnostic push
#pragma ide diagnostic ignored "EndlessLoop"
#endif

/* Private inlines -----------------------------------------------------------*/

/**
 * @brief   Checks if a channel has data to be sent.
 * @param   private_channel_ID   The private ID of the channel to be checked.
 * 
 * @return  True if the channel has data to be sent, false otherwise.
 */
inline bool _channel_has_data(uint8_t private_channel_ID) {
  return network_channels[private_channel_ID].topics_to_be_sent[0].data != NULL;
}

/* Exported functions --------------------------------------------------------*/

void network_queues_init(void) {
  _queue_network_event_very_high = xQueueCreate(CONFIG_NETWORK_QUEUE_VERY_HIGH_PRIORITY_LENGTH, sizeof(network_event_t));
  _queue_network_event_high = xQueueCreate(CONFIG_NETWORK_QUEUE_HIGH_PRIORITY_LENGTH, sizeof(network_event_t));
  _queue_network_event_medium = xQueueCreate(CONFIG_NETWORK_QUEUE_MEDIUM_PRIORITY_LENGTH, sizeof(network_event_t));
  _queue_network_event_low = xQueueCreate(CONFIG_NETWORK_QUEUE_LOW_PRIORITY_LENGTH, sizeof(network_event_t));
}

void send_topic(TopicID public_topic_ID, SendPriority priority) {
  uint8_t private_topic_ID = _get_private_topic_ID(public_topic_ID, true);
  if(private_topic_ID == 255) return;

  network_event_t event = {
    .private_send_topic_ID = private_topic_ID,
    .public_topic_ID = public_topic_ID,
  };

  switch(priority) { //TODO add warning in case the queue is full
    case SendPriority::VERY_HIGH:
      xQueueSend(_queue_network_event_very_high, &event, 0);
      break;
    case SendPriority::HIGH:
      xQueueSend(_queue_network_event_high, &event, 0);
      break;
    case SendPriority::MEDIUM:
      xQueueSend(_queue_network_event_medium, &event, 0);
      break;
    case SendPriority::LOW:
      xQueueSend(_queue_network_event_low, &event, 0);
      break;
    default:
      break;
  }
}

bool add_topic_to_reoccuring_list(TopicID public_topic_ID, SendPriority priority) {
  uint8_t private_topic_ID = _get_private_topic_ID(public_topic_ID, true);
  if(private_topic_ID == 255) return false;

  network_event_t event = {
    .private_send_topic_ID = private_topic_ID,
    .public_topic_ID = public_topic_ID,
  };

  switch(priority) {
    case SendPriority::HIGH:
      if(_reocc_network_event_high_counter >= CONFIG_NETWORK_REOCC_HIGH_PRIORITY_LENGTH) {
        WARNING(Warning::ReoccuringListFull);
        return false;
      } else if(_check_reocc_list_for_topic(public_topic_ID, priority)) {
        WARNING(Warning::TopciAlreadyInList);
        return false;
      }
      _reocc_network_event_high[_reocc_network_event_high_counter] = event;
      _reocc_network_event_high_counter++;
      return true;
      break;
    case SendPriority::MEDIUM:
      if(_reocc_network_event_medium_counter >= CONFIG_NETWORK_REOCC_MEDIUM_PRIORITY_LENGTH) {
        WARNING(Warning::ReoccuringListFull);
        return false;
      } else if(_check_reocc_list_for_topic(public_topic_ID, priority)) {
        WARNING(Warning::TopciAlreadyInList);
        return false;
      }
      _reocc_network_event_medium[_reocc_network_event_medium_counter] = event;
      _reocc_network_event_medium_counter++;
      return true;
      break;
    case SendPriority::LOW:
      if(_reocc_network_event_low_counter >= CONFIG_NETWORK_REOCC_LOW_PRIORITY_LENGTH) {
        WARNING(Warning::ReoccuringListFull);
        return false;
      } else if(_check_reocc_list_for_topic(public_topic_ID, priority)) {
        WARNING(Warning::TopciAlreadyInList);
        return false;
      }
      _reocc_network_event_low[_reocc_network_event_low_counter] = event;
      _reocc_network_event_low_counter++;
      return true;
      break;
    default:
      WARNING(Warning::InvalidPriority);
      break;
  }
  return false;
}

bool remove_topic_from_reoccuring_list(TopicID public_topic_ID, SendPriority priority) {

  switch(priority) {
    case SendPriority::HIGH:
      for(uint8_t i = 0; i < _reocc_network_event_high_counter; i++) {
        // Looking for the event in the list
        if(_reocc_network_event_high[i].public_topic_ID == public_topic_ID) {
          // Remove the event from the list by shifting all elements after the event one to the left
          for(uint8_t j = i; j < _reocc_network_event_high_counter - 1; j++) {
            _reocc_network_event_high[j] = _reocc_network_event_high[j + 1];
          }
          _reocc_network_event_high_counter--;
          return true;
        } else if (i == _reocc_network_event_high_counter - 1) {
          WARNING(Warning::EventNotFound);
          return false;
        }
      }
      break;
    case SendPriority::MEDIUM:
      for(uint8_t i = 0; i < _reocc_network_event_medium_counter; i++) {
        // Looking for the event in the list
        if(_reocc_network_event_medium[i].public_topic_ID == public_topic_ID) {
          // Remove the event from the list by shifting all elements after the event one to the left
          for(uint8_t j = i; j < _reocc_network_event_medium_counter - 1; j++) {
            _reocc_network_event_medium[j] = _reocc_network_event_medium[j + 1];
          }
          _reocc_network_event_medium_counter--;
          return true;
        } else if (i == _reocc_network_event_medium_counter - 1) {
          WARNING(Warning::EventNotFound);
          return false;
        }
      }
      break;
    case SendPriority::LOW:
      for(uint8_t i = 0; i < _reocc_network_event_low_counter; i++) {
        // Looking for the event in the list
        if(_reocc_network_event_low[i].public_topic_ID == public_topic_ID) {
          // Remove the event from the list by shifting all elements after the event one to the left
          for(uint8_t j = i; j < _reocc_network_event_low_counter - 1; j++) {
            _reocc_network_event_low[j] = _reocc_network_event_low[j + 1];
          }
          _reocc_network_event_low_counter--;
          return true;
        } else if (i == _reocc_network_event_low_counter - 1) {
          WARNING(Warning::EventNotFound);
          return false;
        }
      }
      break;
    default:
      WARNING(Warning::InvalidPriority);
      break;
  }
  return false;
}

/** @todo make sure to add this function to the tasks.h file */
/** @todo make sure to call this function in freertos.c */
void task_network_start(void *params) {

  // Give the logging task some time to initialize
  osDelay(10);

  _task_network_init(params);
  LOG_Printf("Task started\r\n");

  for (;;) {
    
    // Wait for any event in the very high priority queue or for the delay to expire
    if (xQueueReceive(_queue_network_event_very_high, &received_event, pdMS_TO_TICKS(CONFIG_NETWORK_TASK_DELAY)) == pdPASS) {
      
      // Handle the received event
      _add_event_to_channels(received_event);
      _send_channels();

      while (uxQueueMessagesWaiting(_queue_network_event_very_high) > 0) {
        xQueueReceive(_queue_network_event_very_high, &received_event, 0);
        _add_event_to_channels(received_event);
        _send_channels();
      }
      
    }

    // check if the delay of the high priority queue is over and handle the event
    if (osKernelGetTickCount() - _timestamp_last_high_prio_send >= pdMS_TO_TICKS(CONFIG_NETWORK_HIGH_PRIORITY_DELAY)) {
      // handle the queue entries
      while (uxQueueMessagesWaiting(_queue_network_event_high) > 0) {
        xQueueReceive(_queue_network_event_high, &received_event, 0);
        // handle the received event
        _add_event_to_channels(received_event);
      }
      // Add reoccuring events to the channels
      for (uint8_t event_index = 0; event_index < _reocc_network_event_high_counter; event_index++) {
        _add_event_to_channels(_reocc_network_event_high[event_index]);
      }

      _timestamp_last_high_prio_send = osKernelGetTickCount();
    }

    // check if the delay of the medium priority queue is over and handle the event
    if (osKernelGetTickCount() - _timestamp_last_medium_prio_send >= pdMS_TO_TICKS(CONFIG_NETWORK_MEDIUM_PRIORITY_DELAY)) {
      // handle the queue entries
      while (uxQueueMessagesWaiting(_queue_network_event_medium) > 0) {
        xQueueReceive(_queue_network_event_medium, &received_event, 0);
        // handle the received event
        _add_event_to_channels(received_event);
      }
      // Add reoccuring events to the channels
      for (uint8_t event_index = 0; event_index < _reocc_network_event_medium_counter; event_index++) {
        _add_event_to_channels(_reocc_network_event_medium[event_index]);
      }

      _timestamp_last_medium_prio_send = osKernelGetTickCount();
    }

    // check if the delay of the low priority queue is over and handle the event
    if (osKernelGetTickCount() - _timestamp_last_low_prio_send >= pdMS_TO_TICKS(CONFIG_NETWORK_LOW_PRIORITY_DELAY)) {
      // handle the queue entries
      while (uxQueueMessagesWaiting(_queue_network_event_low) > 0) {
        xQueueReceive(_queue_network_event_low, &received_event, 0);
        // handle the received event
        _add_event_to_channels(received_event);
      }
      // Add reoccuring events to the channels
      for (uint8_t event_index = 0; event_index < _reocc_network_event_low_counter; event_index++) {
        _add_event_to_channels(_reocc_network_event_low[event_index]);
      }

      _timestamp_last_low_prio_send = osKernelGetTickCount();
    }

    // Send all channels that have data to be sent
    _send_channels();

    // Receive data from all channels
    _receive_from_network();

  }

  // should not get here
  configASSERT(0)
}
#pragma clang diagnostic pop

/* Private functions ---------------------------------------------------------*/
void _task_network_init(void *params) {
  UNUSED(params);

  init_topics();
  init_channels();

  // Set the timestamp of the last send to the current time
  _timestamp_last_high_prio_send = osKernelGetTickCount();
  _timestamp_last_medium_prio_send = osKernelGetTickCount();
  _timestamp_last_low_prio_send = osKernelGetTickCount();
}

bool _check_reocc_list_for_topic(TopicID public_topic_ID, SendPriority priority) {
  switch(priority) {
    case SendPriority::HIGH:
      for(uint8_t i = 0; i < _reocc_network_event_high_counter; i++) {
        if(_reocc_network_event_high[i].public_topic_ID == public_topic_ID) {
          return true;
        }
      }
      break;
    case SendPriority::MEDIUM:
      for(uint8_t i = 0; i < _reocc_network_event_medium_counter; i++) {
        if(_reocc_network_event_medium[i].public_topic_ID == public_topic_ID) {
          return true;
        }
      }
      break;
    case SendPriority::LOW:
      for(uint8_t i = 0; i < _reocc_network_event_low_counter; i++) {
        if(_reocc_network_event_low[i].public_topic_ID == public_topic_ID) {
          return true;
        }
      }
      break;
    default:
      WARNING(Warning::InvalidPriority);
      break;
  }
  return false;
}

void _add_event_to_channels(network_event_t event) {

  // Add the event to all channels of the bitmap of the topic
  uint32_t channels_bitmap = send_topics[event.private_send_topic_ID].channels_bitmap;

  // Iterate over all channels
  for(uint8_t private_channel_ID = 0; private_channel_ID < CONFIG_NETWORK_CHANNELS_LENGTH; private_channel_ID++) {
    // Check if the channel is in the bitmap
    if(channels_bitmap & (1 << private_channel_ID)) {

      // if to be sent counter is smaller than the max topics to be sent per channel -> add the event
      if(network_channels[private_channel_ID].topics_to_be_sent_counter < CONFIG_NETWORK_MAX_TOPICS_TO_BE_SENT_PER_CHANNEL) {
        // Add the event to the next free slot
        network_channels[private_channel_ID].topics_to_be_sent[network_channels[private_channel_ID].topics_to_be_sent_counter].public_topic_ID = event.public_topic_ID;
        network_channels[private_channel_ID].topics_to_be_sent[network_channels[private_channel_ID].topics_to_be_sent_counter].data = send_topics[event.private_send_topic_ID].topic.data;
        network_channels[private_channel_ID].topics_to_be_sent[network_channels[private_channel_ID].topics_to_be_sent_counter].data_length = send_topics[event.private_send_topic_ID].topic.data_length;
        network_channels[private_channel_ID].topics_to_be_sent_counter++;
      } else {
        WARNING(Warning::ToBeSentArrayFull);
        LOG_Printf("No free slot available in channel %d to add event %d (public: %d)\r\n", private_channel_ID, event.private_send_topic_ID, (uint8_t) event.public_topic_ID);
      }
    }
  }

}

void _create_messages(uint8_t private_channel_ID) {

  // Get the max message length of the channel
  uint16_t max_message_length = 0;
  switch(network_channels[private_channel_ID].type) {
    case ChannelType::CAN:
      max_message_length = CAN_MAX_MESSAGE_LENGTH;
      break;
    case ChannelType::UDP:
      max_message_length = UDP_MAX_MESSAGE_LENGTH;
      break;
    case ChannelType::TCP:
      max_message_length = TCP_MAX_MESSAGE_LENGTH;
      break;
    default:
      WARNING(Warning::InvalidChannelType);
      break;
  }

  // Variable to keep track of the next topic to be added to a message
  uint8_t next_topic = 0;

  // Iterate over all possible messages and check for each if the next topic in topics_to_sent can be added or if then the message length would be exceeded
  // If a message is full, the next one will be filled and so on until all topics are added to messages
  for (uint8_t message_index = 0; message_index < max_message_length; message_index++) {
    // Check if there is any more data to handle
    if (next_topic >= network_channels[private_channel_ID].topics_to_be_sent_counter) {
      break;
    }

    // Check if the message is already full
    if (network_channels[private_channel_ID].messages[message_index].data_length >= max_message_length) {
      WARNING(Warning::InvalidMessageLength);
      break; // Stop creating messages
    }

    uint8_t* message_pointer = (uint8_t *) pvPortMalloc(max_message_length);
    if (message_pointer == NULL) {
      vPortFree(message_pointer);
      WARNING(Warning::MemoryAllocationFailed);
      return;
    }
    network_channels[private_channel_ID].messages[message_index].data = message_pointer;

    // Iterate over the topics to be sent
    for (uint8_t topic_index = next_topic; topic_index < network_channels[private_channel_ID].topics_to_be_sent_counter; topic_index++) {
      // Get the topic data and length
      uint8_t* topic_data = network_channels[private_channel_ID].topics_to_be_sent[topic_index].data;
      uint8_t topic_data_length = network_channels[private_channel_ID].topics_to_be_sent[topic_index].data_length;

      // Check if adding the topic to the message would exceed the message length
      if (network_channels[private_channel_ID].messages[message_index].data_length + topic_data_length + 1 <= max_message_length) { // +1 for the public topic ID
        uint8_t* public_ID = (uint8_t*) &network_channels[private_channel_ID].topics_to_be_sent[topic_index].public_topic_ID;
        // Add the topic to the message
        memcpy(network_channels[private_channel_ID].messages[message_index].data + network_channels[private_channel_ID].messages[message_index].data_length, public_ID, 1);
        memcpy(network_channels[private_channel_ID].messages[message_index].data + network_channels[private_channel_ID].messages[message_index].data_length + 1, topic_data, topic_data_length);
        network_channels[private_channel_ID].messages[message_index].data_length += topic_data_length + 1;
        next_topic++;
      } else {
        // The message is full, move to the next message
        break;
      }
    }
    network_channels[private_channel_ID].messages_counter = message_index + 1;
  }
}

void _send_channels() {
  // Iterate over all channels
  for(uint8_t private_channel_ID = 0; private_channel_ID < CONFIG_NETWORK_CHANNELS_LENGTH; private_channel_ID++) {
    // Check if the channel has data to be sent and then send it
    if(_channel_has_data(private_channel_ID) == true) {
      _create_messages(private_channel_ID);
      _send_channel(private_channel_ID);
    }
  }
}

void _send_channel(uint8_t private_channel_ID) {
  // Send the data on channel
  switch(network_channels[private_channel_ID].type) {
    case ChannelType::CAN:
      {
        uint8_t message_index = 0;
        while (network_channels[private_channel_ID].messages_counter - message_index > 0) {

          // Getting some info about the give channel
          FDCAN_HandleTypeDef hfdcan = network_channels[private_channel_ID].CAN_details.hfdcan;
          uint16_t destination = network_channels[private_channel_ID].CAN_details.destination;
          uint8_t* data = network_channels[private_channel_ID].messages[message_index].data;
          uint8_t data_length = network_channels[private_channel_ID].messages[message_index].data_length;

          // Converting data_length in bytes to FDCAN data length, if zero is returned, 
          // then data_length is invalid and send is aborted
          uint32_t FDCAN_data_length = _get_CAN_length_from_datalength(data_length);
          if(!FDCAN_data_length) return;

          // TODO: make this easily configurable
          // Setting the Transmit Header
          FDCAN_TxHeaderTypeDef TxHeader = {
            .Identifier = 0,
            .IdType = FDCAN_STANDARD_ID,
            .TxFrameType = FDCAN_DATA_FRAME,
            .DataLength = FDCAN_data_length,
            .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
            .BitRateSwitch = FDCAN_BRS_OFF,
            .FDFormat = FDCAN_FD_CAN,
            .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
            .MessageMarker = 0,
          };

          // Sending one message for each destination in the bitmap
          for (uint8_t i = 1; i < 10; i++) {
            if (destination & (1 << (i - 1))) {
              // Setting the CAN identifier, if _send_channel() is called, 
              // then it is automatically a low priority message
              bool priority = 1; // 1 is low priority, 0 is high priority
              uint32_t identifier = (priority << 10) | i; // Converting destination bitmap into identifier
              TxHeader.Identifier = identifier;

              HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &TxHeader, data);
            }
          }

          message_index++;
        }

        // Checking if no message has been sent
        if (message_index == 0) {
          WARNING(Warning::NoDataToSend);
          LOG_Printf("No data to send in channel %d\r\n", private_channel_ID);
        }
      }
      break;
    case ChannelType::UDP:
      {
        uint8_t message_index = 0;
        while (network_channels[private_channel_ID].messages_counter - message_index > 0) {
          // Send message with UDP
          int sts;
          uint32_t destination = network_channels[private_channel_ID].UDP_details.destination;
          uint8_t* data = network_channels[private_channel_ID].messages[message_index].data;
          uint16_t data_length = network_channels[private_channel_ID].messages[message_index].data_length;

          // Sending one message for each destination in the bitmap
          for (uint8_t i = 1; i < 32; i++) {
            if (destination & (1 << (i - 1))) {
              // Setting the CAN identifier, if _send_channel() is called, 
              // then it is automatically a low priority message
              sockaddr_in destination_address = network_addresses_UDP[i-1];

              // Send the message
              sts = sendto(network_sd_udp, 
                          (void *) data, data_length, 
                          0, 
                          (const struct sockaddr *) &destination_address, sizeof(destination_address));
            
              // Check the send status
              if (sts < 0) {
                // An error occurred while sending to the socket
                LOG_Printf("UDP frame sending failed with errno %i", errno);
              } else if (sts != data_length) {
                // The not all of the data was sent
                LOG_Printf("Only sent %i Bytes of UDP message frame with length %u.\r\n", sts, data_length);
              }
            }
          }

          message_index++;
        }

        // Checking if no message has been sent
        if (message_index == 0) {
          WARNING(Warning::NoDataToSend);
          LOG_Printf("No data to send in channel %d\r\n", private_channel_ID);
        }
      }
      break;
    case ChannelType::TCP:
      // Send data over TCP
      break;
    default:
      break;
  }

  // Clear the topics to be sent of the channel
  for (uint8_t i = 0; i < network_channels[private_channel_ID].topics_to_be_sent_counter; i++) {
    network_channels[private_channel_ID].topics_to_be_sent[i].data = NULL;
    network_channels[private_channel_ID].topics_to_be_sent[i].data_length = 0;
  }
  network_channels[private_channel_ID].topics_to_be_sent_counter = 0;

  // Clear the messages of the channel
  for (uint8_t i = 0; i < network_channels[private_channel_ID].messages_counter; i++) {
    vPortFree(network_channels[private_channel_ID].messages[i].data); // Free Memory
    network_channels[private_channel_ID].messages[i].data = NULL;
    network_channels[private_channel_ID].messages[i].data_length = 0;
  }
  network_channels[private_channel_ID].messages_counter = 0;
  
}

uint8_t _get_private_topic_ID(TopicID public_topic_ID, bool is_send_topic) {
  // Iterate over all topics to find the private topic ID
  if(is_send_topic) {
    for(uint8_t private_topic_ID = 0; private_topic_ID < CONFIG_NETWORK_TOPICS_LENGTH; private_topic_ID++) {
      if(send_topics[private_topic_ID].topic.public_topic_ID == public_topic_ID) {
        return private_topic_ID;
      }
    }
  } else {
    for(uint8_t private_topic_ID = 0; private_topic_ID < CONFIG_NETWORK_TOPICS_LENGTH; private_topic_ID++) {
      if(receive_topics[private_topic_ID].topic.public_topic_ID == public_topic_ID) {
        return private_topic_ID;
      }
    }
  }
  WARNING(Warning::TopicNotFound);
  LOG_Printf("Topic with public ID %d not found\r\n", (uint8_t) public_topic_ID);
  return 255;
}

uint32_t _get_CAN_length_from_datalength(uint8_t data_length) {
  
  if (1 <= data_length && data_length <= 8) {
    return data_length << 16;
  } else if (data_length <= 12) {
    return FDCAN_DLC_BYTES_12;
  } else if (data_length <= 16) {
    return FDCAN_DLC_BYTES_16;
  } else if (data_length <= 20) {
    return FDCAN_DLC_BYTES_20;
  } else if (data_length <= 24) {
    return FDCAN_DLC_BYTES_24;
  } else if (data_length <= 32) {
    return FDCAN_DLC_BYTES_32;
  } else if (data_length <= 48) {
    return FDCAN_DLC_BYTES_48;
  } else if (data_length <= 64) {
    return FDCAN_DLC_BYTES_64;
  } else {
    WARNING(Warning::InvalidMessageLength);
    LOG_Printf("Message length %d exceeds the maximum allowed length for CAN or is zero!\r\n", data_length);
  }

  return 0;
}

void _receive_from_network() {
  
  if(_medium_has_data(ChannelType::CAN)) {

#if CONFIG_NETWORK_USE_FDCAN1
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1)) _process_CAN_message(FDCAN_RX_FIFO1);
#endif
#if CONFIG_NETWORK_USE_FDCAN2
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1)) _process_CAN_message();
#endif
#if CONFIG_NETWORK_USE_FDCAN3
    while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO1)) _process_CAN_message();
#endif
#if CONFIG_NETWORK_USE_FDCAN1
      while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0)) _process_CAN_message(FDCAN_RX_FIFO0);
#endif
#if CONFIG_NETWORK_USE_FDCAN2
      while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0)) _process_CAN_message();
#endif
#if CONFIG_NETWORK_USE_FDCAN3
      while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0)) _process_CAN_message();
#endif

  }
  while(_medium_has_data(ChannelType::UDP)) {
    // Receive data on UDP socket and call the parser function
    int sts;
    uint8_t message[512];

    sts = recvfrom(network_sd_udp, &message, sizeof(message), 0, NULL, NULL);
    // Check receive status
    if (sts < 0) {
      // An error occurred while receiving from the socket
      WARNING(Warning::NoDataReceived);
      LOG_Printf("UDP message reception failed with errno %i", errno);

    } else {
      // The received control frame is valid. Process it.
      _parse_data(message, sts);
    }
  }
  while(_medium_has_data(ChannelType::TCP)) {
    
  }
}

bool _medium_has_data(ChannelType type) {
  bool ret_val = false;
  switch(type) {
    case ChannelType::CAN:
      // Check if any of the FIFO queues has data
#if CONFIG_NETWORK_USE_FDCAN1
      ret_val |= HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) | HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO1);
#endif
#if CONFIG_NETWORK_USE_FDCAN2
      ret_val |= HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO0) | HAL_FDCAN_GetRxFifoFillLevel(&hfdcan2, FDCAN_RX_FIFO1);
#endif
#if CONFIG_NETWORK_USE_FDCAN3
      ret_val |= HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO0) | HAL_FDCAN_GetRxFifoFillLevel(&hfdcan3, FDCAN_RX_FIFO1);
#endif
      break;
    case ChannelType::UDP:
      {
        // Check if there is data available on the socket
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(network_sd_udp, &readfds);
        int result = select(network_sd_udp + 1, &readfds, NULL, NULL, &tv);
        if (result > 0) {
          ret_val = true;
        }

      }
      return false;
      break;
    case ChannelType::TCP:
      // Check if TCP has data
      return false;
      break;
    default:
      break;
  }

  return ret_val;
}

void _process_CAN_message(uint32_t FIFO) {
  uint8_t received_data[64];
  uint8_t received_data_length = 0;
  FDCAN_RxHeaderTypeDef RxHeader = {};

  // Get the data
  HAL_FDCAN_GetRxMessage(&hfdcan1, FIFO, &RxHeader, received_data);
  received_data_length = _get_datalength_from_CAN_length(RxHeader.DataLength);
  // Parse the data
  _parse_data(received_data, received_data_length);
}

uint8_t _get_datalength_from_CAN_length(uint32_t CANDataLength) {
  switch (CANDataLength) {
    case FDCAN_DLC_BYTES_12:
      return 12;
      break;
    case FDCAN_DLC_BYTES_16:
      return 16;
      break;
    case FDCAN_DLC_BYTES_20:
      return 20;
      break;
    case FDCAN_DLC_BYTES_24:
      return 24;
      break;
    case FDCAN_DLC_BYTES_32:
      return 32;
      break;
    case FDCAN_DLC_BYTES_48:
      return 48;
      break;
    case FDCAN_DLC_BYTES_64:
      return 64;
      break;
    default:
      return (CANDataLength >> 16) & 0xF;
      break;
  }
}

void _parse_data(uint8_t *received_data, uint16_t received_data_length) {
  uint16_t already_parsed_data_length = 0;
  while (received_data_length > already_parsed_data_length) {
    // Check the public topic ID
    TopicID public_topic_ID = (TopicID) received_data[already_parsed_data_length];
    uint8_t private_topic_ID = _get_private_topic_ID(public_topic_ID, false);
    if(private_topic_ID == 255) {
      break;
    }

    // Increment the already parsed data length as the public topic ID is already parsed
    already_parsed_data_length++;

    // Check if the data length can be correct (but this is not a guarantee that the data length is correct)
    if (receive_topics[private_topic_ID].topic.data_length <= received_data_length - already_parsed_data_length) {
      // Call the parser function
      receive_topics[private_topic_ID].parser(received_data + already_parsed_data_length, receive_topics[private_topic_ID].topic.data_length);
    } else {
      WARNING(Warning::InvalidTopicLength);
      break;
    }
    already_parsed_data_length += receive_topics[private_topic_ID].topic.data_length;
  }
}
