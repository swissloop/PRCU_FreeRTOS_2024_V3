/**
 *
 * @file        network.cpp
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

/* Private macros  -----------------------------------------------------------*/

/* Private function prototypes  ----------------------------------------------*/

/**
 * @brief   Parses message of the example topic
*/
void example_parser(uint8_t* data, uint16_t length);

/**
 * @brief   Creates the filters for the CAN peripherals
*/
void _create_CAN_filters();

/**
 * @brief   Starts the CAN peripherals
*/
void _start_CAN_peripherals();

/**
 * @brief   Initializes the UDP network
*/
void _init_UDP();

/* Public variables ----------------------------------------------------------*/

/* Arrays of Topics */
send_topic_t send_topics[CONFIG_NETWORK_TOPICS_LENGTH] = {};
receive_topic_t receive_topics[CONFIG_NETWORK_TOPICS_LENGTH] = {};

/* Array of Channels */
network_channel_t network_channels[CONFIG_NETWORK_CHANNELS_LENGTH];

/* Array of UDP Network Addresses */
sockaddr_in network_addresses_UDP[4];

// Network socket descriptor
int network_sd_udp;

/* Private variables ---------------------------------------------------------*/

uint8_t example_send_data[4] = {0xDE, 0xAD, 0xBE, 0xEF};
uint8_t example_receive_data[4];

const struct sockaddr_in control_panel_sockaddr = {
    .sin_len = sizeof(struct sockaddr_in),
    .sin_family = AF_INET,
    .sin_port = PP_HTONS(CONFIG_NETWORK_PORT), // Port of receiver
    .sin_addr = {.s_addr = IP_TO_INT(192, 168, 0, 9)}, // IP of receiver
    .sin_zero = {},
};

// Weak definitions of CAN handles so that compilation does not fail
__weak FDCAN_HandleTypeDef hfdcan1;
__weak FDCAN_HandleTypeDef hfdcan2;
__weak FDCAN_HandleTypeDef hfdcan3;

/* Exported functions --------------------------------------------------------*/

void init_topics() {
    // Add topics one by one
    // Send topics

    // Example topic
    send_topics[0].topic.public_topic_ID = TopicID::Example1;
    send_topics[0].topic.data = example_send_data;
    send_topics[0].topic.data_length = 4;
    send_topics[0].channels_bitmap = 0b0;

    // Receive topics

    // Example topic
    receive_topics[0].topic.public_topic_ID = TopicID::Example2;
    receive_topics[0].topic.data = example_receive_data;
    receive_topics[0].topic.data_length = 4;
    receive_topics[0].parser = example_parser;
}

// Initialize channels
void init_channels() {

    // Example CAN channel
    network_channels[0].type = ChannelType::CAN;
    network_channels[0].CAN_details.hfdcan = hfdcan1;
    network_channels[0].CAN_details.destination = 0; // Bitmap of all CAN receivers
    network_channels[0].topics_to_be_sent_counter = 0;
    network_channels[0].messages_counter = 0;

    // Example UDP channel
    network_channels[1].type = ChannelType::UDP;
    network_channels[1].UDP_details.destination = 0; // Bitmap of all UDP receivers
    network_channels[1].topics_to_be_sent_counter = 0;
    network_channels[1].messages_counter = 0;

    // Init CAN related things
    _create_CAN_filters();
    _start_CAN_peripherals();

    // Init UDP related things
    _init_UDP();
    network_addresses_UDP[0] = control_panel_sockaddr;

}

/* Private functions ---------------------------------------------------------*/

void example_parser(uint8_t* data, uint16_t length) {
    // Example parser
    LOG_Printf("Example parser: \n");
    for (uint16_t i = 0; i < length; i++) {
        LOG_Printf("%d ", data[i]);
    }
}

void _start_CAN_peripherals() {
    // Init of CAN peripheral
#if CONFIG_NETWORK_USE_FDCAN1
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
        WARNING(Warning::CANStartFailed);
        LOG_Printf("Failed to start CAN peripheral\r\n");
    }
#endif
#if CONFIG_NETWORK_USE_FDCAN2
    if (HAL_FDCAN_Start(&hfdcan2) != HAL_OK) {
        WARNING(Warning::CANStartFailed);
        LOG_Printf("Failed to start CAN peripheral\r\n");
    }
#endif
#if CONFIG_NETWORK_USE_FDCAN3
    if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
        WARNING(Warning::CANStartFailed);
        LOG_Printf("Failed to start CAN peripheral\r\n");
    }
#endif
    
}

void _create_CAN_filters() {

#if CONFIG_NETWORK_USE_FDCAN1
    // Accept low priority messages into RXFIFO1
    FDCAN_FilterTypeDef filter1 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = 0b11111111111U,
        .FilterID2 = (0b1U << 10) & CONFIG_NETWORK_NODE_ID,
        .RxBufferIndex = 0,
        .IsCalibrationMsg = 0,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter1);
    // Accept high priority messages into RXFIFO0
    filter1.FilterIndex = 1;
    filter1.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter1.FilterID2 = (0b0U << 10) & CONFIG_NETWORK_NODE_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan1, &filter1);
    // Reject all other messages
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#endif
#if CONFIG_NETWORK_USE_FDCAN2
    // Accept low priority messages into RXFIFO1
    FDCAN_FilterTypeDef filter2 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = 0b11111111111U,
        .FilterID2 = (0b1U << 10) & CONFIG_NETWORK_NODE_ID,
        .RxBufferIndex = 0,
        .IsCalibrationMsg = 0,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter2);
    // Accept high priority messages into RXFIFO0
    filter2.FilterIndex = 1;
    filter2.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter2.FilterID2 = (0b0U << 10) & CONFIG_NETWORK_NODE_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter2);
    // Reject all other messages
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#endif
#if CONFIG_NETWORK_USE_FDCAN3
    // Accept low priority messages into RXFIFO1
    FDCAN_FilterTypeDef filter3 = {
        .IdType = FDCAN_STANDARD_ID,
        .FilterIndex = 0,
        .FilterType = FDCAN_FILTER_MASK,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO1,
        .FilterID1 = 0b11111111111U,
        .FilterID2 = (0b1U << 10) & CONFIG_NETWORK_NODE_ID,
        .RxBufferIndex = 0,
        .IsCalibrationMsg = 0,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan3, &filter3);
    // Accept high priority messages into RXFIFO0
    filter3.FilterIndex = 1;
    filter3.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter3.FilterID2 = (0b0U << 10) & CONFIG_NETWORK_NODE_ID;
    HAL_FDCAN_ConfigFilter(&hfdcan3, &filter3);
    // Reject all other messages
    HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
#endif
}

void _init_UDP() {

    /* Open UDP Socket for communication */
    // Connection-related
    struct sockaddr_in address;
    int network_port = CONFIG_NETWORK_PORT;

    // Setup the port information
    // address.sin_len = sizeof(address);
    address.sin_family = AF_INET;
    address.sin_port = htons(network_port);
    address.sin_addr.s_addr = INADDR_ANY;

    // Create a UDP socket
    if ((network_sd_udp = socket(AF_INET, SOCK_DGRAM, 0)) < 0) { //maybe change 0 in socket() to IPPROTO_UDP
        WARNING(Warning::NetworkSocketFailed);
        LOG_Printf("Failed to open UDP network socket.\n");
        return;
    }

    // Bind the socket to the port
    if (bind(network_sd_udp, (struct sockaddr *)&address, sizeof(address)) < 0) {
        WARNING(Warning::NetworkSocketFailed);
        LOG_Printf("Failed to bind UDP network socket to port.\n");
        return;
    }
}
