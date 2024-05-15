/**
 *
 * @file      c_to_cpp.h
 * @brief     Since C code cannot call c++ functions directly, this file
 *            serves as a way to glue them together. The declaration of the
 *            functions are located in this file, the definition may be 
 *            placed in c_to_cpp.cpp or in any other cpp file.
 *
 * @author    Luca Rufer, luca.rufer@swissloop.ch
 * @date      09.03.2024
 *
 */

#pragma once
#include "cppapi.h"

#include "stm32g4xx_hal.h"

EXTERN_BEGIN

/*************************************\
 *               Tasks               *
\*************************************/

/**
 * @brief 	Initialize the safety task and runs task main function.
 * @param 	params   Task parameters to be passed during initialization.
 */
void task_safety_start(void* params);

/**
 * @brief 	Initialize the default task and runs task main function.
 * @param 	params   Task parameters to be passed during initialization.
 */
void task_default(void* params);

/**
 * @brief 	Initialize the network task and runs task main function.
 * @param 	params   Task parameters to be passed during initialization.
 */
void task_network_start(void* params);

/*************************************\
 *      Interrupts and Callbacks     *
\*************************************/

/**
 * @brief DMA1 Stream 0 Callback function; is being executed from stm327xx_it.c; handles UART4 Interrupts
 */
void DMA1_Stream0_Callback(void);

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc);
// void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth);
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);
// void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue);
// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c);
// void HAL_OSPI_ErrorCallback(OSPI_HandleTypeDef *hospi);
// void HAL_RCCEx_CRS_ErrorCallback(uint32_t Error);
// void HAL_SD_ErrorCallback(SD_HandleTypeDef *hsd);
// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim);
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);

/*************************************\
 *      Initialization Functions     *
\*************************************/

/**
 * @brief Initialize the status struct
 */
void init_status(void);

/**
 * @brief Initialize safety queues
 */
void safety_queues_init(void);

/**
 * @brief Initialize network queues
 */
void network_queues_init(void);

/*************************************\
 *        Errors and Warnings        *
\*************************************/

/**
 * @brief Hook called by FreeRTOS when it has no memory left
 */
void vApplicationMallocFailedHook(void);

EXTERN_END