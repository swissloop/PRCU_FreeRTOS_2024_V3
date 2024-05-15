/**
 *
 * @file      c_to_cpp.cpp
 * @brief     Since C code cannot call c++ functions directly, this file
 *            serves as a way to glue them together. The declaration of the
 *            functions are located in this c_to_cpp.h, the definition may be 
 *            placed in this file or in any other cpp file.
 *
 * @author    Luca Rufer, luca.rufer@swissloop.ch
 * @date      09.03.2024
 *
 */

#include "Project.hpp"

/*************************************\
 *        Errors and Warnings        *
\*************************************/

void vApplicationMallocFailedHook() {
  ERROR(Error::OutOfMemory);
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc) {
  (void) hadc;
  WARNING(Warning::ADCErrorCallback);
}

// defined in ethernetif.c
// void HAL_ETH_ErrorCallback(ETH_HandleTypeDef *heth) {
//   (void) heth;
//   WARNING(Warning::EthernetErrorCallback);
// }

// void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) {
//   (void) hfdcan;
//   WARNING(Warning::FDCANErrorCallback);
// }

// Defined in ota.c
// void HAL_FLASH_OperationErrorCallback(uint32_t ReturnValue) {
//   (void) ReturnValue;
//   WARNING(Warning::FlashErrorCallback);
// }

// void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
//   (void) hi2c;
//   WARNING(Warning::I2CErrorCallback);
// }

// void HAL_RCCEx_CRS_ErrorCallback(uint32_t Error) {
//   (void) Error;
//   WARNING(Warning::RCCErrorCallback);
// }

// void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
//   (void) hspi;
//   WARNING(Warning::SPIErrorCallback);
// }

void HAL_TIM_ErrorCallback(TIM_HandleTypeDef *htim) {
  (void) htim;
  WARNING(Warning::TimerErrorCallback);
}

// Defined in logging.c
// void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
//   (void) huart;
//   WARNING(Warning::UARTErrorCallback);
// }