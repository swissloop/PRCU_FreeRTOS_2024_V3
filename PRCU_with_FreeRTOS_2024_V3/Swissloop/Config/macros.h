/**
 *
 * @file       version.h
 * @brief      Version information of the project
 *
 * @author     Philip Wiese, philip.wiese@swissloop.ch
 * @date       04.10.2022
 */

#pragma once
#include "cppapi.h"
#include "cmsis_compiler.h"

// Modify variables
#define _PACK_16BIT(ptr)      *((uint16_t*)( ptr ))
#define _MASK_12BIT(val)       ( (val) & 0x0FFFL)
#define _MASK_7BIT(val)        ( (val) & 0x7F)

/*! Check for non-null value */
#define VALID(x)                ( (x) != NULL )

/*! Stringify defines */
#define STR(x)                  STR_HELPER(x)
#define STR_HELPER(x)           #x

#define LINE_CLEAR              "\033[K\r"
#define SCREEN_CLEAR            "\033[2J\033[H\r"

/*! Memory-Align buffer length to architecture */
#define BUFFER_ALIGNED_SIZE(x)  ((((x)+31)/32)*32)

/*! Check if in Interrupt */
#define IS_IRQ()                (__get_IPSR() != 0U)

// IP address to integer macro
#define IP_TO_INT(b0, b1, b2, b3) (((b3) << 24) | ((b2) << 16) | ((b1) << 8) | (b0))

// compiler macros
#define __FAST            __attribute__((optimize("Ofast")))
#define __NAKED           __attribute__((naked))

#if defined( __ICCARM__ )
#define DMA_BUFFER _Pragma("location=\".dma_buffer\"")
#define BACKUP_MEMORY _Pragma("location=\".backup\"")
#define RAM_FUNC _Pragma("location=\".RamFunc\"")
#else
#define DMA_BUFFER __attribute__((section(".dma_buffer"), aligned (32)))
#define BACKUP_MEMORY __attribute__((section(".backup"), aligned (32)))
#define RAM_FUNC __attribute__((section(".RamFunc"), aligned(4)))
#endif
