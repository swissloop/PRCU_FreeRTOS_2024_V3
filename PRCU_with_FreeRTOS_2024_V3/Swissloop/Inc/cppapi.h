/**
 *
 * @file      cppapi.h
 * @brief     Macros for C/C++ interop
 *
 * @author    Philip Wiese, philip.wiese@swissloop.ch
 * @date      04.10.2022
 */
#pragma once

#ifdef __cplusplus
/**
 * Add an extern "C" region around functions inside a header which is included
 * in both C/C++ code Otherwise linker errors may occur due to name scrambling
 * of C++ functions
 */
#define EXTERN_BEGIN extern "C" {
#define EXTERN_END }
#define EXTERN_C extern "C"
#else
#define EXTERN_BEGIN
#define EXTERN_END
#define EXTERN_C
#endif
