/*******************************************************************************
 * Copyright (C) 2016 Maxim Integrated Products, Inc., All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
 * OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 *
 * Except as contained in this notice, the name of Maxim Integrated
 * Products, Inc. shall not be used except as stated in the Maxim Integrated
 * Products, Inc. Branding Policy.
 *
 * The mere transfer of this software does not imply any licenses
 * of trade secrets, proprietary technology, copyrights, patents,
 * trademarks, maskwork rights, or any other form of intellectual
 * property whatsoever. Maxim Integrated Products, Inc. retains all
 * ownership rights.
 *******************************************************************************
 */
#ifndef _STRINGHELPER_H_
#define _STRINGHELPER_H_

#include "mbed.h"

/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint8
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint8 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs(char args[32][32], uint8_t *argsUintPtr, int numberOf);
/**
* @brief Format a binary 8-bit array into a string
* @param uint8Ptr byte buffer to process
* @param numberOf number of bytes in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply(uint8_t *uint8Ptr, int numberOf, char reply[32][32]);
/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32(char args[32][32], uint32_t *argsUintPtr, int numberOf);
/**
* @brief Process an array of decimal numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32Dec(char args[32][32], uint32_t *argsUintPtr, int numberOf);
/**
* @brief Format a binary 32-bit array into a string
* @param uint32Ptr 32-bit value buffer to process
* @param numberOf number of values in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply32(uint32_t *uint32Ptr, int numberOf, char reply[32][32]);
/**
* @brief Parse an incoming string hex value into an 8-bit value
* @param str string to process
* @returns 8-bit byte that is parsed from the string
*/
uint8_t StringToByte(char str[32]);
/**
* @brief Parse an incoming string hex value into 32-bit value
* @param str string to process
* @returns 32-bit value that is parsed from the string
*/
uint32_t StringToInt(char str[32]);
/**
* @brief Parse a single string in decimal format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiDecU32(char *str);
/**
* @brief Parse a single string in hex format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiHexU32(char *str);
/**
* @brief Format a binary 8-bit array into a string
* @param data 8-bit value buffer to process
* @param length number of values in the buffer
* @param str output string buffer
* @return output string
*/
char *BytesToHexStr(uint8_t *data, uint32_t length, char *str);
/**
* @brief Parse an incoming string until a CRLF is encountered, dst will contain
* the parsed string
* @param src source string to process
* @param dst destination string to contain the parsed incoming string
* @param length length of incoming src string
* @returns updated pointer in src string
*/
char *ParseUntilCRLF(char *src, char *dst, int length);

#endif // _STRINGHELPER_H_
