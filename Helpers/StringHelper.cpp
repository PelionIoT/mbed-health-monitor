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
#include "mbed.h"
#include "StringHelper.h"

/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint8
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint8 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs(char args[32][32], uint8_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%x", &val);
    argsUintPtr[i] = (uint8_t)val;
  }
}

/**
* @brief Process an array of hex alpha numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32(char args[32][32], uint32_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%x", &val);
    argsUintPtr[i] = val;
  }
}

/**
* @brief Process an array of decimal numeric strings representing arguments of
* type uint32
* @param args Array of strings to process
* @param argsUintPtr Pointer of uint32 to save converted arguments
* @param numberOf Number of strings to convert
*/
void ProcessArgs32Dec(char args[32][32], uint32_t *argsUintPtr, int numberOf) {
  int i;
  int val;
  for (i = 0; i < numberOf; i++) {
    sscanf(args[i], "%d", &val);
    argsUintPtr[i] = val;
  }
}

/**
* @brief Parse a single string in decimal format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiDecU32(char *str) {
  unsigned int val;
  sscanf(str, "%u", &val);
  return (uint32_t)val;
}

/**
* @brief Parse a single string in hex format to a uint32 number
* @param str String to process
* @returns Returns the converted number of type uint32
*/
uint32_t ParseAsciiHexU32(char *str) {
  unsigned int val;
  sscanf(str, "%x", &val);
  return (uint32_t)val;
}

/**
* @brief Process a string that is "embedded" within another string using the /"
* delimiters
* @brief Extract the string
* @param str String to process
* @returns Returns the string
*/
void ProcessString(char args[32][32], uint8_t *str, int numberOf) {
  int i;
  int start;
  uint8_t *ptr;
  uint8_t *strPtr;
  ptr = (uint8_t *)args;
  strPtr = str;
  start = 0;
  for (i = 0; i < numberOf; i++) {
    if ((start == 0) && (*ptr == '\"')) {
      start = 1;
      ptr++;
      continue;
    }
    if ((start == 1) && (*ptr == '\"')) {
      break;
    }
    if (start == 1) {
      *strPtr = *ptr;
      strPtr++;
    }
    ptr++;
  }
  // terminate the string
  *strPtr = 0;
}

/**
* @brief Parse an incoming string until a CRLF is encountered, dst will contain
* the parsed string
* @param src source string to process
* @param dst destination string to contain the parsed incoming string
* @param length length of incoming src string
* @returns updated pointer in src string
*/
char *ParseUntilCRLF(char *src, char *dst, int length) {
  int i;
  char *srcPtr;

  srcPtr = src;
  i = 0;
  *dst = 0;
  while ((*srcPtr != 0) && (*srcPtr != 13)) {
    dst[i] = *srcPtr;
    i++;
    if (i >= length)
      break;
    srcPtr++;
  }
  if (*srcPtr == 13)
    srcPtr++;
  if (*srcPtr == 10)
    srcPtr++;
  dst[i] = 0; // terminate the string
  return srcPtr;
}

/**
* @brief Parse an incoming string hex value into an 8-bit value
* @param str string to process
* @returns 8-bit byte that is parsed from the string
*/
uint8_t StringToByte(char str[32]) {
  int val;
  uint8_t byt;
  sscanf(str, "%x", &val);
  byt = (uint8_t)val;
  return byt;
}

/**
* @brief Parse an incoming string hex value into 32-bit value
* @param str string to process
* @returns 32-bit value that is parsed from the string
*/
uint32_t StringToInt(char str[32]) {
  int val;
  uint32_t byt;
  sscanf(str, "%x", &val);
  byt = (uint32_t)val;
  return byt;
}

/**
* @brief Format a binary 8-bit array into a string
* @param uint8Ptr byte buffer to process
* @param numberOf number of bytes in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply(uint8_t *uint8Ptr, int numberOf, char reply[32][32]) {
  int i;
  for (i = 0; i < numberOf; i++) {
    sprintf(reply[i], "%02X", uint8Ptr[i]);
  }
  strcpy(reply[i], "\0");
}

/**
* @brief Format a binary 32-bit array into a string
* @param uint32Ptr 32-bit value buffer to process
* @param numberOf number of values in the buffer
* @param reply an array of strings to place the converted array values into
*/
void FormatReply32(uint32_t *uint32Ptr, int numberOf, char reply[32][32]) {
  int i;
  for (i = 0; i < numberOf; i++) {
    sprintf(reply[i], "%02X", (unsigned int)uint32Ptr[i]);
  }
  strcpy(reply[i], "\0");
}

/**
* @brief Format a binary 8-bit array into a string
* @param data 8-bit value buffer to process
* @param length number of values in the buffer
* @param str output string buffer
* @return output string
*/
char *BytesToHexStr(uint8_t *data, uint32_t length, char *str) {
  uint32_t i;
  char tmpStr[8];
  str[0] = 0;
  for (i = 0; i < length; i++) {
    sprintf(tmpStr, "%02X ", data[i]);
    strcat(str, tmpStr);
  }
  return str;
}
