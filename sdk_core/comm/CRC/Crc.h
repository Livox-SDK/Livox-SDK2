//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#ifndef CRC_H
#define CRC_H

#include <stdint.h>
#include <stdbool.h>


/*
**********************************************************************************************************************
* Includes
**********************************************************************************************************************
*/

#include "Crc_Cfg.h"               /* Configuration file */



/*
**********************************************************************************************************************
* Defines/Macros
**********************************************************************************************************************
*/


/*
**********************************************************************************************************************
* Type definitions
**********************************************************************************************************************
*/


/*
**********************************************************************************************************************
* Variables
**********************************************************************************************************************
*/


/*
**********************************************************************************************************************
* Extern declarations
**********************************************************************************************************************
*/


/*
 **********************************************************************************************************************
 * Prototypes
 **********************************************************************************************************************
*/

extern uint8_t Crc_CalculateCRC8(const uint8_t* Crc_DataPtr, uint32_t Crc_Length, uint8_t Crc_StartValue8, bool Crc_IsFirstCall);
extern uint8_t Crc_CalculateCRC8H2F(const uint8_t* Crc_DataPtr, uint32_t Crc_Length, uint8_t Crc_StartValue8, bool Crc_IsFirstCall);
extern uint16_t Crc_CalculateCRC16(const uint8_t* Crc_DataPtr, uint32_t Crc_Length, uint16_t Crc_StartValue16, bool Crc_IsFirstCall);
extern uint32_t Crc_CalculateCRC32(const uint8_t* Crc_DataPtr, uint32_t Crc_Length, uint32_t Crc_StartValue32, bool Crc_IsFirstCall);
extern uint32_t Crc_CalculateCRC32P4(const uint8_t* Crc_DataPtr, uint32_t Crc_Length, uint32_t Crc_StartValue32, bool Crc_IsFirstCall);


#endif /* CRC_H */


