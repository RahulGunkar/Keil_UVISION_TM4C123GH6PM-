/************************************************************************
Copyright (c) 2011, Nic McDonald
All rights reserved.
 
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met: 
 
1. Redistributions of source code must retain the above copyright 
   notice, this list of conditions and the following disclaimer. 
2. Redistributions in binary form must reproduce the above 
   copyright notice, this list of conditions and the following 
   disclaimer in the documentation and/or other materials provided 
   with the distribution. 
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS 
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 
*************************************************************************
 
 Information:
   File Name  :  fifo.h
   Author(s)  :  Nic McDonald
   Hardware   :  Any
   Purpose    :  First In First Out Buffer
 
*************************************************************************
 Modification History:
   Revision   Date         Author    Description of Revision
   1.00       05/30/2011   NGM       initial
 
************************************************************************/
#ifndef _FIFO_H_
#define _FIFO_H_
 
/* includes */
#include <stdint.h>
 
/* defines */
#define FIFO_GOOD       0x00
#define FIFO_OVERFLOW   0x01
#define FIFO_UNDERFLOW  0x02
 
/* typedefs */
typedef struct {
    volatile uint32_t size;
    volatile float* data;
    volatile uint8_t  status;
    volatile uint32_t putIndex;
    volatile uint32_t getIndex;
    volatile uint32_t used;
} FIFO;
 
/* functions */
void     fifo_init(FIFO* f, uint32_t size, float* data);
uint32_t fifo_isFull(FIFO* f);
uint32_t fifo_isEmpty(FIFO* f);
float  fifo_get(FIFO* f);
void     fifo_put(FIFO* f, float c);
float  fifo_peek(FIFO* f);
uint32_t fifo_available(FIFO* f);
void     fifo_clear(FIFO* f);
uint8_t  fifo_status(FIFO* f);
 
#endif // _FIFO_H_

