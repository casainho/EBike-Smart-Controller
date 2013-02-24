/* Copyright (C) 2009-2010 Michael Moon aka Triffid_Hunter   */
/* Copyright (c) 2011 Jorge Pinto - casainho@gmail.com       */
/* Copyright (c) 2013 Jorge Pinto - casainho@gmail.com       */
/* All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE.
*/

#include	"sermsg.h"
//#include        "comms.h"
#include        "uart.h"

#define serial_writechar(x) uart_send_char(x)

/** \file sermsg.c
        \brief primitives for sending numbers over the serial link
*/

/** write a single hex digit
        \param v hex digit to write, higher nibble ignored
*/
void serwrite_hex4(unsigned char v) {
        v &= 0xF;
        if (v < 10)
                serial_writechar('0' + v);
        else
                serial_writechar('A' - 10 + v);
}

/** write a pair of hex digits
        \param v byte to write. One byte gives two hex digits
*/
void serwrite_hex8(unsigned char v) {
        serwrite_hex4(v >> 4);
        serwrite_hex4(v & 0x0F);
}

/** write four hex digits
        \param v word to write
*/
void serwrite_hex16(unsigned short int v) {
        serwrite_hex8(v >> 8);
        serwrite_hex8(v & 0xFF);
}

/** write eight hex digits
        \param v long word to write
*/
void serwrite_hex32(unsigned long int v) {
        serwrite_hex16(v >> 16);
        serwrite_hex16(v & 0xFFFF);
}

/// list of powers of ten, used for dividing down decimal numbers for sending, and also for our crude floating point algorithm
const unsigned long int powers[] = {1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000};

/** write decimal digits from a long unsigned long int
        \param v number to send
*/
void serwrite_uint32(unsigned long int v) {
        unsigned char e, t;

        for (e = 9; e > 0; e--) {
                if (v >= powers[e])
                        break;
        }

        do
        {
                for (t = 0; v >= powers[e]; v -= powers[e], t++);
                serial_writechar(t + '0');
        }
        while (e--);
}

/** write decimal digits from a long signed int
        \param v number to send
*/
void serwrite_int32(long int v) {
        if (v < 0) {
                serial_writechar('-');
                v = -v;
        }

        serwrite_uint32(v);
}

/** write decimal digits from a long unsigned long int
\param v number to send
*/
void serwrite_uint32_vf(unsigned long int v, unsigned char fp) {
        unsigned char e, t;

        for (e = 9; e > 0; e--) {
                if (v >= powers[e])
                        break;
        }

        if (e < fp)
                e = fp;

        do
        {
                for (t = 0; v >= powers[e]; v -= powers[e], t++);
                serial_writechar(t + '0');
                if (e == fp)
                        serial_writechar('.');
        }
        while (e--);
}

/** write decimal digits from a long signed int
\param v number to send
*/
void serwrite_int32_vf(int v, unsigned char fp) {
        if (v < 0) {
                serial_writechar('-');
                v = -v;
        }

        serwrite_uint32_vf(v, fp);
}
