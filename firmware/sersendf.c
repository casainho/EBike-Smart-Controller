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

#include        "sersendf.h"
#include        <stdarg.h>
#include        "uart.h"
#include        "sermsg.h"
//#include        "comms.h"

#define serial_writechar(x) uart_send_char(x)
#define serial_writestr(x) uart_send_str(x)

/** \brief Simplified printf
        \param format pointer to output format specifier string stored in FLASH.
        \param ... output data

        Implements only a tiny subset of printf's format specifiers :-

        %[ls][udcx%]

        l - following data is (32 bits)\n
        s - following data is short (8 bits)\n
        none - following data is 16 bits.

        u - unsigned long int\n
        d - signed int\n
        c - character\n
        x - hex\n
        % - send a literal % character

        Example:

        \code sersendf_P(PSTR("X:%ld Y:%ld temp:%u.%d flags:%sx Q%su/%su%c\n"), target.X, target.Y, current_temp >> 2, (current_temp & 3) * 25, dda.allflags, mb_head, mb_tail, (queue_full()?'F':(queue_empty()?'E':' '))) \endcode
*/
unsigned char str_ox[] = "0x";

void sersendf(char *format, ...) {
        va_list args;
        va_start(args, format);

        unsigned long int i = 0;
        unsigned char c, j = 0;
        while ((c = format[i++])) {
                if (j) {
                        switch(c) {
                                case 's':
                                        j = 1;
                                        break;
                                case 'l':
                                        j = 4;
                                        break;
                                case 'u':
                                        if (j == 4)
                                                serwrite_uint32(va_arg(args, unsigned long int));
                                        else
                                                serwrite_uint16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'd':
                                        if (j == 4)
                                                serwrite_int32(va_arg(args, long int));
                                        else
                                                serwrite_int16(va_arg(args, int));
                                        j = 0;
                                        break;
                                case 'c':
                                        serial_writechar(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
                                case 'x':
                                        serial_writestr(str_ox);
                                        if (j == 4)
                                                serwrite_hex32(va_arg(args, unsigned long int));
                                        else if (j == 1)
                                                serwrite_hex8(va_arg(args, unsigned int));
                                        else
                                                serwrite_hex16(va_arg(args, unsigned int));
                                        j = 0;
                                        break;
/*                              case 'p':
                                        serwrite_hex16(va_arg(args, unsigned short int));*/
                                case 'q':
                                        serwrite_int32_vf(va_arg(args, long int), 3);
                                        j = 0;
                                        break;
                                default:
                                        serial_writechar(c);
                                        j = 0;
                                        break;
                        }
                }
                else {
                        if (c == '%') {
                                j = 2;
                        }
                        else {
                                serial_writechar(c);
                        }
                }
        }
        va_end(args);
}
