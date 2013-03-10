/*
 * Copyright (C) Jorge Pinto aka Casainho, 2012.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL Licence, Version 3
 */

#include "crc.h"
#include "comms.h"
#include "sersendf.h"
#include "serial_fifo.h"

// TODO hack for building
extern void tx_fifo_send(unsigned char c);

void receive_data(unsigned char c, struct received_data *data)
{
  static unsigned char
          counter = 0,
          bytes_counter = 0,
          state = 0;
  static unsigned short crc;

  switch (state)
  {
    /* wait for first 0xff bytes */
    case 0:
    if (c == 0xff)
    {
      if (counter++ == 3)
      {
        state = 1; // go to next state/stage
        counter = 0;
      }
    }
    else
    {
      counter = 0; // reset, wait for a continuous 0xff
    }
    break;

    /* now save the next byte that is data length */
    case 1:
    data->len = c;

    crc = 0;
    update_crc_16(&crc, c);

    bytes_counter++;
    state = 2;
    break;

    /* get data bytes to data.buf */
    case 2:
    data->buf[bytes_counter - 1] = c;
    bytes_counter++;

    if (bytes_counter <= ((data->len) - 2))
    {
      update_crc_16(&crc, c);
    }

    if (bytes_counter == data->len)
    {
      // Verify if CRC is ok
      if ((data->buf[((data->len) - 3)] == (crc >> 8)) &&
          data->buf[(data->len) - 2] == (crc & 0xff))
      {
        data->len -= 2; // subtract 2 due to 2 CRC number of bytes
        data->packet = 1; // signal a complete packet in data.buf
      }

      bytes_counter = 0;
      state = 0; // reset state machine and try get next data bytes
    }
    break;
  }
}

void send_byte(unsigned char c)
{
  static unsigned char index = 3;
  static unsigned char array[256];
  static unsigned short crc;
  unsigned char i;

  if (c != '\n' && c != '\r')
  { // keep storing until found a new line or return char
    array[index] = c;
    index++;
  }
  else
  { /* build now the packet to send:
    * [0xff] [0xff] [0xff] [0xff] [len] [0xbe] [0xef] [data1 ...]
    *                                                           [crc1]  [crc2]
    */

    // add len
    array[0] = index + 2;

    // add beef
    array[1] = 0xbe;
    array[2] = 0xef;

    // add CRC
    crc = 0;
    for (i = 0; i < (array[0] - 2); i++)
    {
      update_crc_16(&crc, array[i]);
    }
    array[array[0] - 2] = crc >> 8;
    array[array[0] - 1] = crc & 0xff;

    // send packet
    tx_fifo_send(0xff);
    tx_fifo_send(0xff);
    tx_fifo_send(0xff);
    tx_fifo_send(0xff);
    for (i = 0; i < array[0]; i++)
    {
      tx_fifo_send(array[i]);
    }

    index = 3;
  }
}

void send_writestr(unsigned char *data)
{
  unsigned char i = 0, r;

  while ((r = data[i++]))
    send_byte(r);
}
