/*
 * Copyright (C) Jorge Pinto aka Casainho, 2012.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL Licence, Version 3
 */

struct received_data
{
  unsigned char len;
  unsigned char buf[256];
  unsigned char packet;
};

void receive_data(unsigned char c, struct received_data *data);
void send_byte(unsigned char c);
void send_writestr(unsigned char *data);
