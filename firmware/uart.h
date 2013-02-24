/*
 * EBike Smart Controller
 *
 * Copyright (C) Jorge Pinto aka Casainho, 2012, 2013.
 *
 *   casainho [at] gmail [dot] com
 *     www.casainho.net
 *
 * Released under the GPL License, Version 3
 */

void uart_init (void);
void uart_send_char (unsigned char c);
void uart_send_str (unsigned char *data);
void uart_send_buf (unsigned char *buf, unsigned int len);
