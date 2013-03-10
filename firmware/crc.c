#include "crc.h"

#define P_16 0xA001

volatile int crc_tab16_init = 0;

volatile unsigned short crc_tab16[256];

/*******************************************************************\
*                                                                   *
*   unsigned short update_crc_16( unsigned short crc, char c );     *
*                                                                   *
*   The function update_crc_16 calculates a  new  CRC-16  value     *
*   based  on  the  previous value of the CRC and the next byte     *
*   of the data to be checked.                                      *
*                                                                   *
\*******************************************************************/

void update_crc_16(unsigned short *crc, char c)
{
  unsigned short tmp, short_c;

  short_c = 0x00ff & (unsigned short) c;

  if ( ! crc_tab16_init ) init_crc16_tab();

  tmp =  *crc       ^ short_c;
  *crc = (*crc >> 8) ^ crc_tab16[ tmp & 0xff ];
}  /* update_crc_16 */

/*******************************************************************\
*                                                                   *
*   static void init_crc16_tab( void );                             *
*                                                                   *
*   The function init_crc16_tab() is used  to  fill  the  array     *
*   for calculation of the CRC-16 with values.                      *
*                                                                   *
\*******************************************************************/

void init_crc16_tab(void)
{
  int i, j;
  unsigned short crc, c;

  for (i=0; i<256; i++) {

      crc = 0;
      c   = (unsigned short) i;

      for (j=0; j<8; j++) {

          if ( (crc ^ c) & 0x0001 ) crc = ( crc >> 1 ) ^ P_16;
          else                      crc =   crc >> 1;

          c = c >> 1;
      }

      crc_tab16[i] = crc;
  }

  crc_tab16_init = 1;

}  /* init_crc16_tab */
