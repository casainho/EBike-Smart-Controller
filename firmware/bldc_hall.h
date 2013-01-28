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

void commutation_disable (void);
void commutation_sector (unsigned int sector);
void commutate (void);
unsigned int get_current_sector (void);
unsigned int increment_sector (unsigned int sector);
unsigned int decrement_sector (unsigned int sector);
