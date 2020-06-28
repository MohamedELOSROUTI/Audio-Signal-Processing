/*
 * prototypes.h
 *
 *  Created on: 06/06/2020
 *      Author: Mohamed El Osrouti, Nora Folon, Amadis Horbach
 */

#ifndef PROTOTYPES_H_
#define PROTOTYPES_H_

void EDMA3_PaRAM_setup();
int is_buffer_full();
void process_buffer();
void zero_buffers();
void compute_twiddle_factors();

void fillSinus();

#endif /* PROTOTYPES_H_ */
