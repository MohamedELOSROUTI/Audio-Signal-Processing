#include "L138_aic3106_init_edma.h"
#include "prototypes.h"

void main(void) {


	zero_buffers();
	compute_twiddle_factors();
	L138_initialise_edma(FS_8000_HZ, ADC_GAIN_0DB, DAC_ATTEN_0DB);
	
	fillSinus();
	

	while (1) {
		if (is_buffer_full()) {
			process_buffer();
		}
	}
}