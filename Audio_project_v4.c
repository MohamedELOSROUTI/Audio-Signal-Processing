/*
 * isr.c
 *
 *  Created on: 10 janv. 2014
 *      Author: osmju
 */
#include <math.h>
#include "L138_aic3106_init_edma.h"
#include "prototypes.h"
#include "fft.h"
#define Fs 			8000				//sampling frequency
#define PI			3.141592625358979
#define BUFCOUNT	1024
#define BUFLENGTH	(BUFCOUNT * 2)
#define FFTLENGTH	2048

// GLOBAL VARIABLES for the project :

//in seconds :
#define T_in  0.5
#define T_noise 5
#define T_play 5
#define T_wait  25 
#define time_batch (((float) (BUFCOUNT)) / ((float) (Fs)))

#define table_size 256	// precision of the sine table
#define arg1 235//=((int) (T_play/ time_batch))

int state = 1;		//represents the state we are in
int cnt_in_mem =0;
float values_buffer=0;
float T_curr = 0;
float T_waiting = 0;
float T_curr_noise = 0;
float L_thres;
int idx_freq;
long int fundFreq[arg1][2];
int freq_fund;
float sinus[table_size];
int tableIndex = 0;
float noise_max=0;

#pragma DATA_SECTION(ping_IN, ".EXT_RAM")
#pragma DATA_SECTION(pong_IN, ".EXT_RAM")
#pragma DATA_SECTION(ping_OUT, ".EXT_RAM")
#pragma DATA_SECTION(pong_OUT, ".EXT_RAM")

int16_t ping_IN[BUFLENGTH];
int16_t pong_IN[BUFLENGTH];
int16_t ping_OUT[BUFLENGTH];
int16_t pong_OUT[BUFLENGTH];

int16_t *pingIN = &ping_IN[0];
int16_t *pingOUT = &ping_OUT[0];
int16_t *pongIN = &pong_IN[0];
int16_t *pongOUT = &pong_OUT[0];
/* declare complex buffers for twiddle factors and samples */
COMPLEX twiddle_forward[FFTLENGTH];
COMPLEX twiddle_backward[FFTLENGTH];
COMPLEX left_samples_cplx[FFTLENGTH];
COMPLEX right_samples_cplx[FFTLENGTH];

volatile int buffer_full = 0;
int procBuffer;
int cnt_time_play = 0;
bool start_cnt = 0;

/*==========================================================================*/

void compute_twiddle_factors() {
	int n;

	for (n = 0; n < FFTLENGTH; ++n) {
		twiddle_forward[n].real = cos(PI * n / FFTLENGTH);
		twiddle_forward[n].imag = -sin(PI * n / FFTLENGTH);
		twiddle_backward[n].real = cos(PI * n / FFTLENGTH);
		twiddle_backward[n].imag = sin(PI * n / FFTLENGTH);
	}
}

/*==========================================================================*/

void zero_buffers() {
	int i = BUFLENGTH;

	while (i > 0) {
		*pingIN++ = 0;
		*pingOUT++ = 0;
		*pongIN++ = 0;
		*pongOUT++ = 0;

		i--;
	}

	pingIN = &ping_IN[0];
	pingOUT = &ping_OUT[0];
	pongIN = &pong_IN[0];
	pongOUT = &pong_OUT[0];
}

/*==========================================================================*/

int is_buffer_full() {
	return buffer_full;
}

/*==========================================================================*/

void EDMA3_PaRAM_setup() {
	uint32_t *EDMA3_PaRAM_ptr;

	/* param set #1 (McASP Transmit) */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04020);

	*EDMA3_PaRAM_ptr++ = 0x00000000; 																							// OPT: TCINTEN=0, TCC=0, SYNCDIM=0 (A-sync)
	*EDMA3_PaRAM_ptr++ = (unsigned int)pingOUT; 																	// SRC
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004);	// BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = 0x01D02000; 																							// DEST=McASP0 DMA Port
	*EDMA3_PaRAM_ptr++ = 0x00000004; 																							// DSTBIDX=0, SRCBIDX=4
	*EDMA3_PaRAM_ptr++ = 0x00000800; 																							// BCNT=BUFCOUNT, LINK=#64
	*EDMA3_PaRAM_ptr++ = 0x00000000; 																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001; 																							// RSVD=None, CCNT=1

	/* param set #64 */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04800);

	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// OPT: TCINTEN=0, TCC=0, SYNCDIM=0 (A-sync)
	*EDMA3_PaRAM_ptr++ = (unsigned int)pongOUT;																		// SRC
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004);	// BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = 0x01D02000;																							// DEST=McASP0 DMA Port
	*EDMA3_PaRAM_ptr++ = 0x00000004;																							// DSTBIDX=0, SRCBIDX=4
	*EDMA3_PaRAM_ptr++ = 0x00000820;																							// LINK=#65
	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001;																							// RSVD=None, CCNT=1

	/* param set #65 */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04820);

	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// OPT: TCINTEN=0, TCC=0, SYNCDIM=0 (A-sync)
	*EDMA3_PaRAM_ptr++ = (unsigned int)pingOUT;																		// SRC
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004); // BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = 0x01D02000;																							// DEST=McASP0 DMA Por
	*EDMA3_PaRAM_ptr++ = 0x00000004;																							// DSTBIDX=0, SRCBIDX=4
	*EDMA3_PaRAM_ptr++ = 0x00000800;																							// LINK=#64
	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001;																							// RSVD=None, CCNT=1

	/* param set #0 (McASP Receive) */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04000);

	*EDMA3_PaRAM_ptr++ = 0x00100000;																							// OPT: TCINTEN=1, TCC=0, SYNCDIM=0 (A-sync)
	*EDMA3_PaRAM_ptr++ = 0x01D02000;																							// SRC=McASP0 DMA Port
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004);	// BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = (unsigned int)pingIN;																		// DEST
	*EDMA3_PaRAM_ptr++ = 0x00040000;																							// DSTBIDX=4, SRCBIDX=0
	*EDMA3_PaRAM_ptr++ = 0x00000860;																							// LINK=#67
	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001;																							// RSVD=None, CCNT=1

	/* param set #67 */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04860);

	*EDMA3_PaRAM_ptr++ = 0x00101000;																							// OPT: TCINTEN=1, TCC=1, SYNCDIM=0
	*EDMA3_PaRAM_ptr++ = 0x01D02000;																							// SRC=McASP0 DMA Port
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004); // BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = (unsigned int)pongIN;																		// DEST
	*EDMA3_PaRAM_ptr++ = 0x00040000;																							// DSTBIDX=4, SRCBIDX=0
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000880);	// LINK=#68
	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001;																							// RSVD=None, CCNT=1

	/* param set #68 */
	EDMA3_PaRAM_ptr = (unsigned int *)(0x01C04880);

	*EDMA3_PaRAM_ptr++ = 0x00100000;																							// OPT: TCINTEN=1, TCC=0, SYNCDIM=0 (A-sync)
	*EDMA3_PaRAM_ptr++ = 0x01D02000;																							// SRC=McASP0 DMA Port
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000004);	// BCNT=BUFCOUNT, ACNT=4
	*EDMA3_PaRAM_ptr++ = (unsigned int)pingIN;																		// DEST
	*EDMA3_PaRAM_ptr++ = 0x00040000;																							// DSTBIDX=4, SRCBIDX=0
	*EDMA3_PaRAM_ptr++ = (int32_t)((((BUFCOUNT)<<16) & 0xFFFF0000) | 0x00000860);	// LINK=#67
	*EDMA3_PaRAM_ptr++ = 0x00000000;																							// DSTCIDX=None, SRCCIDX=None
	*EDMA3_PaRAM_ptr++ = 0x00000001;																							// RSVD=None, CCNT=1


  EDMA_3CC_IECRH  = 0xffffffff;   // IERH - Disable high interrupts
  EDMA_3CC_EECRH  = 0xffffffff;   // EERH - Disable high events
  EDMA_3CC_ICRH   = 0xffffffff;   // ICRH - Clear high interrupts
  EDMA_3CC_ECRH   = 0xffffffff;   // ICRH - Clear high events
  EDMA_3CC_IECR   = 0xffffffff;   // IER  - Disable low interrupts
  EDMA_3CC_EECR   = 0xffffffff;   // EER  - Disable low events
  EDMA_3CC_ICR    = 0xffffffff;   // ICR  - Clear low interrupts
  EDMA_3CC_ECR    = 0xffffffff;   // ICRH - Clear low events
  EDMA_3CC_EESR = 0x00000003; // enable EDMA3 events 0 and 1, i.e. McASP REVT and XEVT
}

/*==========================================================================*/

interrupt void interrupt4(void) { // interrupt service routine
	switch(EDMA_3CC_IPR) {

	case 1:                     // TCC = 0
	  procBuffer = PING;        // process ping
      EDMA_3CC_ICR = 0x0001;    // clear EDMA3 IPR bit TCC
      break;

    case 2:                     // TCC = 1
      procBuffer = PONG;        // process pong
      EDMA_3CC_ICR = 0x0002;    // clear EDMA3 IPR bit TCC
      break;

    default:                    // may have missed an interrupt
    	EDMA_3CC_ICR = 0x0003;    // clear EDMA3 IPR bits 0 and 1
      break;
	}

	EVTCLR0 = 0x00000100;
	buffer_full = 1;              // flag EDMA3 transfer
  return;
}

/*==========================================================================*/

/**
 * @brief Fill the table 'sinus' with 'table_size' values of the sinus over one 
 *  period
 * 
 */
void fillSinus(){
    int i;
	int amplitude = 10000;
    for(i=0; i<table_size; i++)
        sinus[i] = amplitude*sin(2.0*PI*(float)i/(float)table_size);
}

/*==========================================================================*/
/**
 * @brief Return the square value
 * 
 * @param value is the value to be squared
 * @return float that corresponds to the value squared
 */
float Square(float value)
{
	return value * value;
}

/*==========================================================================*/
/**
 * @brief Computes the sum of the squared level of the values of an array and
 * put the result in the variable 'values_buffer'. It also increments the 
 * counter number of the number of values added, and the time T_curr by the time 
 * a buffer needs to samples these values. 
 * 
 * @param inBuf is an array which contains the values of the batch to be added
 */
void compute_level(int16_t* inBuf)
{
	int i =0;
	for (i = 0; i < BUFCOUNT; i++)
	{
		float vartmp = *inBuf++; //canal de gauche est neglige car defectueux
		values_buffer += Square(*inBuf++);
	}
	cnt_in_mem = cnt_in_mem +BUFCOUNT;
	T_curr = T_curr + time_batch;
}

/*==========================================================================*/

/**
 * @brief Function that computes the fundamental frequency of a batch
 * 
 * @param samples COMPLEX array that contains the fft of a batch
 * @param out pointer that receives the frequency and amplitude of the 
 * fundamental frequency of the corresponding batch
 */
void FundamentalFreq(COMPLEX samples[],long int *out)
{
	int i;
	int k =0;
	float value_cmp1 =0;
	float value_cmp2 =0;
	for (i=0;i<BUFCOUNT; i++)
	{
		value_cmp2 = samples[i].real*samples[i].real + 
				samples[i].imag*samples[i].imag; //amplitude
		if (value_cmp2 > value_cmp1)
		{
			value_cmp1 = value_cmp2;
			k =i;
		}
	}
	long int freq = (long int) k*Fs/FFTLENGTH; 
	long int ampl = (long int) (value_cmp1);
	out[0] = freq;
	out[1] = ampl;

}

/*==========================================================================*/

/**
 * @brief Generates a sinusoidal signal with a specified frequency 'freq_fund'
 * 
 * @param outBuf pointer that receives the values of the sinusoidal signal to
 * be outputted
 */
void generateSignal(int16_t* outBuf){
		
	int i =0;
	for (i = 0; i < BUFCOUNT; i++){

		tableIndex += (int) table_size*freq_fund/Fs;
		tableIndex %= table_size;
		*outBuf++ = sinus[tableIndex];
		*outBuf++ = sinus[tableIndex];
	}
	T_curr = T_curr + time_batch;
}

/*==========================================================================*/

/**
 * @brief Generates a silence
 * 
 * @param outBuf pointer that receives 0 to output a silence
 */
void generateSilence(int16_t* outBuf){
	int i =0;
	for (i = 0; i < BUFCOUNT; i++){
		*outBuf++ = 0;
		*outBuf++ = 0;
	}
}

/*==========================================================================*/
/**
 * @brief Function that process batches in order to detect a note and then
 * play it back for a fixed duration T_play
 * 
 */
void process_buffer(void)
{ 

	int16_t *inBuf, *outBuf;      // pointers to process buffers
	int16_t left_sample, right_sample;
	int i;


	if (procBuffer == PING) {    // use ping or pong buffers

		inBuf = pingIN;
		outBuf = pingOUT;
	}

	if (procBuffer == PONG) {

		inBuf = pongIN;
		outBuf = pongOUT;
	}

	/* Evaluation du niveau de bruit de fond L_n
	*/
	if(state==1)
	{
		compute_level(inBuf);
		
		if(T_curr >= T_in)
		{
			// on calcule l'amplitude moyenne de tous les samples.
			float L_n = values_buffer/cnt_in_mem;
			
			//reset variables
			T_curr_noise += T_curr;
			T_curr = 0;
			cnt_in_mem =0;
			values_buffer=0;
			
			if (L_n> noise_max)
			{
				noise_max = L_n;
			}
			
			if(T_curr_noise>=T_noise)
			{
				L_thres = 10*noise_max;
				
				//reset variables
				T_curr_noise = 0;
				state =2;
				noise_max=0;
				buffer_full = 0; // indicate that buffer has been processed
				return;
			}
		}
		
	}

	/* Détection d'un signal utile 
	*/
	if (state==2)
	{
		compute_level(inBuf);

		if(T_curr >= T_in)
		{
			// calcul de l'amplitude sur 500ms
			float L = values_buffer/cnt_in_mem;
			
			// réinitialise les variables pour calculer l'amplitude sur 500ms 
			// uniquement
			cnt_in_mem =0;
			values_buffer=0;
			T_waiting = T_waiting + T_curr;
			T_curr = 0;
				
			if(L>=L_thres)//signal detecté
			{
				state =3;
				T_waiting =0;
				idx_freq =0;
				buffer_full = 0; // indicate that buffer has been processed
				return;
			}
		}
		if (T_waiting >= T_wait)
		{
			//Si il n'y a pas de signal, on retourne à l'étape 1.
			state =1;

			//réinitialise les variables
			T_waiting =0;
			buffer_full = 0; // indicate that buffer has been processed
			return;
		}
	}
 
 
	/* Détermination de la fréquence fondamentale
	*/
	if (state == 3)
	{

		/* Copy input samples in COMPLEX left and right buffers
		*
		* Do not forget to zero-pad the COMPLEX buffers up to FFTLENGTH.
		* The imaginary part of the signal should be set to 0.
		*/
		for (i = 0; i < BUFCOUNT; i++)
		{
			float vartmp = *inBuf++; // canal gauche est neglige car defectueux

			right_samples_cplx[i].real = *inBuf++;
			left_samples_cplx[i].real = right_samples_cplx[i].real;
			values_buffer += Square(right_samples_cplx[i].real);					
		}

		for (i = BUFCOUNT; i < FFTLENGTH; i++)
		{
			left_samples_cplx[i].real = 0.0;
			right_samples_cplx[i].real = 0.0;
		}

		for (i = 0; i < FFTLENGTH; i++)
		{
			left_samples_cplx[i].imag = 0.0;
			right_samples_cplx[i].imag = 0.0;
		}
  

		/* Vérifie si le signal est toujours joué pour chaque batch : dès que le
		* sifflement s'arrête, il faut calculer la fréquence fondamentale de
		* celui-ci.
		*/
		
		cnt_in_mem = cnt_in_mem +BUFCOUNT;
		float L = values_buffer/cnt_in_mem; 
		cnt_in_mem =0;
		values_buffer=0;
		
		if(L<L_thres) // Le signal utile s'est arrêté.
		{
			if(idx_freq == 0)
			{	// Aucun signal détecté (cela ne devrait pas arriver)
				state = 2;
			}
			else
			{
				long int ampl_max=fundFreq[0][1];
				int cnt =1;
				long int freq_max  = fundFreq[0][0];
				// Compare les différentes fréquences
				for(i=1;i<idx_freq;i++)
				{
					if (fundFreq[i][1] > ampl_max)
					{
						ampl_max = fundFreq[i][1];
						freq_max = fundFreq[i][0];
						cnt = 1;
					}
					else if (fundFreq[i][0] == freq_max)
					{
						cnt = cnt + 1;
					}
				}
				// AU moins 2/3 du signal devrait avoir la même fréquence 
				// fondamentale.
				if (cnt >= (2/3)* idx_freq)
				{
					freq_fund = freq_max;
					state =4;
				}
				else
				{	//Retour à l'étape 2, le signal n'est pas stable
					state =2;
				}
			}
			buffer_full = 0; // indicate that buffer has been processed
			return;
		}

		/*     Compute forward FFT by calling the fft(COMPLEX_INPUT, SIZE, TWIDDLE_FACTORS)
		*     function.
		*/
		fft(left_samples_cplx, FFTLENGTH, twiddle_forward);

		FundamentalFreq(left_samples_cplx,fundFreq[idx_freq]);

		idx_freq = idx_freq+1;
	}
	
	if(state==4)
	{
		generateSignal(outBuf);
		
		if (T_curr >= T_play){
			state=5;
			tableIndex = 0;
			T_curr = 0;
			buffer_full = 0; // indicate that buffer has been processed
			return;
		}

	}
	
	if(state==5)//silence généré avant de retourner à l'étape 1
	{
		generateSilence(outBuf);
		state=1;
	}
		
	buffer_full = 0; // indicate that buffer has been processed
	return;
}


