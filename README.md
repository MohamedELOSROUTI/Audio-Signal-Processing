# Audio-Signal-Processing
Extract the main fourier component of a sound (hiss, musical note, ...) using the C6748 DSP provided by Texas Instruments and play it. The DSP is programmed using C language. 

We use frame-based signal processing. The CPU receives a block of samples (called frame) which fills the input buffer. This will give more time to the CPU to perform complex operations between 2 frames (FFT). Framed-based processing has many advantages over simple sampled-based signal processing : 

![](https://i.imgur.com/zYt1XEp.png =600x)


We also make use of the DMA "Direct Memory Access" controller integrated to the DSP. It is in charge of filling the input buffer by triggering periodically an interrupt.

The DMA controller takes care of the input/output of samples without interfering with the CPU, which can therefore dedicate its power to signal processing tasks. The DMA controller fills the input buffer and sink the output buffer, at the sampling frequency. When the input buffer is full, the DMA generates an interruption, which allows the CPU to fetch the input samples from the input buffer and send the processed samples to the outbut buffer.

DMA is implemented using two pairs of buffers, called the ‘ping’ and the ‘pong’ buffers. While the ‘ping’ input buffer is being filled with input data and the ‘ping’ output buffer is sent to the codec for output, the content of the ‘pong’ input buffer is processed by the CPU and the results (next output frame) is stored in the ‘pong’ output buffer. At each DMA interruption (‘input buffer full’), the buffers are swapped.

STEP 1            |  STEP 2
:-------------------------:|:-------------------------:
![](https://i.imgur.com/Ohn0XNS.png)|![](https://i.imgur.com/WxDkwOG.png)
  

The file ‘isr.c’ contains:
-the initialization functions,
-the declaration of the buffers (PING,PONG),
-the interrupt function called each time a buffer is full,
-the function ‘process buffer()’ in which the processing must be implemented.



## 1. Objective

The purpose of this project is as follows. The DSP must be able to recover the component fundamental of a sound signal (whistling, musical note, ...) as input and then
generate a sinusoidal signal of the same fundamental frequency at the output.

Certain precautions must be taken so that the program correctly identifies the fundamental frequency. Analyzing this signal is therefore crucial. The input signal can be polluted by noise. Furthermore, we make the assumption that  the input signal contains some harmonics.

## 2. Global structure of the code

The state diagram shown in the following figure summarizes the general organization of the code. This
diagram having four different states, it was decided to use a global variable ***state*** in order to represent these states. 


![](https://i.imgur.com/7ToSKfe.png =600x)

1. state = 1: The background noise Ln is evaluated for a duration Tnoise in order to determine the limit of detection Lthresh.
2. state = 2: The second state consists in waiting to receive a whistle or a note input music. This condition is fulfilled when the effective value of the signal detected by the DSP input exceeds Lthresh. When the waiting is too long, i.e. the time spent in this state exceeds Twait, we return to state = 1.
3. state = 3: The useful signal detected in the previous step is analyzed to determine its fundamental frequency. To do this, the Fourier transform of the input signal is calculated for each block of audio sample, also called batch.
4. state = 4: In the case where a fundamental frequency is determined, a sinewave signal of the same frequency is generated during Tplay. Then a return to the state: state = 1 is achieved through state = 5.

The time required to assess the background noise level Tnoise and the generation time (Tplay) of the output signal are set by the user. The following assumptions are made for the input signal : 


- it must be preceded by a period of silence of at least Tnoise;
- it must be strong enough, i.e. at least 10dB higher than the background noise ;
- it must be stable enough to be able to determine its fundamental frequency in step 3.

## 3. Description of the code

### 3.1 Main variables

- T_in: time during which a window of the background noise level is evaluated, fixed to 0.5 seconds;
- T_noise: total time during which the background noise level is evaluated, fixed to 5 seconds;
- T_wait: maximum waiting time for a signal, fixed to 25 seconds;
- T_play: time during which the note is played, fixed to 5 seconds;
- time_batch: time that a batch takes to be sampled, equal to BUFCOUNT / Fs.
- table_size: constant used for generating the sine wave signal and determining its accuracy, equal to 256.

### 3.2 Main functions

- fillSinus() : initialise the output sinewave vector.
- Square() : computes the squared of a variable (useful to evaluate the background noise level).
- FundamentalFreq(): Function that computes the fundamental frequency of a batch (using FFT).
- generateSignal(): updates the output batch with the new fundamental frequency.
- generateSilence(): initialise to 0 the output batch after the output pitch.
- process_buffer(): main function of the code that executes all the states described above.
- main(): main function.



