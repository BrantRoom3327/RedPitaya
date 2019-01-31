/**
 * $Id: generate.c 1246 2014-06-02 09:07am pdorazio $
 *
 * @brief Red Pitaya simple signal/function generator with pre-defined
 *        signal types.
 *
 * @Author Ales Bardorfer <ales.bardorfer@redpitaya.com>
 *
 * (c) Red Pitaya  http://www.redpitaya.com
 *
 * This part of code is written in C programming language.
 * Please visit http://en.wikipedia.org/wiki/C_(programming_language)
 * for more details on the language used herein.
 */

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "fpga_awg.h"
#include "redpitaya/version.h"

/**
 * GENERAL DESCRIPTION:
 *
 * The code below performs a function of a signal generator, which produces
 * a a signal of user-selectable pred-defined Signal shape
 * [Sine, Square, Triangle], Amplitude and Frequency on a selected Channel:
 *
 *
 *                   /-----\
 *   Signal shape -->|     | -->[data]--+-->[FPGA buf 1]--><DAC 1>
 *   Amplitude ----->| AWG |            |
 *   Frequency ----->|     |             -->[FPGA buf 2]--><DAC 2>
 *                   \-----/            ^
 *                                      |
 *   Channel ---------------------------+ 
 *
 *
 * This is achieved by first parsing the four parameters defining the 
 * signal properties from the command line, followed by synthesizing the 
 * signal in data[] buffer @ 125 MHz sample rate within the
 * generate_signal() function, depending on the Signal shape, Amplitude
 * and Frequency parameters. The data[] buffer is then transferred
 * to the specific FPGA buffer, defined by the Channel parameter -
 * within the write_signal_fpga() function.
 * The FPGA logic repeatably sends the data from both FPGA buffers to the
 * corresponding DACs @ 125 MHz, which in turn produces the synthesized
 * signal on Red Pitaya SMA output connectors labeled DAC1 & DAC2.
 *
 */

/** Maximal signal frequency [Hz] */
const double c_max_frequency = 62.5e6;

/** Minimal signal frequency [Hz] */
const double c_min_frequency = 0;

/** Maximal signal amplitude [Vpp] */
const double c_max_amplitude = 2.0;

/** AWG buffer length [samples]*/
#define BUFFER_LEN (16*1024)

/** AWG data buffer */
int32_t data[BUFFER_LEN];

/** Signal types */
typedef enum {
    eSignalSine,         ///< Sinusoidal waveform.
    eSignalSquare,       ///< Square waveform.
    eSignalTriangle,     ///< Triangular waveform.
    eSignalSweep         ///< Sinusoidal frequency sweep.
} signal_e;

/** AWG FPGA parameters */
typedef struct {
    int32_t  offsgain;   ///< AWG offset & gain.
    uint32_t wrap;       ///< AWG buffer wrap value.
    uint32_t step;       ///< AWG step interval.
} awg_param_t;

/* Forward declarations */
void synthesize_signal(double ampl, double freq, signal_e type, double endfreq, int32_t *data, awg_param_t *params);
void write_data_fpga(uint32_t ch, const int32_t *data, const awg_param_t *awg);

int fireASG(double frequency, float amplitude, const char* waveType)
{
    //don't do sweeps right now, if we can do sweeps in the future at different
    //frequencys add it in.
	double endFrequency = frequency;

    /* Signal type argument parsing */
    signal_e type = eSignalSine;
	if (strcmp(waveType, "sine") == 0) {
		type = eSignalSine;
	} else if (strcmp(waveType, "sqr") == 0) {
		type = eSignalSquare;
	} else if (strcmp(waveType, "tri") == 0) {
		type = eSignalTriangle;
	} else if (strcmp(waveType, "sweep") == 0) {
		type = eSignalSweep;   
	} else {
		fprintf(stderr, "Invalid signal type: %s\n", waveType);
		return -1;
	}

    /* Check frequency limits */
    if ( (frequency < c_min_frequency) || (frequency > c_max_frequency ) ) {
        fprintf(stderr, "Invalid frequency: %f\n", frequency);
        return -1;
    }

    awg_param_t params;

    /* Prepare data buffer (calculate from input arguments) */
    synthesize_signal(amplitude, frequency, type, endFrequency, data, &params);

    /* Write the data to the FPGA and set FPGA AWG state machine */
    write_data_fpga(0, data, &params);

	return 0;
}

/**
 * Synthesize a desired signal.
 *
 * Generates/synthesized  a signal, based on three pre-defined signal
 * types/shapes, signal amplitude & frequency. The data[] vector of 
 * samples at 125 MHz is generated to be re-played by the FPGA AWG module.
 *
 * @param ampl  Signal amplitude [Vpp].
 * @param freq  Signal frequency [Hz].
 * @param type  Signal type/shape [Sine, Square, Triangle].
 * @param data  Returned synthesized AWG data vector.
 * @param awg   Returned AWG parameters.
 *
 */
void synthesize_signal(double ampl, double freq, signal_e type, double endfreq, int32_t *data, awg_param_t *awg) {

    uint32_t i;

    /* Various locally used constants - HW specific parameters */
    const int dcoffs = -155;
    const int trans0 = 30;
    const int trans1 = 300; //samples at 1mhz?
    const double tt2 = 0.249;

    /* This is where frequency is used... */
    printf("shifted -155 up 16 places and you get %d\n", dcoffs << 16);
    awg->offsgain = (dcoffs << 16) + 0x1fff;  //dc voltage offset is (-155 << 16) + 0x1fff ?
    printf("offs gain %d\n", awg->offsgain);
    awg->step = round(65536 * freq/c_awg_smpl_freq * BUFFER_LEN);
    awg->wrap = round(65536 * BUFFER_LEN - 1);

    int trans = freq / 1e6 * trans1; /* 300 samples at 1 MHz */
    uint32_t amp = ampl * 4000.0;    /* 1 Vpp ==> 4000 DAC counts */
    if (amp > 8191) {
        /* Truncate to max value if needed */
        amp = 8191;
    }

    if (trans <= 10) {
        trans = trans0;
    }

    /* Fill data[] with appropriate buffer samples */
    for(i = 0; i < BUFFER_LEN; i++) {
        
        /* Sine */
        if (type == eSignalSine) {
            data[i] = round(amp * cos(2*M_PI*(double)i/(double)BUFFER_LEN));
            //this appears to ramp from -4000 to 4000 with 1.0vpp
 //           printf("data[%d] = %f\n", i, (float)data[i]);
        }
 
        /* Square */
        if (type == eSignalSquare) {
            data[i] = round(amp * cos(2*M_PI*(double)i/(double)BUFFER_LEN));
            if (data[i] > 0)
                data[i] = amp;
            else 
                data[i] = -amp;

            /* Soft linear transitions */
            double mm, qq, xx, xm;
            double x1, x2, y1, y2;    

            xx = i;       
            xm = BUFFER_LEN;
            mm = -2.0*(double)amp/(double)trans; 
            qq = (double)amp * (2 + xm/(2.0*(double)trans));
            
            x1 = xm * tt2;
            x2 = xm * tt2 + (double)trans;
            
            if ( (xx > x1) && (xx <= x2) ) {  
                
                y1 = (double)amp;
                y2 = -(double)amp;
                
                mm = (y2 - y1) / (x2 - x1);
                qq = y1 - mm * x1;

                data[i] = round(mm * xx + qq); 
            }
            
            x1 = xm * 0.75;
            x2 = xm * 0.75 + trans;
            
            if ( (xx > x1) && (xx <= x2)) {  
                    
                y1 = -(double)amp;
                y2 = (double)amp;
                
                mm = (y2 - y1) / (x2 - x1);
                qq = y1 - mm * x1;
                
                data[i] = round(mm * xx + qq); 
            }
        }
        
        /* Triangle */
        if (type == eSignalTriangle) {
            data[i] = round(-1.0*(double)amp*(acos(cos(2*M_PI*(double)i/(double)BUFFER_LEN))/M_PI*2-1));
        }

        /* Sweep */
        /* Loops from i = 0 to n = 16*1024. Generates a sine wave signal that
           changes in frequency as the buffer is filled. */
        double start = 2 * M_PI * freq;
        double end = 2 * M_PI * endfreq;
        if (type == eSignalSweep) {
            double sampFreq = c_awg_smpl_freq; // 125 MHz
            double t = i / sampFreq; // This particular sample
            double T = BUFFER_LEN / sampFreq; // Wave period = # samples / sample frequency
            /* Actual formula. Frequency changes from start to end. */
            data[i] = round(amp * (sin((start*T)/log(end/start) * ((exp(t*log(end/start)/T)-1)))));
        }
        
         // this is setting bit 15 as negative, for what reason?  Some kind of sign extension?
        /* TODO: Remove, not necessary in C/C++. */
        if(data[i] < 0)
            data[i] += (1 << 14);
    }
}

/**
 * Write synthesized data[] to FPGA buffer.
 *
 * @param ch    Channel number [0, 1].
 * @param data  AWG data to write to FPGA.
 * @param awg   AWG paramters to write to FPGA.
 */
void write_data_fpga(uint32_t ch, const int32_t *data, const awg_param_t *awg)
{
    uint32_t i;

    fpga_awg_init();

	/* Channel A */
	g_awg_reg->state_machine_conf = 0x000041;
	g_awg_reg->cha_scale_off      = awg->offsgain;
	g_awg_reg->cha_count_wrap     = awg->wrap;
	g_awg_reg->cha_count_step     = awg->step;
	g_awg_reg->cha_start_off      = 0;

	printf("cha state machine 0x%x cha_scale_off 0x%x cha_count_wrap 0x%x count_step 0x%x start_off 0x%x\n",
		g_awg_reg->state_machine_conf, g_awg_reg->cha_scale_off, g_awg_reg->cha_count_wrap, 
		g_awg_reg->cha_count_step, g_awg_reg->cha_start_off);

	for(i = 0; i < BUFFER_LEN; i++) {
		g_awg_cha_mem[i] = data[i];
	}

	//enable channel A and set enable of wrapping in memory...

    //g_awg_reg->state_machine_conf = 0x110011;
    g_awg_reg->state_machine_conf = 0x11;  //only channel A

//    fpga_awg_exit();
}
