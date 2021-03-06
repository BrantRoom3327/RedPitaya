#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>

#include "redpitaya/rp.h"

#define WRITE_OUT_TEXT
#define WAIT_TIME_BEFORE_SAMPLING 1000
#define WAIT_TIME_AFTER_TRIGGER 0

//can we pull samples using ddrdump.bit and slicing off all but the first 16 or so?

static uint32_t buff_size = 16384;

int main(int argc, char **argv){

    if(rp_Init() != RP_OK){
        fprintf(stderr, "Rp api init failed!\n");
        return -1;
    }

    if (argc != 5) {
        fprintf(stderr, "args: <frequence in hz> <frequency increment> <num steps> <filename>\n");
        return -1;
    }

    int frequency = atoi(argv[1]);
    int increment = atoi(argv[2]);
    int steps = atoi(argv[3]);

    printf("Frequency %d increment %d steps %d\n", frequency, increment, steps);
    if (steps <= 0) {
        printf("Steps must be positive\n");
        return -1;
    }
    FILE *fp = NULL; 
#ifdef WRITE_OUT_TEXT
    fp = fopen(argv[4], "w");
    char *text = malloc(64);
#else
    fp = fopen(argv[4], "wb");
#endif
    if (fp == NULL)
    {
        fprintf(stderr, "Invalid filename for writing\n");
        return -1;
    }	
    fseek(fp, 0, SEEK_SET);
    float *buff = (float *)malloc(buff_size * sizeof(float));

    // unchanging things
    rp_GenWaveform(RP_CH_1, RP_WAVEFORM_SINE);
    rp_GenOutEnable(RP_CH_1);

    rp_AcqReset();
    rp_AcqSetSamplingRate(RP_SMP_125M);
    rp_AcqSetDecimation(RP_DEC_1);  //125m samples
    rp_AcqSetTriggerDelay(0);

    for (int i=0; i<steps; ++i) {
        if (i % 2 == 0)
	    rp_GenAmp(RP_CH_1, 1.0);
        else
	    rp_GenAmp(RP_CH_1, 0.5);

	rp_GenFreq(RP_CH_1, frequency);
        frequency += increment;

        rp_AcqStart();

	usleep(WAIT_TIME_BEFORE_SAMPLING);
        rp_AcqSetTriggerSrc(RP_TRIG_SRC_NOW);
        rp_acq_trig_state_t state = 0;

        while(1)
        {
            rp_AcqGetTriggerState(&state);
            if(state == RP_TRIG_STATE_TRIGGERED)
            {
                //usleep(WAIT_TIME_AFTER_TRIGGER);
                break;
            }
        }

        rp_AcqGetOldestDataV(RP_CH_1, &buff_size, buff);
        rp_AcqStop();
     // rp_GenOutDisable(RP_CH_1);  //do we need to disable and reenable outputs?  it appears no
#ifdef WRITE_OUT_TEXT
        for(int i = 0; i < buff_size; i++) {
	    sprintf(text, "%d %f\n", i, buff[i]); //writing a full 32 bit float, would be nice to convert to 16 bit float
            fwrite(text, 1, strlen(text), fp);
        }
#else
	fwrite(buff, 1, buff_size, fp);
#endif
    }

    if (buff)
        free(buff);

    if (fp)
        fclose(fp);

    rp_Release();
}
