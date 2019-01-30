/*
 * asg.c
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Brant Rosenberger
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>
#include <stdint.h>
#include <math.h>

#include "options.h"
#include "asg.h"

#define BUFFER_LENGTH 16384

void asg_start(struct asg_parameter *param);
void asg_cleanup(struct asg_parameter *param, option_fields_t *options);

void asg_dump_sample_memory(struct asg_parameter *param)
{
    for (int i=0x10000; i <= 0x1FFFC; i+=4)
        printf("sine 0x%x => 0x%x\n", i, *(uint32_t *)(param->mapped_io + i));
}

void asg_register_dump(struct asg_parameter *param)
{
    for (int i=0; i <= 0x20; i+=4) {
        printf("siggen 0x%x => 0x%x\n", i, *(uint32_t *)(param->mapped_io + i));
    }
}

void synthesize_sine(float *data_out)
{
    for(int unsigned i = 0; i < BUFFER_LENGTH; i++) {
        data_out[i] = (float) (sin(2 * M_PI * (float) i / (float) BUFFER_LENGTH));
    }
}

int asg_init(struct asg_parameter *param, option_fields_t *options)
{
    memset(param, 0, sizeof(*param));

//	param->fd = open("/dev/mem", O_RDWR);
	// assume we are using the already open param->fd from scope_init
	if (param->fd < 0) {
		fprintf(stderr, "open asg failed, %d\n", errno);
		return -1;
	}

    //asg memory area
	param->mapped_io = mmap(NULL, 0x00100000UL, PROT_WRITE | PROT_READ, MAP_SHARED, param->fd, 0x40200000UL);
	if (param->mapped_io == MAP_FAILED) {
		fprintf(stderr, "mmap scope io failed (non-fatal), %d\n", errno);
		param->mapped_io = NULL;
	}

	if (!param->mapped_io) {
        printf("Failed to memmap the ASG register set\n");
		goto out;
    }

	asg_start(param);

out:
	return 0;
}

void asg_cleanup(struct asg_parameter *param, option_fields_t *options)
{
    printf("asg_cleanup\n");
	if (param->mapped_io) {
		munmap(param->mapped_io, 0x00100000UL);
	}

	if (param->mapped_buf_a)
		munmap(param->mapped_buf_a, param->buf_a_size);

	close(param->fd);
}

//TODO: figure out more of the memory map and insert new values.
void asg_start(struct asg_parameter *param)
{
	float sine[BUFFER_LENGTH];
    synthesize_sine(sine);
#if 0
    for (int i=0; i < BUFFER_LENGTH; i++) {
       printf("sine[%d] = %f\n", i, sine[i]);
    }
#endif

     asg_register_dump(param);
	// for 0x04 the amplitude offset is set to 7, not sure why yet.

	///need the full value set here so we can control the triggering and all the other things.
    *(uint32_t *)(param->mapped_io + 0x00000) = 0x11;
    *(uint32_t *)(param->mapped_io + 0x00004) = 0xD1FFF;
    *(uint32_t *)(param->mapped_io + 0x00008) = 0x3fffffff; //wrap at max value
    *(uint32_t *)(param->mapped_io + 0x0000C) = 0;  //offset when trigger arrives
    *(uint32_t *)(param->mapped_io + 0x00010) = 0;
    *(uint32_t *)(param->mapped_io + 0x00014) = 0;
    *(uint32_t *)(param->mapped_io + 0x00018) = 0;
    *(uint32_t *)(param->mapped_io + 0x0001C) = 0;
    *(uint32_t *)(param->mapped_io + 0x00020) = 0;

     asg_register_dump(param);

	//generate a waveform and put it into memory, read it back to confirm it matches...
    for (int i=0x10000, j=0; i <= 0x1FFFC; i+=4, j++) {
        //*(uint32_t *)(param->mapped_io + i) = sine[j];
        *(uint32_t *)(param->mapped_io + i) = 1;
    }

    //read it back and diff it
	for (int i=0x10000, j=0; i <= 0x1FFFC; i+=4, j++) {
        if (*(float *)(param->mapped_io + i) != sine[j])
            printf("Failed to read back sine wave at index %d val %f\n", j, *(float *)(param->mapped_io + i));
    }

	asg_dump_sample_memory(param);
}
