/*
 * ring_buffer.h
 *
 *  Created on: 8 Sep 2019
 *      Author: klukomski
 */

#ifndef RING_BUFFER_H_
#define RING_BUFFER_H_

#include <stdint.h>

#define RSAM_AUDIO_FREQ		48000
#define RSAM_FFT_LENGTH		4096
//#define RSAM_WINDOW_INC		256
#define RSAM_WINDOW_INC		512
//#define RSAM_WINDOW_INC		4096

#define RING_BUFFER_ACCESS_WIDTH	RSAM_WINDOW_INC * sizeof(float)
#define RING_BUFFER_BLOCKS			RSAM_FFT_LENGTH / RSAM_WINDOW_INC

#define BUFFER_END(bfr)				((bfr)->memory_p + (RING_BUFFER_BLOCKS - 1) * RING_BUFFER_ACCESS_WIDTH)

typedef struct ring_buffer_struct
{
	void*	memory_p;
	void*	write_p;
	void*	read_p;
} ring_buffer_t;

void ring_buffer_init(ring_buffer_t* self, void* memory_p);

void ring_buffer_push(ring_buffer_t* self, void* data_p);

void ring_buffer_push_convert(ring_buffer_t* self, int16_t* data_p);

void ring_buffer_cpy_to(ring_buffer_t* self, void* data_p);

#endif /* RING_BUFFER_H_ */
