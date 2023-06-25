/*
 * rsam.h
 *
 *  Created on: 8 Sep 2019
 *      Author: klukomski
 */

#ifndef RSAM_H_
#define RSAM_H_

#include "main.h"
#include "arm_math.h"
#include "ring_buffer.h"

#define RSAM_AUDIO_FREQ		48000
#define RSAM_FFT_LENGTH		4096
//#define RSAM_WINDOW_INC		256
#define RSAM_WINDOW_INC		512
//#define RSAM_WINDOW_INC		4096
//#define RSAM_WINDOW_INC		480

/* Typical RSAM parameters */
//#define RSAM_OVERMAG	0.125
#define RSAM_OVERMAG	0.25
#define RSAM_A			0.9692
#define RSAM_B			43.1464

typedef struct {
	int16_t						rawaudio[RSAM_WINDOW_INC * 2];
	float						audio[RSAM_FFT_LENGTH];
	float						audio_fft[RSAM_FFT_LENGTH];
	float						fft[RSAM_FFT_LENGTH * 2];
	float 						mag[1024 - 43];
	float 						maxmag;
	uint32_t					maxmag_idx;
	uint32_t					peak_idx;
	float						overmag_limit;
} rsam_workdata_t;

/* Rotation Speed Audio Measurement Algorithm */
typedef struct {
	arm_rfft_fast_instance_f32	hrfft;
	ring_buffer_t				hrbfr;
	float						overmag;
	float						a;
	float						b;
	float						result;
	DFSDM_Filter_HandleTypeDef* hdfsdm_filter;
	rsam_workdata_t				workdata;
} rsam_algorithm_t;

extern volatile rsam_algorithm_t	hrsam;

void rsam_init(rsam_algorithm_t* self, DFSDM_Filter_HandleTypeDef* hdfsdm_filter, float overmag, float a, float b);

void rsam_start(rsam_algorithm_t* self);

void rsam_stop(rsam_algorithm_t* self);

void rsam_callback(rsam_algorithm_t* self) __attribute__((weak));

#endif /* RSAM_H_ */
