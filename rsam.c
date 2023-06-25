/*
 * rsam.c
 *
 *  Created on: 8 Sep 2019
 *      Author: klukomski
 */

#include "rsam.h"
#include "main.h"


#define FFT_FORWARD		0


volatile rsam_algorithm_t	hrsam;

void rsam_push_window(rsam_algorithm_t* self, int16_t* audiodata_p);
void rsam_push_window_rbfr(rsam_algorithm_t* self, int16_t* audiodata_p);

void rsam_calculate(rsam_algorithm_t* self);

void rsam_init(rsam_algorithm_t* self, DFSDM_Filter_HandleTypeDef* hdfsdm_filter, float overmag, float a, float b)
{
	self->hdfsdm_filter = hdfsdm_filter;
	self->overmag = overmag;
	self->a	= a;
	self->b	= b;
	arm_rfft_fast_init_f32(&self->hrfft, RSAM_FFT_LENGTH);
	//ring_buffer_init(&self->hrbfr, self->workdata.audio);
}

void rsam_start(rsam_algorithm_t* self)
{
	HAL_DFSDM_FilterRegularStart_DMA(self->hdfsdm_filter, (int32_t*)self->workdata.rawaudio, (RSAM_WINDOW_INC * 2));
}

void rsam_stop(rsam_algorithm_t* self)
{
	HAL_DFSDM_FilterRegularStop_DMA(self->hdfsdm_filter);
}

void rsam_push_window(rsam_algorithm_t* self, int16_t* audiodata_p)
{
	volatile int i;
	/* Shift data to left by RSAM_WINDOW_INC parameter */
	for (i = 0; i < (RSAM_FFT_LENGTH - RSAM_WINDOW_INC); i++)
	{
		self->workdata.audio[i] = self->workdata.audio[i + RSAM_WINDOW_INC];
	}
	/* Put new data to array with conversion to float */
	for (i = (RSAM_FFT_LENGTH - RSAM_WINDOW_INC); i < RSAM_FFT_LENGTH; i++)
	{
		/* This data contains offset at 0 frequency from regular
		 * conversion channel but this has no effetct on algorithm */
		self->workdata.audio[i] = (float) (audiodata_p[i - (RSAM_FFT_LENGTH - RSAM_WINDOW_INC)]);
	}
	/* Copy ready buffer to FFT workdata buffer */
	memcpy(self->workdata.audio_fft, self->workdata.audio, RSAM_FFT_LENGTH * sizeof(self->workdata.audio_fft[0]));
}

void rsam_push_window_rbfr(rsam_algorithm_t* self, int16_t* audiodata_p)
{
	ring_buffer_push_convert(&self->hrbfr, audiodata_p);
	ring_buffer_cpy_to(&self->hrbfr, self->workdata.audio_fft);
}

void rsam_calculate(rsam_algorithm_t* self)
{
	int i;
	self->workdata.peak_idx = -0xFFFF;
	/* Calculate FFT */
	arm_rfft_fast_f32(&self->hrfft, self->workdata.audio_fft, self->workdata.fft, FFT_FORWARD);
	/* Calculate magnitude squared from 500 Hz to 12 kHz */
	/* This corresponds to bins 43 (couting from 0) to 1024 */
	arm_cmplx_mag_squared_f32(self->workdata.fft + 2*43, self->workdata.mag, (1024 - 43));
	/* Calculate maximum magnitude in this frequency interval */
	/* POWINNO BYĆ W ROZMIARZE + 1 !!!!!!!! ???? ???? */
	arm_max_f32(self->workdata.mag, (1024 - 43), &self->workdata.maxmag, &self->workdata.maxmag_idx);
	/* Calculate minimum considered magnitude */
	self->workdata.overmag_limit = self->overmag * self->workdata.maxmag;
	/* Find first peak over considered limit from this frequency interval */
	for (i = 0; i < (1024 - 43); i++)
	{
		if (self->workdata.mag[i] >= self->workdata.overmag_limit)
		{
			self->workdata.peak_idx = i;
			break;
		}
	}
	/* Calculate resulting rotating speed based on calibration data */
	self->result = self->a * self->workdata.peak_idx + self->b;
}

void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
	volatile uint32_t calculationTime;
	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	calculationTime = HAL_GetTick();
	//tick = SysTick->VAL;
	HAL_GPIO_WritePin(STMOD_SCOPE10_GPIO_Port, STMOD_SCOPE10_Pin, GPIO_PIN_SET);
	/* First half of buffer */
	rsam_push_window((rsam_algorithm_t*)&hrsam, (int16_t*)hrsam.workdata.rawaudio);
	rsam_calculate((rsam_algorithm_t*)&hrsam);
	rsam_callback((rsam_algorithm_t*)&hrsam);
	HAL_GPIO_WritePin(STMOD_SCOPE10_GPIO_Port, STMOD_SCOPE10_Pin, GPIO_PIN_RESET);
	//tick = SysTick->VAL - tick;
	calculationTime = HAL_GetTick() - calculationTime;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef* hdfsdm_filter)
{
	//HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STMOD_SCOPE10_GPIO_Port, STMOD_SCOPE10_Pin, GPIO_PIN_SET);
	/* Second half of buffer */
	rsam_push_window((rsam_algorithm_t*)&hrsam, (int16_t*)(hrsam.workdata.rawaudio + RSAM_WINDOW_INC));
	rsam_calculate((rsam_algorithm_t*)&hrsam);
	rsam_callback((rsam_algorithm_t*)&hrsam);
	HAL_GPIO_WritePin(STMOD_SCOPE10_GPIO_Port, STMOD_SCOPE10_Pin, GPIO_PIN_RESET);
}
