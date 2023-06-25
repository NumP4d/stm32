/*
 * ring_buffer.c
 *
 *  Created on: 8 Sep 2019
 *      Author: klukomski
 */

#include "ring_buffer.h"

#include <string.h>

static void ring_buffer_inc(ring_buffer_t* self, void* indicator_p);

void ring_buffer_init(ring_buffer_t* self, void* memory_p)
{
	self->memory_p 	= memory_p;
	self->write_p 	= memory_p;
	self->read_p  	= self->write_p + RING_BUFFER_ACCESS_WIDTH;
}

void ring_buffer_push(ring_buffer_t* self, void* data_p)
{
	memcpy(self->write_p, data_p, RING_BUFFER_ACCESS_WIDTH);
	ring_buffer_inc(self, self->write_p);
	ring_buffer_inc(self, self->read_p);
}

void ring_buffer_push_convert(ring_buffer_t* self, int16_t* data_p)
{
	for (int i = 0; i < (RING_BUFFER_ACCESS_WIDTH / sizeof(float)); i++)
	{
		((float*)self->write_p)[i] = (float) (data_p[i]);
	}
	ring_buffer_inc(self, self->write_p);
	ring_buffer_inc(self, self->read_p);
}

void ring_buffer_cpy_to(ring_buffer_t* self, void* dest_p)
{
	void* bfr_p = self->read_p;
	for (int i = 0; i < RING_BUFFER_BLOCKS; i++)
	{
		memcpy(dest_p + i * RING_BUFFER_ACCESS_WIDTH, bfr_p, RING_BUFFER_ACCESS_WIDTH);
		ring_buffer_inc(self, bfr_p);
	}
}

static void ring_buffer_inc(ring_buffer_t* self, void* indicator_p)
{
	if (indicator_p == BUFFER_END(self))
	{
		indicator_p = self->memory_p;
	}
	else
	{
		indicator_p += RING_BUFFER_ACCESS_WIDTH;
	}
}
