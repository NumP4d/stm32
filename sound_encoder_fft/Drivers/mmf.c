/*
 * mmf.c
 *
 *  Created on: 12 Sep 2019
 *      Author: klukomski
 */

#include "mmf.h"

#include <string.h>

void		mmf_swap(mmf_data_element_t* p1, mmf_data_element_t* p2);

void		mmf_bubble_sort(mmf_t* self, size_t elementIndex);

void 		mmf_init(mmf_t* self, mmf_data_element_t* elementArray, size_t arrayLength, mmf_data_t initializerElement)
{
	self->elementArray 	= elementArray;
	self->arrayLength 	= arrayLength;
	self->oldestElement	= 0;
	self->median_p		= &(self->elementArray[(self->arrayLength / 2)].data);
	for (size_t i = 0; i < self->arrayLength; i++)
	{
		self->elementArray[i].data 	= initializerElement;
		self->elementArray[i].index	= self->newestIndex++;
	}
}

mmf_data_t 	mmf_calculate(mmf_t* self, mmf_data_t newElement)
{
	/* Put new element */
	for (size_t i = 0; i < self->arrayLength; i++)
	{
		if (self->elementArray[i].index == self->oldestIndex)
		{
			self->elementArray[i].data 	= newElement;
			self->elementArray[i].index = self->newestIndex++;
			self->oldestIndex++;
			mmf_bubble_sort(self, i);
			break;
		}
	}

	/* Return middle element */
	return *(self->median_p);
}

void		mmf_swap(mmf_data_element_t* p1, mmf_data_element_t* p2)
{
	mmf_data_element_t temp;
	temp = *p1;
	*p1 = *p2;
	*p2 = temp;
}

void		mmf_bubble_sort(mmf_t* self, size_t elementIndex)
{
	if (elementIndex != 0)
	{
		/* Bubble sort to left */
		if (self->elementArray[elementIndex].data < self->elementArray[elementIndex - 1].data)
		{
			do
			{
				mmf_swap(&self->elementArray[elementIndex], &self->elementArray[elementIndex - 1]);
				elementIndex--;
			}
			while (	(self->elementArray[elementIndex].data < self->elementArray[elementIndex - 1].data)
					&& (elementIndex != 0) );
		}
	}
	if (elementIndex != (self->arrayLength - 1))
	{
		/* Bubble sort to right */
		if (self->elementArray[elementIndex].data > self->elementArray[elementIndex + 1].data)
		{
			do
			{
				mmf_swap(&self->elementArray[elementIndex], &self->elementArray[elementIndex + 1]);
				elementIndex++;
			}
			while (	(self->elementArray[elementIndex].data > self->elementArray[elementIndex + 1].data)
					&& (elementIndex != (self->arrayLength - 1)) );
		}
	}
}
