/*
 * mmf.h
 *
 *  Created on: 12 Sep 2019
 *      Author: klukomski
 */

#ifndef MMF_H_
#define MMF_H_

#include <stdint.h>
#include <stdio.h>

typedef float 	mmf_data_t;
typedef uint8_t	mmf_index_t;

typedef struct {
	mmf_data_t	data;
	mmf_index_t	index;
} mmf_data_element_t;

typedef struct {
	mmf_data_element_t*	elementArray;
	size_t				arrayLength;
	mmf_data_element_t*	oldestElement;
	mmf_index_t			oldestIndex;
	mmf_index_t			newestIndex;
	mmf_data_t*			median_p;
} mmf_t;

void 		mmf_init(mmf_t* self, mmf_data_element_t* elementArray, size_t arrayLength, mmf_data_t initializerElement);

mmf_data_t 	mmf_calculate(mmf_t* self, mmf_data_t newElement);

#endif /* MMF_H_ */
