/*
 * FIRfilter.h
 *
 *  Created on: Jul. 19, 2023
 *      Author: Taim
 */

#ifndef INC_FIRFILTER_H_
#define INC_FIRFILTER_H_

#include <stdint.h>

#define FIR_FILTER_LENGTH 16

typedef struct FIR_FILTER {
	float buf[FIR_FILTER_LENGTH];
	uint8_t bufIndex;

	float output;
} FIRfilter;

void FIR_Filter_Init(FIRfilter* fir);
float FIR_Filter_Update(FIRfilter* fir, float input);

#endif /* INC_FIRFILTER_H_ */
