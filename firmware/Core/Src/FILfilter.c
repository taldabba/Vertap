/*
 * FILfilter.c
 *
 *  Created on: Jul. 19, 2023
 *      Author: Taim
 */

#include "FIRfilter.h"

static const float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {-0.0032906f,-0.0052635f,-0.0068811f,0.0000000f,0.0254209f,0.0724719f,0.1311260f,0.1805961f,0.2000000f,0.1805961f,0.1311260f,0.0724719f,0.0254209f,0.0000000f,-0.0068811f,-0.0052635f};

void FIR_Filter_Init(FIRfilter* fir) {
	// clear filter buffer
	for (uint8_t i=0; i<FIR_FILTER_LENGTH; i++) {
		fir->buf[i]=0.0f;
	}

	// reset index
	fir->bufIndex = 0;

	// clear input
	fir->output = 0.0f;
}
float FIR_Filter_Update(FIRfilter* fir, float input) {
	fir->buf[fir->bufIndex] = input;

	fir->bufIndex++;
	if (fir->bufIndex == FIR_FILTER_LENGTH) fir->bufIndex=0;

	fir->output = 0.0f;
	uint8_t sumIndex = fir->bufIndex;

	for (uint8_t i=0; i<FIR_FILTER_LENGTH;i++){
		if(sumIndex>0) sumIndex--;
		else sumIndex=FIR_FILTER_LENGTH-1;

		fir->output += FIR_IMPULSE_RESPONSE[i]*fir->buf[sumIndex];
	}

	return fir->output;
}
