/*
 * adc.h
 *
 *  Created on: May 16, 2017
 *      Author: chuankai
 */

#ifndef ADC_H_
#define ADC_H_

void ADC_ConfigChannel();
void ADC_Start();
void ADC_PollForConversion();
uint32_t ADC_GetValue();

void ADC_ConfigChannel()
{
}

void ADC_Start()
{
}

void ADC_PollForConversion()
{
}

uint32_t ADC_GetValue()
{
	static uint32_t mV = 0;
	mV += 100;
    return mV;
}

#endif /* ADC_H_ */
