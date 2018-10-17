
#ifndef __ADC_H
#define __ADC_H

#include "stm32f10x.h"

#define ADC1_DR_Address    ((u32)0x4001244C)

//����12λ��ADC��3.3V��ADCֵΪ0xfff,�¶�Ϊ25��ʱ��Ӧ�ĵ�ѹֵΪ1.43V��0x6EE
#define V25  0x6EE	 

//б�� ÿ���϶�4.3mV ��Ӧÿ���϶�0x05
#define AVG_SLOPE 0x05 

void Temp_ADC1_Init(void);

 

void ADC_Configuration(void);

#endif
