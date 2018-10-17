/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��ADC.c
 * ����    �����ADC�ĳ�ʼ��   
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-10-20
 * Ӳ������: 
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/
#include "ADC.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_adc.h"

float AD_value;
__IO vu16 ADC_ConvertedValue;

 /**
  * @file   ADC_Configuration
  * @brief  ���ADC��DMA�ĳ�ʼ��
  * @param  ��
  * @retval ��
  */
void ADC_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* DMA channel1 configuration */
		   
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;		        // �������ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	        // ADת��ֵ����ŵ��ڴ����ַ	�����Ǹ�����ַ��
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                        // ������Ϊ���ݴ������Դ	
  DMA_InitStructure.DMA_BufferSize = 1;                                     // ����ָ��DMAͨ�� DMA����Ĵ�С
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          // �����ַ�Ĵ�������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;			        // �ڴ��ַ�Ĵ�������
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // ���ݿ��Ϊ16λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // HalfWord
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                    //������ѭ��ģʽ��
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                    //�����ȼ�
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		                        //û������Ϊ�ڴ浽�ڴ�Ĵ���
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	 /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);							
   
   /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                        //��������ģʽ
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;			                    //��ͨ��
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			            //����ת��
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;       //�������������
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;					//Right
  ADC_InitStructure.ADC_NbrOfChannel = 1;				                    //��һ��ͨ��ת��
  ADC_Init(ADC1, &ADC_InitStructure);
	
  	/*����ADCʱ�ӣ�ΪPCLK2��8��Ƶ����9Hz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

  /* ADC1 regular channel16 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);  //���ò���ͨ��IN16, ���ò���ʱ��	

  //ʹ���¶ȴ��������ڲ��ο���ѹ   
  ADC_TempSensorVrefintCmd(ENABLE);                                    

   /* Enable ADC1 DMA */	  
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);			                              
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));                         

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);				                        
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));	  
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
}


