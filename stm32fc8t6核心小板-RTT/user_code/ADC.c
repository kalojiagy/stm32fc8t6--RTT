/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：ADC.c
 * 描述    ：完成ADC的初始化   
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2015-10-20
 * 硬件连接: 
 * 调试方式：J-Link-OB
**********************************************************************************/
#include "ADC.h"
//#include "stm32f10x_dma.h"
//#include "stm32f10x_adc.h"

float AD_value;
__IO vu16 ADC_ConvertedValue;

 /**
  * @file   ADC_Configuration
  * @brief  完成ADC和DMA的初始化
  * @param  无
  * @retval 无
  */
void ADC_Configuration(void)
{
  DMA_InitTypeDef DMA_InitStructure;
  ADC_InitTypeDef ADC_InitStructure;
	
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
  
  /* DMA channel1 configuration */
		   
  DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;		        // 外设基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (u32)&ADC_ConvertedValue;	        // AD转换值所存放的内存基地址	（就是给个地址）
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;                        // 外设作为数据传输的来源	
  DMA_InitStructure.DMA_BufferSize = 1;                                     // 定义指定DMA通道 DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;          // 外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Disable;			        // 内存地址寄存器不变
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; // 数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;         // HalfWord
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;		                    //工作在循环模式下
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;	                    //高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;		                        //没有设置为内存到内存的传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
	 /* Enable DMA channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);							
   
   /* ADC1 configuration */
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;                        //独立工作模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;			                    //多通道
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;			            //连续转换
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;       //由软件触发启动
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;					//Right
  ADC_InitStructure.ADC_NbrOfChannel = 1;				                    //仅一个通道转换
  ADC_Init(ADC1, &ADC_InitStructure);
	
  	/*配置ADC时钟，为PCLK2的8分频，即9Hz*/
	RCC_ADCCLKConfig(RCC_PCLK2_Div8); 

  /* ADC1 regular channel16 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);  //设置采样通道IN16, 设置采样时间	

  //使能温度传感器和内部参考电压   
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


