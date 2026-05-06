#include "headfile.h"
#include "ADC.h"
#include "stm32f4xx_adc.h"


float MCU_TEPERATE = 0.0;
unsigned short average = 0;
unsigned short ADC_Value[10];
float mcu_current_temperate=0; 
/******************************************************************************
* 函数名称       : DMA_Configuraiton
* 描述           : DMA 模块设置
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void DMA_Configuration (void)
{
  DMA_InitTypeDef DMA_InitStructure;
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
  DMA_DeInit(DMA1_Channel1);//将DMA的通道1寄存器重设为缺省值
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)( &(ADC1->DR));//DMA外设ADC基地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (unsigned long)&ADC_Value;//DMA内存基地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;//内存作为数据传输的目的地
  DMA_InitStructure.DMA_BufferSize = 2;//DMA通道的DMA缓存的大小
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不变
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址寄存器递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord; //数据宽度为16位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//数据宽度为16位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//工作在循环缓存模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//DMA通道 x拥有高优先级
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;//DMA通道x没有设置为内存到内存传输
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据DMA_InitStruct中指定的参数初始化DMA的通道
  DMA_ITConfig(DMA1_Channel1,DMA1_IT_TC1,ENABLE);//允许DMA通道1传输结束中断
  DMA_Cmd(DMA1_Channel1, ENABLE);
}

/******************************************************************************
* 函数名称       : ADC_GPIO_Init
* 描述           : ADC GPIO口设置
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void ADC_GPIO_Init (void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  //GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/******************************************************************************
* 函数名称       : ADC_Configuraiton
* 描述           : ADC 模块设置
* 输入           : 无
* 输出           : 无
* 返回           : 无
*******************************************************************************/
void ADC_Configuration (void)
{
  ADC_InitTypeDef ADC_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);
  RCC_ADCCLKConfig(RCC_PCLK2_Div6);//ADC最大时钟频率为14M
  ADC_DeInit(ADC1); //将外设 ADC1 的全部寄存器重设为缺省值
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC工作模式:ADC1和ADC2工作在独立模式
  ADC_InitStructure.ADC_ScanConvMode = ENABLE; //模数转换工作在扫描模式
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;//模数转换工作在连续转换模式
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;  //外部触发由软件触发
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
  ADC_InitStructure.ADC_NbrOfChannel = 2;//顺序进行规则转换的ADC通道的数目
  ADC_Init(ADC1, &ADC_InitStructure);//根据ADC_InitStruct中指定的参数初始化外设ADCx的寄存器
  ADC_TempSensorVrefintCmd(ENABLE);//开启内部温度传感器
  /***设置指定ADC的规则组通道，设置它们的转化顺序和采样时间;ADC1,ADC通道x,规则采样顺序值为y,采样时间为239.5周期  */
  ADC_RegularChannelConfig(ADC1, ADC_Channel_16, 1, ADC_SampleTime_239Cycles5);
  ADC_RegularChannelConfig(ADC1, ADC_Channel_4, 2, ADC_SampleTime_239Cycles5);
  
  ADC_DMACmd(ADC1, ENABLE);// 开启ADC的DMA支持（要实现DMA功能，还需独立配置DMA通道等参数）  
  ADC_Cmd(ADC1, ENABLE);//使能指定的ADC1
  ADC_ResetCalibration(ADC1);//复位指定的ADC1的校准寄存器
  while(ADC_GetResetCalibrationStatus(ADC1)); //获取ADC1复位校准寄存器的状态,设置状态则等待
  ADC_StartCalibration(ADC1); //开始指定ADC1的校准状态
  while(ADC_GetCalibrationStatus(ADC1));//获取指定ADC1的校准程序,设置状态则等待
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  ADC_ITConfig(ADC1,ADC_IT_EOC,ENABLE);//使能ADC中断
  ADC_ExternalTrigConvCmd(ADC1, ENABLE); //使能ADC经外部触发启动转换功能
}


/*******************************************************************************
* 函数名称: calculate_MCU_temperate
* 函数功能: 计算当前单片机内部的温度
* 输入参数: Value                   -- 内部温度传感器AD测量值
* 输出参数: mcu_current_temperate   -- MCU当前温度测量值
* 函数说明：无
*******************************************************************************/
float adc4_temp = 0.0;
float calculate_MCU_temperate (float Value)
{ 
  static const float mcu_25_temperate = 1.43;// MCU在25度的典型值 为1.43；
  static const float avg_slope = 0.0043;//单片机温度与电压值曲线的平均斜率  4.3mV/摄氏度
  adc4_temp = 3.3*Value/4096.0f;
  mcu_current_temperate = (((mcu_25_temperate-adc4_temp)/avg_slope) + 25.0);
  return (mcu_current_temperate); 
}

//以下为中断处理函数，定时器2提供500ns的定时，500ns内部AD取40个mcu温度传感器上的值。
/*******************************************************************************
* 函数名称: DMAChannel1_IRQHandler
* 函数功能: DMA中断处理函数
* 输入参数: 无
* 输出参数: 无
* 函数说明：无
*******************************************************************************/
void DMA1_Channel1_IRQHandler(void)
{    
  //unsigned char i;
  //unsigned short sum;
  if(DMA_GetITStatus(DMA1_IT_TC1))  //在DMA通道1传输完成中断中进行检测
  {
    //for(i=0;i<1;i++)
    //{
    //    sum = ADC_Value[i];
    //}
  }
  //average = sum / 1;
  DMA_ClearITPendingBit(DMA1_IT_GL1);//清DMA通道1的
}


void Chip_ADC_Init()
{
  NVIC_InitTypeDef NVIC_InitStructure;
  ADC_GPIO_Init();
  ADC_Configuration();
  DMA_Configuration();
  /* Configure one bit for preemption priority */ 
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;         
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;   
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;         
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;            
  NVIC_Init(&NVIC_InitStructure);
}

