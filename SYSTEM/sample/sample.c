#include "sys.h"
#include "sample.h"

u16 AD_Value[16];//DMA缓冲区
u8 gchannels;

//程序流程：TIM溢出->TRGO信号->ADC对所有通道完成一次采样->将采样值写入DMA缓冲区->写入完毕产生DMA中断->执行DMA中断函数将缓冲区数据通过串口发送

//入口参数：采集通道数，采样频率
void ADC1_Init(u8 channels, u16 sampfreq)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	u8 i;
	
	gchannels = channels;
	
	/*-------------------------------------ADC配置-------------------------------------*/
	//1、ADC被配置为由外部（ADC外部）信号触发。
	//2、ADC被配置为非连续工作模式（ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;）
	//	所谓“连续工作模式”，就是前面提到了通过配置每次A/D转换时间实现采样定时的工作方式。
	//	如果该模式被使能，就意味着ADC会在被一次触发后就逐通道、不停歇的进行连续的转换，而不会等到下次定时器触发信号再启动A/D转换。
	//3、ADC被配置为多通道扫描模式（ADC_InitStructure.ADC_ScanConvMode = ENABLE;），这样ADC会在每次被TIM2触发后依次完成规则通道组中每个通道的一轮转换。
	
	GPIO_InitStructure.GPIO_Pin = (0x0001<<channels)-1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			//模拟输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//初始化引脚
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //设置ADC分频因子6 72M/=12M,ADC最大时间不能超过14M
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1工作在独立模式
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//模数转换工作在扫描模式（多通道）
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ADC工作在非连续模式
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;//定时器3的TRGO信号触发
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC数据右对齐
	ADC_InitStructure.ADC_NbrOfChannel = channels;//转换的ADC通道数
	ADC_Init(ADC1, &ADC_InitStructure);//根据以上参数参数初始化ADC_InitStructure
	
	for(i=0;i<channels;i++)
	{
		ADC_RegularChannelConfig(ADC1, i, i+1, ADC_SampleTime_239Cycles5);//ADC1通道 ，采样时间为239.5个周期
	}
	
	/*-------------------------------------DMA配置-------------------------------------*/
	//1、DMA被配置为外设到内存方式，每次传输的数据宽度是半字（16位）。
	//2、缓冲长度为channels个半字（DMA_InitStructure.DMA_BufferSize = channels;），存放一次采样的结果。
	//3、使用循环模式（DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;），使能了DMA1中断（DMA_Init(DMA1_Channel1, &DMA_InitStructure);）
	//	ADC每次转换完成后都会将结果写入DMA缓冲区，缓冲区写入完毕后会触发DMA中断。
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(ADC1->DR);//传输的源头地址
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;//目标地址
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //外设作源头
  DMA_InitStructure.DMA_BufferSize = channels;//数据长度为通道数
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//外设地址寄存器不递增
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//内存地址递增
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//外设传输以半字为单位
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//内存以半字为单位
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//循环模式
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//4优先级之一的(高优先)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //非内存到内存
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//根据以上参数初始化DMA_InitStructure
	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//配置DMA1通道1传输完成中断
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//抢占优先级设置为1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//子优先级设置为1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//中断使能
  NVIC_Init(&NVIC_InitStructure);//按指定参数初始化中断
	
	/*-------------------------------------TIM配置-------------------------------------*/
	//1、周期寄存器和预分频寄存器配置后，将TIM设置为PWM模式。
	//2、TIM2溢出后将输出TRGO信号（TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);）。
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 1000000/sampfreq-1;			//预分频系数
	TIM_TimeBaseStructure.TIM_Period = 72-1;					//周期
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //采样分频TIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //向上计数
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);//设置输出TRGO信号
	
	ADC_DMACmd(ADC1,ENABLE);					//开启ADC与DMA的关联
	ADC_Cmd(ADC1, ENABLE);						//开启ADC1
	ADC_ResetCalibration(ADC1);					//重置ADC校准
	while(ADC_GetResetCalibrationStatus(ADC1));	//等待重置完成
	ADC_StartCalibration(ADC1);	 				//开始校准
	while(ADC_GetCalibrationStatus(ADC1));	 	//等待校准完成			
	DMA_Cmd(DMA1_Channel1,ENABLE);				//开启DMA
	ADC_ExternalTrigConvCmd(ADC1,ENABLE);		//开启外部触发
	TIM_Cmd(TIM3, ENABLE);						//开启TIM3
}

//DMA中断服务函数将DMA缓冲区的数据通过串口发送，发送格式：帧头（0D 0A）通道1（2字节）通道2（2字节）通道3（2字节）...
void DMA1_Channel1_IRQHandler(void)
{
	u16 i;
	if(DMA_GetITStatus(DMA1_IT_TC1))//判断通道1是否传输完成
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);    //清除通道1传输完成标志位
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		USART_SendData(USART1, 0x0D);         //向串口发送数据
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
		USART_SendData(USART1, 0x0A);         //向串口发送数据
		for(i=0;i<gchannels;i++)
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART1, AD_Value[i]>>8);         //向串口发送数据
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//等待发送结束
			USART_SendData(USART1, AD_Value[i]&0x00FF);         //向串口发送数据
		}
	}
}
