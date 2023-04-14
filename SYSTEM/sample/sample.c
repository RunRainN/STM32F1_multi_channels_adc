#include "sys.h"
#include "sample.h"

u16 AD_Value[16];//DMA������
u8 gchannels;

//�������̣�TIM���->TRGO�ź�->ADC������ͨ�����һ�β���->������ֵд��DMA������->д����ϲ���DMA�ж�->ִ��DMA�жϺ���������������ͨ�����ڷ���

//��ڲ������ɼ�ͨ����������Ƶ��
void ADC1_Init(u8 channels, u16 sampfreq)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	u8 i;
	
	gchannels = channels;
	
	/*-------------------------------------ADC����-------------------------------------*/
	//1��ADC������Ϊ���ⲿ��ADC�ⲿ���źŴ�����
	//2��ADC������Ϊ����������ģʽ��ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;��
	//	��ν����������ģʽ��������ǰ���ᵽ��ͨ������ÿ��A/Dת��ʱ��ʵ�ֲ�����ʱ�Ĺ�����ʽ��
	//	�����ģʽ��ʹ�ܣ�����ζ��ADC���ڱ�һ�δ��������ͨ������ͣЪ�Ľ���������ת����������ȵ��´ζ�ʱ�������ź�������A/Dת����
	//3��ADC������Ϊ��ͨ��ɨ��ģʽ��ADC_InitStructure.ADC_ScanConvMode = ENABLE;��������ADC����ÿ�α�TIM2������������ɹ���ͨ������ÿ��ͨ����һ��ת����
	
	GPIO_InitStructure.GPIO_Pin = (0x0001<<channels)-1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;			//ģ������
	GPIO_Init(GPIOA, &GPIO_InitStructure);					//��ʼ������
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_ADCCLKConfig(RCC_PCLK2_Div6);   //����ADC��Ƶ����6 72M/=12M,ADC���ʱ�䲻�ܳ���14M
	
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;//ADC1�����ڶ���ģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ģ��ת��������ɨ��ģʽ����ͨ����
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//ADC�����ڷ�����ģʽ
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T3_TRGO;//��ʱ��3��TRGO�źŴ���
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//ADC�����Ҷ���
	ADC_InitStructure.ADC_NbrOfChannel = channels;//ת����ADCͨ����
	ADC_Init(ADC1, &ADC_InitStructure);//�������ϲ���������ʼ��ADC_InitStructure
	
	for(i=0;i<channels;i++)
	{
		ADC_RegularChannelConfig(ADC1, i, i+1, ADC_SampleTime_239Cycles5);//ADC1ͨ�� ������ʱ��Ϊ239.5������
	}
	
	/*-------------------------------------DMA����-------------------------------------*/
	//1��DMA������Ϊ���赽�ڴ淽ʽ��ÿ�δ�������ݿ���ǰ��֣�16λ����
	//2�����峤��Ϊchannels�����֣�DMA_InitStructure.DMA_BufferSize = channels;�������һ�β����Ľ����
	//3��ʹ��ѭ��ģʽ��DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;����ʹ����DMA1�жϣ�DMA_Init(DMA1_Channel1, &DMA_InitStructure);��
	//	ADCÿ��ת����ɺ󶼻Ὣ���д��DMA��������������д����Ϻ�ᴥ��DMA�жϡ�
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);
	
  DMA_InitStructure.DMA_PeripheralBaseAddr = (u32)&(ADC1->DR);//�����Դͷ��ַ
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)AD_Value;//Ŀ���ַ
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC; //������Դͷ
  DMA_InitStructure.DMA_BufferSize = channels;//���ݳ���Ϊͨ����
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;//�����ַ�Ĵ���������
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;//�ڴ��ַ����
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//���贫���԰���Ϊ��λ
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;//�ڴ��԰���Ϊ��λ
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;//ѭ��ģʽ
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;//4���ȼ�֮һ��(������)
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; //���ڴ浽�ڴ�
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);//�������ϲ�����ʼ��DMA_InitStructure
	
	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);//����DMA1ͨ��1��������ж�
	
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;//��ռ���ȼ�����Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;//�����ȼ�����Ϊ1
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//�ж�ʹ��
  NVIC_Init(&NVIC_InitStructure);//��ָ��������ʼ���ж�
	
	/*-------------------------------------TIM����-------------------------------------*/
	//1�����ڼĴ�����Ԥ��Ƶ�Ĵ������ú󣬽�TIM����ΪPWMģʽ��
	//2��TIM2��������TRGO�źţ�TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);����
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	
	TIM_TimeBaseStructure.TIM_Prescaler = 1000000/sampfreq-1;			//Ԥ��Ƶϵ��
	TIM_TimeBaseStructure.TIM_Period = 72-1;					//����
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;      //������ƵTIM_CKD_DIV1
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;     //���ϼ���
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
	
	TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);//�������TRGO�ź�
	
	ADC_DMACmd(ADC1,ENABLE);					//����ADC��DMA�Ĺ���
	ADC_Cmd(ADC1, ENABLE);						//����ADC1
	ADC_ResetCalibration(ADC1);					//����ADCУ׼
	while(ADC_GetResetCalibrationStatus(ADC1));	//�ȴ��������
	ADC_StartCalibration(ADC1);	 				//��ʼУ׼
	while(ADC_GetCalibrationStatus(ADC1));	 	//�ȴ�У׼���			
	DMA_Cmd(DMA1_Channel1,ENABLE);				//����DMA
	ADC_ExternalTrigConvCmd(ADC1,ENABLE);		//�����ⲿ����
	TIM_Cmd(TIM3, ENABLE);						//����TIM3
}

//DMA�жϷ�������DMA������������ͨ�����ڷ��ͣ����͸�ʽ��֡ͷ��0D 0A��ͨ��1��2�ֽڣ�ͨ��2��2�ֽڣ�ͨ��3��2�ֽڣ�...
void DMA1_Channel1_IRQHandler(void)
{
	u16 i;
	if(DMA_GetITStatus(DMA1_IT_TC1))//�ж�ͨ��1�Ƿ������
	{
		DMA_ClearITPendingBit(DMA1_IT_TC1);    //���ͨ��1������ɱ�־λ
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		USART_SendData(USART1, 0x0D);         //�򴮿ڷ�������
		while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
		USART_SendData(USART1, 0x0A);         //�򴮿ڷ�������
		for(i=0;i<gchannels;i++)
		{
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART1, AD_Value[i]>>8);         //�򴮿ڷ�������
			while(USART_GetFlagStatus(USART1,USART_FLAG_TC)!=SET);//�ȴ����ͽ���
			USART_SendData(USART1, AD_Value[i]&0x00FF);         //�򴮿ڷ�������
		}
	}
}
