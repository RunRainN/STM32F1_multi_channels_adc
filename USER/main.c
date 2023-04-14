#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "sample.h"
#include "led.h"

int main(void)
{

	delay_init();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	uart_init(115200);
	ADC1_Init(3, 500);
	LED_Init();			     //LED¶Ë¿Ú³õÊ¼»¯
	LED0 = 0;
	//LED1 = 0;
	while(1)
	{
		
	}
}
