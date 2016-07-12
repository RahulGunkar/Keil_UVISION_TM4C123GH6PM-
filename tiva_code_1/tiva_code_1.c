#include "tm4c123gh6pm.h"
#include "PLL.h"
#include  "SysTick.h"
#include "UART.h"
#include <stdio.h>



int main(void)
{
	PLL_Init();
	SysTick_Init();
	
	UART5_Init();//38,400=baud
	
	
	while(1)
	{
			
		SysTick_Wait10ms(10);
		UART5_OutChar('A');	
			
		
	}
}

