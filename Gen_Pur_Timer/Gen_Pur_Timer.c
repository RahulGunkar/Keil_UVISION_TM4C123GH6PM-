#include "tm4c123gh6pm.h"
#include "SysTick.h"
#include "PLL.h"
#include "UART.h"

void Gen_Pur_Timer_Init()
{
	SYSCTL_RCGCTIMER_R |= (1<<0);
	TIMER0_CTL_R &= ~(1<<0);
	TIMER0_CFG_R = 0x00000000;
	TIMER0_TAMR_R |= 0x1; //Bits set: 1(periodic mode),4(count-up),(edge count mode)
	//TIMER0_TAMR_R &= ~(0x);//6,7
	TIMER0_CTL_R |= (1<<0);
	TIMER0_TAILR_R=0x4C4B400;//80 MHz
	
}

int main(void)
{
	PLL_Init();
	SysTick_Init();
	UART4_Init();
	UART4_OutChar('G');
	Gen_Pur_Timer_Init();
	
	while(1)
	{
		/*UART_OutUDec(TIMER0_TAV_R,4);
		SysTick_Wait10ms(20);
		TIMER0_TAILR_R=(TIMER0_TAILR_R)/10;
*/
		if(((TIMER0_RIS_R) &(0x00000001))==1)
		{
			TIMER0_ICR_R &= ~(1<<0);
			UART_OutUDec(10,4);
		
		}

		New_Line(4);
		/*if(TIMER0_TAV_R==2000)
		{
			UART4_OutChar('A');
		New_Line(4);
		}	*/
	}
}