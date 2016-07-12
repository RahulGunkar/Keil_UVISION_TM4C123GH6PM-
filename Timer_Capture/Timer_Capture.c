#include <stdio.h>
#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "UART.h"
#include "SysTick.h"

long long FallingEdges=0;
void EnableInterrupts(void);
void WaitForInterrupt();

void Edge_Detect_Init(void){
	
	SYSCTL_RCGC2_R|=(1<<PORTB);  //Enable clock for PortA, PortB
	GPIO_PORTB_DIR_R&=~(1<<PB6);  //Set GPIO pins as Input
	GPIO_PORTB_AFSEL_R|=(1<<PB6); //Disable alt. function
	
	GPIO_PORTB_DEN_R|=(1<<PB6);  //Select Digital I/O operation
	
	GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xF0FFFFFF)|(0x07000000);   //Configure for GPIO operation
	
	GPIO_PORTB_AMSEL_R&=~(1<<PB6);  //Disable analog function
	


	
}
void Timer_Capture_Init(void){
	SYSCTL_RCGCTIMER_R |= (1<<0);
	TIMER0_CTL_R &= ~(1<<0);	//clear timer enable
	TIMER0_CFG_R = 0x00000004;
	TIMER0_TAMR_R|=(1<<0)|(1<<1)|(1<<4);// capture mode and edge count,upcount 
	TIMER0_CTL_R &= ~((1<<2)|(1<<3));//positive edge detect
	TIMER0_TAPR_R |= (1<<1);
	TIMER0_TAILR_R = 0x13880;//1ms correspond to 80000 counts
	TIMER0_TAPMR_R |= (1<<0);
	TIMER0_TAMATCHR_R = 0x9C40;
	TIMER0_IMR_R |= (1<<1);
	TIMER0_CTL_R |= (1<<0);
	
}
/*void GPIOPortA_Handler(void){

	GPIO_PORTA_ICR_R|=(1<<PA2);
		FallingEdges++;
		New_Line(4);
		UART_OutUDec(FallingEdges,4);

}*

void Timer0A_Handler(void){
	TIMER0_ICR_R|=(1<<2);
	UART_OutUDec(TIMER0_TAR_R,4);
	
}*/



int main(void){

PLL_Init();
SysTick_Init();
	
UART4_Init();
	Edge_Detect_Init();

	Timer_Capture_Init();
	UART_OutUDec(1,4);
while(1)
{
	 SysTick_Wait10ms(100);

	if(((1<<TIMER0_RIS_R)&(0x1)))
	{
		TIMER0_ICR_R|=(1<<1);
		UART_OutUDec(TIMER0_TAR_R,4);
		New_Line(4);
		
	}
	
}
}