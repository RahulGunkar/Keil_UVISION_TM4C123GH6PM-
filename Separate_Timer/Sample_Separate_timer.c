#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"
#include "stdlib.h"
#include "stdbool.h"

#include <stdio.h>
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

volatile uint32_t count_a=0,count_b=0;

void WideTimer0A_Init()
{
	
	SYSCTL_RCGC2_R |= (1<<PORTC);
	GPIO_PORTC_DEN_R |= (1<<PC4)|(1<<PC7);
	GPIO_PORTC_ODR_R &= ~((1<<PC4)|(1<<PC7));
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0x0FF0FFFF)| 0x70070000;
	GPIO_PORTC_AFSEL_R |= (1<<PC4)|(1<<PC7);
	
	SYSCTL_RCGCWTIMER_R |= 0x03;   // 0) activate Wide TIMER1 clock
	WTIMER0_CTL_R = 0x00000000;    // 1) disable Wide TIMER1B during setup
		WTIMER1_CTL_R = 0x00000000;    // 1) disable Wide TIMER1B during setup
	WTIMER0_CFG_R = 0x00000004;    // 2) configure for Input-Edge mode
	WTIMER1_CFG_R = 0x00000004;    // 2) configure for Input-Edge mode
	WTIMER0_TAMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER1_TBMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER0_CTL_R |= 0x0000000C; // 4) Configure for timing mode
	WTIMER1_CTL_R |= 0x00000C00; // 4) Configure for timing mode
	WTIMER0_TAILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER1_TBILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER0_TAPR_R = 0;
	WTIMER1_TBPR_R = 0;
	WTIMER0_ICR_R = 0x00000004;
	WTIMER1_ICR_R = 0x00000400;
	WTIMER0_IMR_R = 0x00000004;    // 6) Timer capture event match interrupt
	WTIMER1_IMR_R = 0x00000400;    // 6) Timer capture event match interrupt
	NVIC_PRI23_R = (NVIC_PRI23_R&0xFF00FFFF)|0x00000000; // 7) priority 0 (Highest)
	NVIC_PRI24_R = (NVIC_PRI24_R&0xFFFF0FFF)|0x00000000; // 7) priority 0 (Highest)
	NVIC_EN2_R=1<<30;  // 8) Enable WideTimer0A_Handler
	NVIC_EN3_R=1<<1;  // 8) Enable WideTimer0A_Handler
	WTIMER0_CTL_R|= (0x00000001);	 //	9) Timer Enable
	WTIMER1_CTL_R|= (0x00000100);	 //	9) Timer Enable
}

void WideTimer0A_Handler(){
	
	
	WTIMER0_ICR_R = TIMER_ICR_CAECINT;
	count_a=WTIMER0_TAR_R;
}




void WideTimer1B_Handler(){
	
	
	WTIMER1_ICR_R = TIMER_ICR_CBECINT;
	count_b=WTIMER1_TBR_R;
}

int main(void){

	PLL_Init();
	SysTick_Init();
	UART5_Init();
	WideTimer0A_Init();
	
	EnableInterrupts();

	while(1){
		
		SysTick_Wait10ms(100);
	
		UART_OutUDec(count_a,5);
		Print_Space(4,5);
		UART_OutUDec(count_b,5);
		New_Line(5);
	
	}

}