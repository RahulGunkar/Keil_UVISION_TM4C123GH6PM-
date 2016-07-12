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

volatile uint32_t rpm;


void WideTimer0A_Init()
{
	//UART5_OutChar('A');
	SYSCTL_RCGC2_R |= (1<<PORTC);
	GPIO_PORTC_DEN_R |= (1<<PC4);
	GPIO_PORTC_ODR_R &= ~(1<<PC4);
	GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R & 0xFFF0FFFF)| 0x00070000;
	GPIO_PORTC_AFSEL_R |= (1<<PC4);
	
	SYSCTL_RCGCWTIMER_R |= 0x01;   // 0) activate Wide TIMER0 clock
	WTIMER0_CTL_R = 0x00000000;    // 1) disable Wide TIMER0A during setup
	WTIMER0_CFG_R = 0x00000004;    // 2) configure for Input-Edge mode
	WTIMER0_TAMR_R= 0x00000007;		 // 3) Input Rising Edge Timing mode
	WTIMER0_CTL_R |= 0x0000000C; // 4) Configure for timing mode
	WTIMER0_TAILR_R = 0xFFFFFFFF;  // 5) Reload value
	WTIMER0_TAPR_R = 0;
	WTIMER0_ICR_R = 0x00000004;
	WTIMER0_IMR_R = 0x00000004;    // 6) Timer capture event match interrupt
	NVIC_PRI23_R = (NVIC_PRI23_R&0xFF00FFFF)|0x00000000; // 7) priority 0 (Highest)
	NVIC_EN2_R=1<<30;  // 8) Enable WideTimer0A_Handler
	WTIMER0_CTL_R|= (0x00000001);	 //	9) Timer Enable
}



void WideTimer0A_Handler()
{
		
		WTIMER0_ICR_R = TIMER_ICR_CAECINT;	// acknowledge TIMER0A capture event
    rpm=WTIMER0_TAR_R;
}

int main(void){
	PLL_Init();
	SysTick_Init();
	//Switch_Init();
	UART5_Init();
	WideTimer0A_Init();
	//Timer0_Init(8000000*4);
	//PWM_QEI_Init();
	EnableInterrupts();
	
//	PWM(200,200);
	
	while(1)
	{	

		SysTick_Wait10ms(100);			
		UART_OutUDec(rpm,5);
		New_Line(5);
			
			/*rpm_QEI = (QEI0_SPEED_R * 60)/( 2048);
			rpm = (6.0f)/(90*4.5*vel_sum);
			UART_OutUDec(rpm,5);
			Print_Space(4,5);
			UART_OutUDec(rpm_QEI,5);
			New_Line(5);
			*/
					
	}
}