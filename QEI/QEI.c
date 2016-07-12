#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"



void QEI_Init(void)
{
	unsigned long delay;
	SYSCTL_RCGCQEI_R|=(1<<0);
	//SYSCTL_RCGCGPIO_R|=(1<<3);
	 SYSCTL_RCGC2_R |= 0x00000008;     // 1) D clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTD_LOCK_R = 0x4C4F434B ;   //Unlock PD7
  GPIO_PORTD_CR_R = 0xF0;           // allow changes to PD7-4       
  GPIO_PORTD_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x00FFFFFF)|0x66000000;   
  GPIO_PORTD_DIR_R &= ~((1<<7)|(1<<6));          // 5) PD7,PD6 input   
  GPIO_PORTD_AFSEL_R |= (1<<7)|(1<<6);        // 6) alternate function
  GPIO_PORTD_DEN_R = 0xF0;          // 7) enable digital pins P7-4        	
	
	//pd6,7
	QEI0_CTL_R |=(1<<3)|(1<<4)|(1<<5) ;
	QEI0_MAXPOS_R = 0x00000F9F;
	QEI0_LOAD_R = 20000000;
	QEI0_CTL_R |= (1<<0);
}

int main(void){
	
uint32_t rpm; 
PLL_Init();
SysTick_Init();
UART4_Init();

QEI_Init();	

	while(1){
	
		SysTick_Wait10ms(100);
		rpm = (QEI0_SPEED_R * 60)/( 2048);
		UART_OutUDec(rpm,4);
		New_Line(4);
	}


}