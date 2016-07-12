#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"


void QEI_Init(void)
{
	unsigned long delay;
	SYSCTL_RCGCQEI_R|=(1<<1);
	 SYSCTL_RCGC2_R |= 0x00000004;     // 1) C clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTC_CR_R = 0xF0;           // allow changes to PC7-4       
  GPIO_PORTC_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xF00FFFFF)|0x06600000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTC_DIR_R &= ~((1<<6)|(1<<5));          // 5) PD7,PD6 input   
  GPIO_PORTC_AFSEL_R |= (1<<6)|(1<<5);        // 6) no alternate function
  GPIO_PORTC_DEN_R = 0xF0;          // 7) enable digital pins P7-4        	
	QEI1_CTL_R |=(1<<3)|(1<<4)|(1<<5) ;
	QEI1_MAXPOS_R = 0x00000F9F;
	QEI1_LOAD_R = 20000000;
	QEI1_CTL_R |= (1<<0);
}

int main(void){
	
uint32_t rpm; 
PLL_Init();
SysTick_Init();
UART5_Init();

QEI_Init();	

	while(1){
	
		SysTick_Wait10ms(100);
		rpm = (QEI1_SPEED_R * 60)/( 2048);
		UART_OutUDec(rpm,5);
		New_Line(5);
	}


}