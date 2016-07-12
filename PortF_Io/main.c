#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"

void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x1F;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x00;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}

int main(void){
	PLL_Init();
	SysTick_Init();
	PortF_Init();
	
	GPIO_PORTF_DATA_R &= ~(1<<1);
	GPIO_PORTF_DATA_R |= (1<<0);
	GPIO_PORTF_DATA_R &= ~(1<<2);
	GPIO_PORTF_DATA_R |= (1<<4);
	
	/*GPIO_PORTF_DATA_R |= (1<<1);          // 5) PF4,PF0 input, PF3,PF2,PF1 output
	SysTick_Wait10ms(1);
	GPIO_PORTF_DATA_R &= ~(1<<1);          // 5) PF4,PF0 input, PF3,PF2,PF1 output
	SysTick_Wait10ms(30);
	GPIO_PORTF_DATA_R |= (1<<2);	// 5) PF4,PF0 input, PF3,PF2,PF1 output
	SysTick_Wait10ms(1);
	GPIO_PORTF_DATA_R &= ~(1<<2);          // 5) PF4,PF0 input, PF3,PF2,PF1 output
*/
	while(1);
}