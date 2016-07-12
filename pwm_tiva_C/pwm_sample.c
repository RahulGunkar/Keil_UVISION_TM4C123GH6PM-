#include "tm4c123gh6pm.h"
#include "PLL.h" 
#include "SysTick.h"
#include "UART.h"



void PWM(int pwm1,int pwm2)
{
	PWM0_3_CMPA_R = 398-pwm1; 
	PWM0_3_CMPB_R = 398-pwm2;
}

void PWM_Init(void){  
    SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= 0x00000008; // activate port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&0x00000008) == 0){}; // wait until port D is ready
    GPIO_PORTD_AFSEL_R |= (1<<PD0)|(1<<PD1);  // enable PD0 and PD1 pin alternative functionality
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)|0x00000044;  // replace portE to portD
    GPIO_PORTD_AMSEL_R &= ~((1<<PD0)|(1<<PD1)); // disable aternative functionality PD0 and PD1
    GPIO_PORTD_DEN_R |= (1<<PD0)|(1<<PD1); // enable i/o on PD0 and PD1
    SYSCTL_RCC_R |=0x00140000; // use USEPWMDIV and set divider PWMDIV to divide by 8 
GPIO_PORTD_DR_R|=0;
   PWM0_3_CTL_R &= ~0x00000002;  
    PWM0_3_GENA_R |= 0x0000008C; 
    PWM0_3_GENB_R |= 0x0000080C; 

    PWM0_3_LOAD_R = 399; 
			
    PWM0_3_CMPA_R =  299;
    PWM0_3_CMPB_R =  299;
    
		PWM0_3_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= 0x00000C0; // enable PWM output
}

void Switch_Init(void){  unsigned long volatile delay;
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
  GPIO_PORTF_LOCK_R = 0x4C4F434B; // unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;         // allow changes to PF4,0
  //GPIO_PORTF_DIR_R &= ~0x11;    // (c) make PF4,0 in (built-in button)
  GPIO_PORTF_DIR_R |= 0x00;
	GPIO_PORTF_DATA_R |= (0x00);
	GPIO_PORTF_AFSEL_R &= ~0x11;  //     disable alt funct on PF4,0
  GPIO_PORTF_DEN_R |= 0x1F;     //     enable digital I/O on PF4,0
  GPIO_PORTF_PCTL_R &= ~0x000FFFFF; //  configure PF4,0 as GPIOc
  GPIO_PORTF_AMSEL_R &= ~0x1F;  //     disable analog functionality on PF4,0
  GPIO_PORTF_PUR_R |= 0x11;     //     enable weak pull-up on PF4,0
}

int main(void)
{
	
	SysTick_Init();
	PWM_Init();
	//Switch_Init();
	PWM(300,100);
	while(1)
	{
		
		//if(!(GPIO_PORTF_DATA_R&(1<<PF0)))
			
		
	}
}

