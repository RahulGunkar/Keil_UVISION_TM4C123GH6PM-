#include "tm4c123gh6pm.h"
#include "PLL.h" 
#include "SysTick.h"
#include "UART.h"


void QEI1_Init(void)
{
	//pc5,pc6	
	SYSCTL_RCGCQEI_R|=(1<<1);
	 SYSCTL_RCGC2_R |= 0x00000004;     // 1) C clock
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


void QEI0_Init(void)
{
	QEI0_CTL_R |=(1<<3)|(1<<4)|(1<<5) ;
	QEI0_MAXPOS_R = 0x00000F9F;
	QEI0_LOAD_R = 20000000;
	QEI0_CTL_R |= (1<<0);
	
}

void PWM(int pwm1,int pwm2)
{
	PWM0_3_CMPA_R = 999-pwm1; 
	PWM0_3_CMPB_R = 999-pwm2;
}

void PWM2(int pwm11,int pwm22)
{
	PWM0_1_CMPA_R = 999-pwm11; 
	PWM0_1_CMPB_R = 999-pwm22;
}

void PWM_Init(void){  
    
	/* PD0, PD1 : Module 0 Generator 3
		 PB4, PB5 : Module 0 Generator 1
	*/
	
		SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCQEI_R |= (1<<0);
		SYSCTL_RCGCGPIO_R |= 0x0000000A; // activate port B Clock (PB4, PB5) as well as port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&0x0000000A) == 0){}; // wait until port D is ready
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
		GPIO_PORTD_CR_R = 0xF0;
		GPIO_PORTD_AFSEL_R |= (1<<PD0)|(1<<PD1)|(1<<PD6)|(1<<PD7);  // enable PD0 and PD1 pin alternative functionality
		GPIO_PORTB_AFSEL_R |= (1<<PB4)|(1<<PB5);  // enable PB4 and PB5 pin alternative functionality	
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x00FFFF00)|0x66000044;  // replace portE to portD
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)|0x00440000;  // replace portE to portD	
    GPIO_PORTD_AMSEL_R &= ~((1<<PD0)|(1<<PD1)|(1<<PD6)|(1<<PD7)); // disable aternative functionality PD0 and PD1
		GPIO_PORTB_AMSEL_R &= ~((1<<PB4)|(1<<PB5)); // disable aternative functionality PB4 and PB5	
    GPIO_PORTD_DEN_R |= (1<<PD0)|(1<<PD1)|(1<<PD6)|(1<<PD7); // enable i/o on PD0 and PD1
		GPIO_PORTB_DEN_R |= (1<<PB4)|(1<<PB5); // enable i/o on PB4 and PB5	
    SYSCTL_RCC_R |=0x00140000; // use USEPWMDIV and set divider PWMDIV to divide by 8 

    PWM0_3_CTL_R &= ~0x00000002;
		PWM0_1_CTL_R &= ~0x00000002;		
    
		PWM0_3_GENA_R |= 0x0000008C;
	  PWM0_1_GENA_R |= 0x0000008C;
    
		PWM0_3_GENB_R |= 0x0000080C;
		PWM0_1_GENB_R |= 0x0000080C;

    PWM0_3_LOAD_R = 1000;
		PWM0_1_LOAD_R = 1000;
			
    PWM0_3_CMPA_R =  500;
		PWM0_1_CMPA_R =  500;
		
    PWM0_3_CMPB_R =  500;
		PWM0_1_CMPB_R =  500;
    
		PWM0_3_CTL_R |= 0x00000001; // start PWM Generator 3 timers
		PWM0_1_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= 0x00000CC; // enable PWM output
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
	uint32_t rpm,rpm1;
	PLL_Init();
	SysTick_Init();
	PWM_Init();
	QEI0_Init();
	QEI1_Init();
	UART5_Init();

	Switch_Init();
	PWM(300,300);
	PWM2(300,300);
	while(1)
	{
		SysTick_Wait10ms(100);
		rpm = (QEI0_SPEED_R * 60)/( 2048);
		rpm1 = (QEI1_SPEED_R * 60)/( 2048);
		UART_OutUDec(rpm,5);
		Print_Space(4,5);
		
		
		
		UART_OutUDec(rpm1,5);
		New_Line(5);
	
		
	}
}

