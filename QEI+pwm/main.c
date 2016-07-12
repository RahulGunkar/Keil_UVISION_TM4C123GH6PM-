#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void)	;  // low power mode

char data;

/*
enum {forward,backward,right,left};

//pwm funtion
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

void PWM_Init(void)
	{  
    
	   PD0, PD1 : Module 0 Generator 3
		 PB4, PB5 : Module 0 Generator 1
	
	
		SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    //SYSCTL_RCGCQEI_R |= (1<<0);
		SYSCTL_RCGCGPIO_R |= 0x0000000A; // activate port B Clock (PB4, PB5) as well as port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&0x0000000A) == 0){}; // wait until port D is ready
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
		GPIO_PORTD_CR_R = 0xF0;
		GPIO_PORTD_AFSEL_R |= (1<<PD0)|(1<<PD1); // enable PD0 and PD1 pin alternative functionality
		GPIO_PORTB_AFSEL_R |= (1<<PB4)|(1<<PB5);  // enable PB4 and PB5 pin alternative functionality	
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)|0x00000044;  // replace portE to portD
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)|0x00440000;  // replace portE to portD	
    GPIO_PORTD_AMSEL_R &= ~((1<<PD0)|(1<<PD1)|(1<<PB0)); // disable aternative functionality PD0 and PD1
		GPIO_PORTB_AMSEL_R &= ~((1<<PB4)|(1<<PB5)|(1<<PB0)); // disable aternative functionality PB4 and PB5	
    GPIO_PORTD_DEN_R |= (1<<PD0)|(1<<PD1)|(1<<PB0); // enable i/o on PD0 and PD1
		GPIO_PORTB_DEN_R |= (1<<PB4)|(1<<PB5)|(1<<PB0); // enable i/o on PB4 and PB5	
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


void PortA_Init(void)
{
	
 volatile unsigned long delay;
SYSCTL_RCGC2_R=0x00000001;   // to enable the clock for port A
SYSCTL_RCGC2_R |=delay; // to wait for the clock to start
//GPIO_PORTA_LOCK_R |= 0x4C4F434B;                        //port A is unlocked ..so need to unlock it
 // GPIO_PORTA_CR_R = 0xFF; // allow changes by writting to commit register on corresponding pins"
GPIO_PORTA_AMSEL_R |=0x00;               // disable analogue mode
GPIO_PORTA_PCTL_R &=0x00000000;   // clear bits on gpio pins
GPIO_PORTA_DIR_R |=0xFF;               //set directions for 0-4 as input ,rest as output
GPIO_PORTA_AFSEL_R |=0x00;            //disable all the alternative functions 
GPIO_PORTA_DEN_R |=0xFF; 	//enable digital functionality of the correponding pins
	GPIO_PORTA_DATA_R|=(1<<PA7);
	
}



void move(int x){
		if(x == forward) {
		GPIO_PORTA_DATA_R|=(1<<PA0);
		GPIO_PORTA_DATA_R|=~(1<<PA1);
		GPIO_PORTA_DATA_R|=(1<<PA2);
		GPIO_PORTA_DATA_R|=~(1<<PA3);
		GPIO_PORTA_DATA_R|=(1<<PA4);	
		GPIO_PORTA_DATA_R|=~(1<<PA5);
		GPIO_PORTA_DATA_R|=(1<<PA6);
		GPIO_PORTA_DATA_R|=~(1<<PA7);
		}
		
		else if(x == backward) {
			GPIO_PORTA_DATA_R|=~(1<<PA0);
		GPIO_PORTA_DATA_R|=(1<<PA1);
		GPIO_PORTA_DATA_R|=~(1<<PA2);
		GPIO_PORTA_DATA_R|=(1<<PA3);
		GPIO_PORTA_DATA_R|=~(1<<PA4);	
		GPIO_PORTA_DATA_R|=(1<<PA5);
		GPIO_PORTA_DATA_R|=~(1<<PA6);
		GPIO_PORTA_DATA_R|=(1<<PA7);
		
		}
		else if(x == left) {
		GPIO_PORTA_DATA_R|=(1<<PA0);
		GPIO_PORTA_DATA_R|=~(1<<PA1);
		GPIO_PORTA_DATA_R|=~(1<<PA2);
		GPIO_PORTA_DATA_R|=(1<<PA3);
		GPIO_PORTA_DATA_R|=(1<<PA4);	
		GPIO_PORTA_DATA_R|=~(1<<PA5);
		GPIO_PORTA_DATA_R|=~(1<<PA6);
		GPIO_PORTA_DATA_R|=(1<<PA7);
		}
		else if(x == right) {
		GPIO_PORTA_DATA_R|=~(1<<PA0);
		GPIO_PORTA_DATA_R|=(1<<PA1);
		GPIO_PORTA_DATA_R|=(1<<PA2);
		GPIO_PORTA_DATA_R|=~(1<<PA3);
		GPIO_PORTA_DATA_R|=~(1<<PA4);	
		GPIO_PORTA_DATA_R|=(1<<PA5);
		GPIO_PORTA_DATA_R|=(1<<PA6);
		GPIO_PORTA_DATA_R|=~(1<<PA7);

		}
		
}
void receive_move(char b) {
			if(b == 'u')
				move(forward);
			else if(b =='l')
				move(left);
			else if(b =='d')
				move(backward);
			else if(b =='r')
				move(right);
}*/
UART5_Handler(){
  
	UART5_ICR_R = UART_ICR_RXIC;
	UART5_DR_R|=data;
	
	
}
int main()
{ 
	char a;
	PLL_Init();
	UART5_Init();
	EnableInterrupts();
	//PortA_Init();
	//PWM_Init();
	//PWM(999,999);
	//PWM2(999,999);
	0x40011030|=(1<<8);

	while(1) 
	{ 
		data=UART5_InChar();	
		
		 //receive_move(a);
		  UART5_OutChar(data);
		  New_Line(5);
		
		
			
	}
}
