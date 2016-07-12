#include "tm4c123gh6pm.h"
#include "PLL.h" 
#include "SysTick.h"
#include "UART.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>


void EnableInterrupts(void);
void DisableInterrupts(void);
void WaitForInterrupt(void);

volatile bool disp_flag=0;
volatile float set_rpm=200.0;
volatile int rpm=0, prev_rpm=0;
volatile int wheel1_clockwise,wheel2_clockwise;
float eIntg=0, prev_error=0;

float Kp[2] = {0,0};
float Ki[2] = {0,0};
float Kd[2] = {0,0};


void PWM_assign(unsigned int PWM)
{
	PWM0_3_CMPA_R = PWM;
  PWM0_3_CMPB_R = PWM;
}

int determine_rotation1(void){
	
	if((QEI0_STAT_R) & (0x00000002))
	{
		wheel1_clockwise=1;
		
		}
	else wheel1_clockwise=0;

		return wheel1_clockwise;
}

int determine_rotation2(void){
	
	if((QEI1_STAT_R) & (0x00000002))
	{
		wheel2_clockwise=1;
		
		}
	else wheel2_clockwise=0;

		return wheel2_clockwise;
}



void PID_Controller(float rpm_error)
{
	float PID_value=0;
	long int pwm = 0;
	PID_value=(Kp*rpm_error)+(Ki*eIntg*0.01)+(Kd*(rpm_error-prev_error)/0.01);
	eIntg+=rpm_error;
	prev_error=rpm_error;
	pwm=(long int)PWM0_3_CMPA_R + PID_value;
	if(pwm>=998)
		pwm=998;
	else if(pwm<=0)
		pwm=0;
	PWM_assign(pwm);
}

void Timer0_Init(unsigned long period)
{
  SYSCTL_RCGCTIMER_R |= 0x01;   // 0) activate TIMER0  
  TIMER0_CTL_R = 0x00000000;    // 1) disable TIMER0A during setup
  TIMER0_CFG_R = 0x00000000;    // 2) configure for 32-bit mode
  TIMER0_TAMR_R = 0x00000002;   // 3) configure for periodic mode, default down-count settings
  TIMER0_TAILR_R = period-1;    // 4) reload value
  TIMER0_TAPR_R = 0;            // 5) bus clock resolution
  TIMER0_ICR_R = 0x00000001;    // 6) clear TIMER0A timeout flag
  TIMER0_IMR_R = 0x00000001;    // 7) arm timeout interrupt
  NVIC_PRI4_R = (NVIC_PRI4_R&0x00FFFFFF)|0x80000000; // 8) priority 4
	// interrupts enabled in the main program after all devices initialized
	// vector number 35, interrupt number 19
  NVIC_EN0_R = 1<<19;           // 9) enable IRQ 19 in NVIC
  TIMER0_CTL_R = 0x00000001;    // 10) enable TIMER0A
}

void Timer0A_Handler()
{
	TIMER0_ICR_R = TIMER_ICR_TATOCINT;// acknowledge TIMER0A timeout
	disp_flag=1;		
}

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





int LPF(int prev, int curr, float weight)
{
	return weight*curr + (1-weight)*prev;
}


int main(void)
{
	uint32_t rpm,rpm1;
	PLL_Init();
	SysTick_Init();
	UART5_Init();
	PortF_Init();
	Timer0_Init(800000*4);
	PWM_Init();
	QEI0_Init();
	QEI1_Init();
	
	
	
	PWM(300,300);
	PWM2(300,300);
	while(1)
	{
		
		if(disp_flag)
		{
			disp_flag=0;
			rpm = (QEI0_SPEED_R * 60)/( 2048);
			rpm1 = (QEI1_SPEED_R * 60)/( 2048);

			prev_rpm=rpm;
			rpm = LPF(prev_rpm, rpm, 0.9);
		
			PID_Controller(rpm - set_rpm);
			
		}	
		
		
		
	}
}

void UART5_Handler(){
	

	UART5_ICR_R = UART_ICR_RXIC;
	
	
	
}

