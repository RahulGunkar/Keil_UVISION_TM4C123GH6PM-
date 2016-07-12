#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"

#define Kp 0.1
#define Ki 0.0005

#define Kd 0.00

volatile int int_flag;
volatile float set_rpm=220.0;
volatile float PID_value=0;
volatile int pwm = 0;
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode


volatile int rpm=0, prev_rpm=0;
float eIntg=0, prev_error=0;

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
	int_flag++;		
}



void QEI_Init(void)
{
	//pc5,pc6	
	SYSCTL_RCGCQEI_R|=(1<<1);
	 SYSCTL_RCGC2_R |= 0x00000004;     // 1) C clock
   GPIO_PORTC_CR_R = 0xF0;           // allow changes to PC7-4       
  GPIO_PORTC_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xF00FFFFF)|0x06600000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTC_DIR_R &= ~((1<<6)|(1<<5));          // 5) PD7,PD6 input   
  GPIO_PORTC_AFSEL_R |= (1<<6)|(1<<5);        // 6)  alternate function
  GPIO_PORTC_DEN_R = 0xF0;          // 7) enable digital pins P7-4        	
	QEI1_CTL_R |=(1<<3)|(1<<4)|(1<<5) ;
	QEI1_MAXPOS_R = 0x00000F9F;
	QEI1_LOAD_R = 20000000;
	QEI1_CTL_R |= (1<<0);
}


void PWM(int pwm1,int pwm2)
{
	PWM0_3_CMPA_R = 999-pwm1; 
	PWM0_3_CMPB_R = 999-pwm2;
}

void PWM_Init(void){  
   
	SYSCTL_RCGCPWM_R |= 0x00000001; // activate PWM0 clock
    SYSCTL_RCGCGPIO_R |= 0x00000008; // activate port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&0x00000008) == 0){}; // wait until port D is ready
    GPIO_PORTD_AFSEL_R |= (1<<PD0)|(1<<PD1);  // enable PD0 and PD1 pin alternative functionality
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0xFFFFFF00)|0x00000044;  // replace portE to portD
		GPIO_PORTD_AMSEL_R = 0x00; // disable aternative functionality PD0 and PD1
    GPIO_PORTD_DEN_R |= (1<<PD0)|(1<<PD1); // enable i/o on PD0 and PD1
		
		 
		SYSCTL_RCC_R |=0x00140000; // use USEPWMDIV and set divider PWMDIV to divide by 8 

   PWM0_3_CTL_R &= ~0x00000002;  
    PWM0_3_GENA_R |= 0x0000008C; 
    PWM0_3_GENB_R |= 0x0000080C; 

    PWM0_3_LOAD_R = 1000; 
			
    PWM0_3_CMPA_R =  500;
    PWM0_3_CMPB_R =  500;
    
		PWM0_3_CTL_R |= 0x00000001; // start PWM Generator 3 timers
    PWM0_ENABLE_R |= 0x000000C0; // enable PWM output
}

void PWM_assign(unsigned int PWM)
{
	PWM0_3_CMPA_R = PWM;
  PWM0_3_CMPB_R = PWM;
}


void PID_Controller(float rpm_error)
{
	
	//UART_OutUDec(rpm_error,4);
	PID_value=(Kp*rpm_error)+(Ki*eIntg)+(Kd*(rpm_error-prev_error));
	eIntg+=rpm_error;
	/*UART4_OutChar(' ');
	UART_OutUDec(PID_value,4);
	*/
	prev_error=rpm_error;
	pwm = PWM0_3_CMPA_R + PID_value;
	
	/*UART_OutUDec(PID_value,4);
	UART4_OutChar(' ');
	UART_OutUDec(pwm,4);
	UART4_OutChar(' ');
	UART_OutUDec(PWM0_3_CMPA_R,4);
	UART4_OutChar(' ');
	*/if(pwm>=999)
		pwm=999;
	else if(pwm<=0)
		pwm=0;
	PWM_assign(pwm);
	
	
}



int main(void){
	
uint32_t rpm; 
PLL_Init();
SysTick_Init();
UART5_Init();
PWM_Init();
QEI_Init();	
Timer0_Init(800000*4);//800000*4
	PWM(0,700);
	
	while(1){
	/* SysTick_Wait10ms(100);
		rpm = (QEI0_SPEED_R * 60)/( 2048);
		UART_OutUDec(rpm,4);
		UART4_OutChar(' ');
		UART_OutUDec(PWM0_3_CMPA_R,4);
		New_Line(4);
		*/if(int_flag){
		
		int_flag=0;
		rpm = (QEI1_SPEED_R * 60)/( 2048);
		prev_rpm=rpm;
		//	UART_OutUDec(rpm - set_rpm,4);
		PID_Controller(rpm - set_rpm);
		UART_OutUDec(rpm,5);	
		New_Line(5);
			
		}
	}


}






//the folowing code is for Tiva2..it should be copied in Tiva 2