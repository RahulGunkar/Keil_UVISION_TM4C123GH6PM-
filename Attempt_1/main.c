#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"
#include "stdlib.h"
#include "stdbool.h"
#include "fifo.h"
#include "Queue_Lib.h"
#include <stdio.h>

#define FIFO_size 500
volatile float time_queue[FIFO_size];
FIFO fifo;

uint32_t rpmA,rpm_QEI,rpmB,rpm_QEI1;


void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
void WaitForInterrupt(void);  // low power mode

volatile bool disp_flag=0;
volatile bool cap_index=0,cap_index_1=0;
volatile int queue_count=0,queue_count_1;
	
volatile unsigned int cap_val[2],cap_val1[2];
volatile float vel_sum=1,vel_sum_1;

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


	
void WideTimer0A_Handler()
{
			//UART5_OutChar('C');
		float temp=0;
		WTIMER0_ICR_R = TIMER_ICR_CAECINT;	// acknowledge TIMER0A capture event
    cap_val[0]= WTIMER0_TAR_R;  // execute user task
		if(cap_index)
		{
			temp=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//vel_sum +=(cap_val[1]-cap_val[0])*12.5*0.000000001;
			//edge_counter++;
			if(queue_count<FIFO_size)
			{
				//UART5_OutChar('D');
				queue_count++;
				vel_sum=(vel_sum*(queue_count-1) + temp)/(queue_count);
				fifo_put(&fifo, temp);
			}else if(queue_count==FIFO_size)
			{
				vel_sum +=((temp - fifo_get(&fifo))/(FIFO_size*1.0f)); 
				fifo_put(&fifo,temp);
			}
			//UART_OutUDec(vel_sum*1000,0);
			//Print_Space(2,0);
		}else cap_index=1;
		cap_val[1]=cap_val[0];
}





void WideTimer1B_Handler(){
	
		float temp1=0;
		WTIMER1_ICR_R = TIMER_ICR_CBECINT;	// acknowledge TIMER0A capture event
    cap_val1[0]= WTIMER1_TBR_R;  // execute user task
		if(cap_index_1)
		{
			temp1=(cap_val1[1]-cap_val1[0])*12.5*0.000000001;
						if(queue_count_1<FIFO_size)
			{
								queue_count_1++;
				vel_sum_1=(vel_sum_1*(queue_count_1-1) + temp1)/(queue_count_1);
				fifo_put(&fifo, temp1);
			}else if(queue_count_1==FIFO_size)
			{
				vel_sum_1 +=((temp1 - fifo_get(&fifo))/(FIFO_size*1.0f)); 
				fifo_put(&fifo,temp1);
			}
					}else cap_index_1=1;
		cap_val1[1]=cap_val1[0];
	
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
    SYSCTL_RCGCGPIO_R |= 0x0000000A; // activate port B Clock (PB4, PB5) as well as port D clock  (PD0, PD1)
    while((SYSCTL_PRGPIO_R&0x0000000A) == 0){}; // wait until port D is ready
    GPIO_PORTD_LOCK_R = 0x4C4F434B;
		GPIO_PORTD_CR_R = 0xF0;
			GPIO_PORTD_AFSEL_R |= (1<<PD0)|(1<<PD1)|(1<<PD7)|(1<<PD6);  // enable PD0 and PD1 pin alternative functionality
		GPIO_PORTB_AFSEL_R |= (1<<PB4)|(1<<PB5);  // enable PB4 and PB5 pin alternative functionality	
    GPIO_PORTD_PCTL_R = (GPIO_PORTD_PCTL_R&0x00FFFF00)|0x66000044;  // replace portE to portD
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFF00FFFF)|0x00440000;  // replace portE to portD	
    GPIO_PORTD_AMSEL_R &= ~((1<<PD0)|(1<<PD1)|(1<<PD6)|(1<<PD7)); // disable aternative functionality PD0 and PD1
		GPIO_PORTB_AMSEL_R &= ~((1<<PB4)|(1<<PB5)); // disable aternative functionality PB4 and PB5	
    GPIO_PORTD_DEN_R |= (1<<PD0)|(1<<PD1)|(1<<PD6)|(1<<PD7); // enable i/o on PD0 and PD1
		GPIO_PORTB_DEN_R |= (1<<PB4)|(1<<PB5); // enable i/o on PB4 and PB5	
    GPIO_PORTD_DIR_R &= ~((1<<PD7)|(1<<PD6));
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
void QEI_Init(void)
{
	//pd6,pd7
	SYSCTL_RCGCQEI_R|=(1<<0);
	QEI0_CTL_R |=(1<<3)|(1<<4)|(1<<5) ;
	QEI0_MAXPOS_R = 0x00000F9F;
	QEI0_LOAD_R = 20000000;
	QEI0_CTL_R |= (1<<0);
}

void QEI1_Init(void){

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

int main(void)
{
	uint32_t rpmA,rpm_QEI,rpmB,rpm_QEI1;
	PLL_Init();
	SysTick_Init();
	PWM_Init();
	UART5_Init();
	Switch_Init();
	fifo_init(&fifo, FIFO_size, (float*)time_queue);
	WideTimer0A_Init();
	Timer0_Init(8000000*4);
	QEI_Init();
	QEI1_Init();
	EnableInterrupts();
	PWM(250,250);
	PWM2(250,250);
	while(1)
	{
		
		if(disp_flag){
		
		disp_flag=0;
			rpm_QEI = (QEI0_SPEED_R * 60)/( 2048);
			rpm_QEI1 = (QEI1_SPEED_R * 60)/( 2048);
			rpmA = (6.0f)/(90*4.5*vel_sum);
			rpmB = (6.0f)/(90*4.5*vel_sum_1);
			UART_OutUDec(rpmA,5);
			Print_Space(4,5);
			UART_OutUDec(rpm_QEI,5);
			Print_Space(4,5);
			UART_OutUDec(rpmB,5);
			Print_Space(4,5);
			UART_OutUDec(rpm_QEI1,5);
			New_Line(5);
		
		
		
		
		
		}
			
		
	}
}
