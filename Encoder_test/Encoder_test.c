#include "tm4c123gh6pm.h"
#include "PLL.h"
#include "SysTick.h"
#include "UART.h"

void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void WaitForInterrupt(void);  // low power mode

volatile unsigned long FallingEdges = 0;

void Encoder_Counter_Init(void)
{
	SYSCTL_RCGC2_R|=(1<<PORTB)|(1<<PORTA);  //Enable clock for PortA, PortB
	GPIO_PORTB_DIR_R&=~((1<<PB2)|(1<<PB3));  //Set GPIO pins as Input
	GPIO_PORTA_DIR_R&=~((1<<PA2)|(1<<PA3));
	GPIO_PORTB_AFSEL_R&=~((1<<PB2)|(1<<PB3)); //Disable alt. function
	GPIO_PORTA_AFSEL_R&=~((1<<PA2)|(1<<PA3));
	GPIO_PORTB_DEN_R|=(1<<PB2)|(1<<PB3);  //Select Digital I/O operation
	GPIO_PORTA_DEN_R|=(1<<PA2)|(1<<PA3);
	GPIO_PORTB_PCTL_R&=~0x0000FF00;   //Configure for GPIO operation
	GPIO_PORTA_PCTL_R&=~0x0000FF00;
	GPIO_PORTB_AMSEL_R&=~((1<<PB2)|(1<<PB3));  //Disable analog function
	GPIO_PORTA_AMSEL_R&=~((1<<PA2)|(1<<PA3));
	
	//Rising Edge, Interrupt,Input configuration
	GPIO_PORTB_IS_R &= ~(1<<PB2); //clearing enables edge sensitivity
	GPIO_PORTA_IS_R &= ~(1<<PA2);	//same as above
	GPIO_PORTB_IBE_R &= ~(1<<PB2);//Interrupt generation is controlled by the GPIO Interrupt Event
	GPIO_PORTA_IBE_R &= ~(1<<PA2);//	same as above
	GPIO_PORTB_IEV_R|=(1<<PB2);		//rising edge trigger
	GPIO_PORTA_IEV_R|=(1<<PA2);		//same as above
	GPIO_PORTB_ICR_R|=(1<<PB2);		//edge-detect
	GPIO_PORTA_ICR_R|=(1<<PA2);		//same as above
	GPIO_PORTB_IM_R|=(1<<PB2);		//allow interrupt to be sent to controller
	GPIO_PORTA_IM_R|=(1<<PA2);		//same as above
	
	NVIC_PRI0_R= (NVIC_PRI0_R&0xFFFF0F0F)|0x0000A0A0;  //Set Interrupt priority to 5
	
	NVIC_EN0_R|=(1<<PORTA)|(1<<PORTB);  //Enable Interrupt in NVIC
	
	EnableInterrupts();    
}

void GPIOPortF_Handler(void)
{
		GPIO_PORTA_ICR_R|=(1<<PA2);

		FallingEdges++;
		New_Line(7);
		UART_OutUDec(FallingEdges,7);
	
	/*if(GPIO_PORTA_DATA_R&(1<<PA3))
	{		R_ticks++;
		UART4_OutChar('I');
		if(R_ticks==100)
		{
			i=1;
		}
	}
		else R_ticks--;
*/
}


/*
void EdgeCounter_Init(void){ volatile unsigned long delay;       
  SYSCTL_RCGC2_R |= 0x00000020; // (a) activate clock for port F
  delay = SYSCTL_RCGC2_R;
	FallingEdges = 0;             // (b) initialize count and wait for clock
  GPIO_PORTF_DIR_R &= ~0x10;    // (c) make PF4 in (built-in button)
  GPIO_PORTF_AFSEL_R &= ~0x10;  //     disable alt funct on PF4
  GPIO_PORTF_DEN_R |= 0x10;     //     enable digital I/O on PF4
  GPIO_PORTF_PCTL_R &= ~0x000F0000; //  configure PF4 as GPIO
  GPIO_PORTF_AMSEL_R &= ~0x10;  //    disable analog functionality on PF4
  GPIO_PORTF_PUR_R |= 0x10;     //     enable weak pull-up on PF4
  GPIO_PORTF_IS_R &= ~0x10;     // (d) PF4 is edge-sensitive
  GPIO_PORTF_IBE_R &= ~0x10;    //     PF4 is not both edges
  GPIO_PORTF_IEV_R &= ~0x10;    //     PF4 falling edge event
  GPIO_PORTF_ICR_R = 0x10;      // (e) clear flag4
  GPIO_PORTF_IM_R |= 0x10;      // (f) arm interrupt on PF4
  NVIC_PRI7_R = (NVIC_PRI7_R&0xFF00FFFF)|0x00A00000; // (g) priority 5
  NVIC_EN0_R = 0x40000000;      // (h) enable interrupt 30 in NVIC
  EnableInterrupts();           // (i) Enable global Interrupt flag (I)
}

void GPIOPortF_Handler(void){
  GPIO_PORTF_ICR_R = 0x10;      // acknowledge flag4
  FallingEdges = FallingEdges + 1;
	New_Line(4);
	UART_OutUDec(FallingEdges,4);
}
*/
int main(void){ 
		PLL_Init();
		SysTick_Init();
		UART7_Init();
		UART7_OutChar('A');
   Encoder_Counter_Init(); // initialize GPIO Port F interrupt 
   while(1){ 
      WaitForInterrupt(); 
   } 
}

/*
void Encoder_Counter_Init(void)
{
	SYSCTL_RCGC2_R|=(1<<PORTB)|(1<<PORTA);  //Enable clock for PortA, PortB
	GPIO_PORTB_DIR_R&=~((1<<PB2)|(1<<PB3));  //Set GPIO pins as Input
	GPIO_PORTA_DIR_R&=~((1<<PA2)|(1<<PA3));
	GPIO_PORTB_AFSEL_R&=~((1<<PB2)|(1<<PB3)); //Disable alt. function
	GPIO_PORTA_AFSEL_R&=~((1<<PA2)|(1<<PA3));
	GPIO_PORTB_DEN_R|=(1<<PB2)|(1<<PB3);  //Select Digital I/O operation
	GPIO_PORTA_DEN_R|=(1<<PA2)|(1<<PA3);
	GPIO_PORTB_PCTL_R&=~0x0000FF00;   //Configure for GPIO operation
	GPIO_PORTA_PCTL_R&=~0x0000FF00;
	GPIO_PORTB_AMSEL_R&=~((1<<PB2)|(1<<PB3));  //Disable analog function
	GPIO_PORTA_AMSEL_R&=~((1<<PA2)|(1<<PA3));
	
	//Rising Edge, Interrupt,Input configuration
	GPIO_PORTB_IS_R &= ~(1<<PB2); //clearing enables edge sensitivity
	GPIO_PORTA_IS_R &= ~(1<<PA2);	//same as above
	GPIO_PORTB_IBE_R &= ~(1<<PB2);//Interrupt generation is controlled by the GPIO Interrupt Event
	GPIO_PORTA_IBE_R &= ~(1<<PA2);//	same as above
	GPIO_PORTB_IEV_R|=(1<<PB2);		//rising edge trigger
	GPIO_PORTA_IEV_R|=(1<<PA2);		//same as above
	GPIO_PORTB_ICR_R|=(1<<PB2);		//edge-detect
	GPIO_PORTA_ICR_R|=(1<<PA2);		//same as above
	GPIO_PORTB_IM_R|=(1<<PB2);		//allow interrupt to be sent to controller
	GPIO_PORTA_IM_R|=(1<<PA2);		//same as above
	
	NVIC_PRI0_R= (NVIC_PRI0_R&0xFFFF0F0F)|0x0000A0A0;  //Set Interrupt priority to 5
	
	NVIC_EN0_R|=(1<<PORTA)|(1<<PORTB);  //Enable Interrupt in NVIC
	
	EnableInterrupts();
}

void GPIOPortA_Handler(void)
{

}

*/




















/*void PortF_Init(void){ volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) F clock
  delay = SYSCTL_RCGC2_R;           // delay   
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock PortF PF0  
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0       
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog function
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) GPIO clear bit PCTL  
  GPIO_PORTF_DIR_R = 0x0E;          // 5) PF4,PF0 input, PF3,PF2,PF1 output   
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) no alternate function
  GPIO_PORTF_PUR_R = 0x11;          // enable pullup resistors on PF4,PF0       
  GPIO_PORTF_DEN_R = 0x1F;          // 7) enable digital pins PF4-PF0        
}
*/