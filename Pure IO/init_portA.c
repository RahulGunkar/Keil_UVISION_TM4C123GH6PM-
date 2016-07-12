#include <tm4c123gh6pm.h>

#define GPIO_PORTA_DATA_R       (*((volatile unsigned long *)0x400043FC))
#define GPIO_PORTA_DIR_R        (*((volatile unsigned long *)0x40004400))
#define GPIO_PORTA_AFSEL_R      (*((volatile unsigned long *)0x40004420))
#define GPIO_PORTA_PUR_R        (*((volatile unsigned long *)0x40004510))
#define GPIO_PORTA_DEN_R        (*((volatile unsigned long *)0x4000451C))
#define GPIO_PORTA_LOCK_R       (*((volatile unsigned long *)0x40004520))
#define GPIO_PORTA_CR_R         (*((volatile unsigned long *)0x40004524))
#define GPIO_PORTA_AMSEL_R      (*((volatile unsigned long *)0x40004528))
#define GPIO_PORTA_PCTL_R       (*((volatile unsigned long *)0x4000452C))
#define GPIO_PORTA_DR8R_R       (*((volatile unsigned long *)0x40004508))
#define SYSCTL_RCGC2_R          (*((volatile unsigned long *)0x400FE108))
	
void PortF_Init(void);

int main(void)
{
	PortF_Init();

	while(1)
	{
	}
}

void PortF_Init(void)
{ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000001;      // 1) A clock
  delay = SYSCTL_RCGC2_R;            // delay to allow clock to stabilize     
  GPIO_PORTA_AMSEL_R &= 0x00;        // 2) disable analog function
  GPIO_PORTA_PCTL_R &= 0x00000000;   // 3) GPIO clear bit PCTL  
  GPIO_PORTA_DIR_R |= (0x01);
	GPIO_PORTA_DATA_R |= (0x01);
  GPIO_PORTA_AFSEL_R = 0x00;        // 5) no alternate function
  GPIO_PORTA_PUR_R |= 0x10;          // 6) enable pullup resistor on PF4       
  GPIO_PORTA_DEN_R |= 0x1E;          // 7) enable digital pins PF4-PF1
}