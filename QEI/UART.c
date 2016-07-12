/****** Author ******/
/* 	 Riken Mehta		*/
/*    U12EE004			*/
/*	Team Robocon		*/
/****** Author ******/


/******* Info *******/
/*	UART.c
		Runs on TM4C123GH6PM (Tiva C) Stellaris LaunchPad
		Contains Defination of all the UART viz.(UART1, UART2, UART3, UART4, UART5, UART7)
		Contains general function for OutChar and InChar and OutDec									
*/

#include "tm4c123gh6pm.h"
#include "UART.h"

/****** UART 2 ******/

void UART2_Init(void)
{
	SYSCTL_RCGC1_R|=(1<<2);
	SYSCTL_RCGC2_R|=(1<<PORTD);
	GPIO_PORTD_LOCK_R=0x4C4F434B;
	GPIO_PORTD_CR_R|=(1<<PD7);
	UART2_CTL_R&=~(1<<0);
	UART2_IBRD_R=43;
	UART2_FBRD_R=26;
	UART2_LCRH_R = 0x00000070;
	UART2_CTL_R|=(1<<0);
	GPIO_PORTD_AFSEL_R|=(1<<PD6)|(1<<PD7);
	GPIO_PORTD_DEN_R|=(1<<PD6)|(1<<PD7);
	GPIO_PORTD_PCTL_R= (GPIO_PORTD_PCTL_R&0x00FFFFFF)+0x11000000;
	GPIO_PORTD_AMSEL_R&=~((1<<PD6)|(1<<PD7));
}

unsigned char UART2_InChar(void){
  while((UART2_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART2_DR_R&0xFF));
}


unsigned char UART2_InCharNonBlocking(void){
  if((UART2_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART2_DR_R&0xFF));
  } else{
    return 0;
  }
}

void UART2_OutChar(unsigned char data){
  while((UART2_FR_R&UART_FR_TXFF) != 0);
  UART2_DR_R = data;
}

/****** UART 1 ******/

void UART1_Init(int i)
{
	if(i)
	{
		SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
		SYSCTL_RCGC2_R |= (1<<PORTB); // activate port B
		UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART1_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART1_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART1_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTB_AFSEL_R |=(1<<PB0)|(1<<PB1);           // enable alt funct on PB0-1
		GPIO_PORTB_DEN_R |= (1<<PB0)|(1<<PB1);             // enable digital I/O on PB0-1
																					// configure PB0-1 as UART1
		GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R&0xFFFFFF00)+0x00000011;
		GPIO_PORTB_AMSEL_R &= ~((1<<PB0)|(1<<PB1));          // disable analog functionality on PB0-1
	}
	else 
	{
		SYSCTL_RCGC1_R |= SYSCTL_RCGC1_UART1; // activate UART1
		SYSCTL_RCGC2_R |= SYSCTL_RCGC2_GPIOC; // activate port C
		UART1_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART1_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART1_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART1_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART1_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTC_AFSEL_R |= 0x30;           // enable alt funct on PC5-4
		GPIO_PORTC_DEN_R |= 0x30;             // enable digital I/O on PC5-4
																					// configure PC5-4 as UART1
		GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xFF00FFFF)+0x00220000;
		GPIO_PORTC_AMSEL_R &= ~0x30;          // disable analog functionality on PC5-4
	}
  
}

unsigned char UART1_InChar(void){
  while((UART1_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART1_DR_R&0xFF));
}
unsigned char UART1_InCharNonBlocking(void){
  if((UART1_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART1_DR_R&0xFF));
  } else{
    return 0;
  }
}
void UART1_OutChar(unsigned char data){
  while((UART1_FR_R&UART_FR_TXFF) != 0);
  UART1_DR_R = data;
}


/****** UART 3 ******/

void UART3_Init()
{
		SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R3; // activate UART3
		SYSCTL_RCGC2_R |= (1<<PORTC); // activate port B
		UART3_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART3_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART3_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART3_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART3_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTC_AFSEL_R |=(1<<PC6)|(1<<PC7);           // enable alt funct on PB0-1
		GPIO_PORTC_DEN_R |= (1<<PC6)|(1<<PC7);             // enable digital I/O on PB0-1
																					// configure PB0-1 as UART1
		GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0x00FFFFFF)+0x11000000;
		GPIO_PORTC_AMSEL_R &= ~((1<<PC6)|(1<<PC7));          // disable analog functionality on PB0-1
}

unsigned char UART3_InChar(void){
  while((UART3_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART3_DR_R&0xFF));
}
unsigned char UART3_InCharNonBlocking(void){
  if((UART3_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART3_DR_R&0xFF));
  } else{
    return 0;
  }
}
void UART3_OutChar(unsigned char data){
  while((UART3_FR_R&UART_FR_TXFF) != 0);
  UART3_DR_R = data;
}

/****** UART 4 ******/

void UART4_Init()
{
		SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R4; // activate UART4
		SYSCTL_RCGC2_R |= (1<<PORTC); // activate port C
		UART4_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART4_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART4_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART4_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART4_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTC_AFSEL_R |=(1<<PC4)|(1<<PC5);           // enable alt funct on PC4-5
		GPIO_PORTC_DEN_R |= (1<<PC4)|(1<<PC5);             // enable digital I/O on PC4-5
																					// configure PC4-5 as UART4
		GPIO_PORTC_PCTL_R = (GPIO_PORTC_PCTL_R&0xFF00FFFF)+0x00110000;
		GPIO_PORTC_AMSEL_R &= ~((1<<PC4)|(1<<PC5));          // disable analog functionality on PC4-5
}

unsigned char UART4_InChar(void){
  while((UART4_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART4_DR_R&0xFF));
}
unsigned char UART4_InCharNonBlocking(void){
  if((UART4_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART4_DR_R&0xFF));
  } else{
    return 0;
  }
}
void UART4_OutChar(unsigned char data){
  while((UART4_FR_R&UART_FR_TXFF) != 0);
  UART4_DR_R = data;
}

/****** UART 5 ******/

void UART5_Init()
{
		SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R5; // activate UART4
		SYSCTL_RCGC2_R |= (1<<PORTE); // activate port C
		UART5_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART5_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART5_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART5_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART5_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTE_AFSEL_R |=(1<<PE4)|(1<<PE5);           // enable alt funct on PC4-5
		GPIO_PORTE_DEN_R |= (1<<PE4)|(1<<PE5);             // enable digital I/O on PC4-5
																					// configure PC4-5 as UART4
		GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFF00FFFF)+0x00110000;
		GPIO_PORTE_AMSEL_R &= ~((1<<PE4)|(1<<PE5));          // disable analog functionality on PC4-5
}

unsigned char UART5_InChar(void){
  while((UART5_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART5_DR_R&0xFF));
}
unsigned char UART5_InCharNonBlocking(void){
  if((UART5_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART5_DR_R&0xFF));
  } else{
    return 0;
  }
}
void UART5_OutChar(unsigned char data){
  while((UART5_FR_R&UART_FR_TXFF) != 0);
  UART5_DR_R = data;
}

/****** UART 7 ******/

void UART7_Init()
{
		SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R7; // activate UART4
		SYSCTL_RCGC2_R |= (1<<PORTE); // activate port C
		UART7_CTL_R &= ~UART_CTL_UARTEN;      // disable UART
		UART7_IBRD_R = 43;  // 80,000,000/(16*115,200)) = 43.40278
		UART7_FBRD_R = 26;  //6-bbit fraction, round(0.40278 * 64) = 26
																					// 8 bit word length (no parity bits, one stop bit, FIFOs)
		UART7_LCRH_R = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
		UART7_CTL_R |= UART_CTL_UARTEN;       // enable UART
		GPIO_PORTE_AFSEL_R |=(1<<PE0)|(1<<PE1);           // enable alt funct on PC4-5
		GPIO_PORTE_DEN_R |= (1<<PE0)|(1<<PE1);             // enable digital I/O on PC4-5
																					// configure PC4-5 as UART4
		GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R&0xFFFFFF00)+0x00000011;
		GPIO_PORTE_AMSEL_R &= ~((1<<PE0)|(1<<PE1));          // disable analog functionality on PC4-5
}

unsigned char UART7_InChar(void){
  while((UART7_FR_R&UART_FR_RXFE) != 0);
  return((unsigned char)(UART7_DR_R&0xFF));
}
unsigned char UART7_InCharNonBlocking(void){
  if((UART7_FR_R&UART_FR_RXFE) == 0){
    return((unsigned char)(UART7_DR_R&0xFF));
  } else{
    return 0;
  }
}
void UART7_OutChar(unsigned char data){
  while((UART7_FR_R&UART_FR_TXFF) != 0);
  UART7_DR_R = data;
}

/****** General Functions ******/

void UART_OutUDec(unsigned long n,int i){
  if(n >= 10){
    UART_OutUDec(n/10,i);
    n = n%10;
  }
	switch(i)
	{
		case 1:
						UART1_OutChar(n+'0'); /* n is between 0 and 9 */
						break;
		case 2:
						UART2_OutChar(n+'0');
						break;
		case 3:
						UART3_OutChar(n+'0');
						break;
		case 4:
						UART4_OutChar(n+'0');
						break;
		case 5:
						UART5_OutChar(n+'0');
						break;
		case 7:
						UART7_OutChar(n+'0');
						break;
	}
}

void New_Line(int i)
{
	switch(i)
	{
		case 1:
						UART1_OutChar(0x0A); /* n is between 0 and 9 */
						UART1_OutChar(0x0D);
						break;
		case 2:
						UART2_OutChar(0x0A);
						UART2_OutChar(0x0D);
						break;
		case 3:
						UART3_OutChar(0x0A);
						UART3_OutChar(0x0D);
						break;
		case 4:
						UART4_OutChar(0x0A);
						UART4_OutChar(0x0D);
						break;
		case 5:
						UART5_OutChar(0x0A);
						UART5_OutChar(0x0D);
						break;
		case 7:
						UART7_OutChar(0x0A);
						UART7_OutChar(0x0D);
						break;
	}
}

void Print_Space(int n,int i)
{
	while(n)
	{
		switch(i)
		{
			case 1:
						UART1_OutChar(' '); /* n is between 0 and 9 */
						break;
			case 2:
						UART2_OutChar(' ');
						break;
			case 3:
						UART3_OutChar(' ');
						break;
			case 4:
						UART4_OutChar(' ');
						break;
			case 5:
						UART5_OutChar(' ');
						break;
			case 7:
						UART7_OutChar(' ');
						break;
		}
		n--;
	}
}


