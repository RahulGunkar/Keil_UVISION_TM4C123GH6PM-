/****** Author ******/
/* 	 Riken Mehta		*/
/*    U12EE004			*/
/*	Team Robocon		*/
/****** Author ******/


/******* Info *******/
/*	UART.h
		Runs on TM4C123GH6PM (Tiva C) Stellaris LaunchPad
		Contains Declaration of all the UART viz.(UART1, UART2, UART3, UART4, UART5, UART7)
		Contains Declaration of all general function for OutChar and InChar									
*/


/* All #defines */
#define UART1_PORTB 1
#define UART1_PORTC 0
#define UART_1 1
#define UART_2 2
#define UART_3 3
#define UART_4 4
#define UART_5 5
#define UART_7 7 

/****** UART 1 ******/
void UART1_Init(int);
unsigned char UART1_InChar(void);
unsigned char UART1_InCharNonBlocking(void);
void UART1_OutChar(unsigned char data);


/****** UART 2 ******/
void UART2_Init(void);
unsigned char UART2_InChar(void);
unsigned char UART2_InCharNonBlocking(void);
void UART2_OutChar(unsigned char data);

/****** UART 3 ******/
void UART3_Init(void);
unsigned char UART3_InChar(void);
unsigned char UART3_InCharNonBlocking(void);
void UART3_OutChar(unsigned char data);

/****** UART 4 ******/
void UART4_Init(void);
unsigned char UART4_InChar(void);
unsigned char UART4_InCharNonBlocking(void);
void UART4_OutChar(unsigned char data);

/****** UART 5 ******/
void UART5_Init(void);
unsigned char UART5_InChar(void);
unsigned char UART5_InCharNonBlocking(void);
void UART5_OutChar(unsigned char data);

/****** UART 7 ******/
void UART7_Init(void);
unsigned char UART7_InChar(void);
unsigned char UART7_InCharNonBlocking(void);
void UART7_OutChar(unsigned char data);

/****** General Function *******/
void UART_OutUDec(unsigned long n,int i);
void New_Line(int i);
void Print_Space(int n,int i);
