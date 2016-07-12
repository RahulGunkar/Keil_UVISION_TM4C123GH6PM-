/****** Author ******/
/* 	 Riken Mehta		*/
/*    U12EE004			*/
/*	Team Robocon		*/
/****** Author ******/



/******* Info *******/
/*	SysTick.h
		Runs on TM4C123GH6PM (Tiva C) Stellaris LaunchPad
		Initialize SysTick timer for accurate CPU frequency and delay function
*/



// Initialize SysTick with busy wait running at bus clock.
void SysTick_Init(void);

// Time delay using busy wait.
// The delay parameter is in units of the core clock. (units of 12.5 nsec for 80 MHz clock)
void SysTick_Wait(unsigned long delay);

// Time delay using busy wait.
// This assumes 80 MHz system clock.
void SysTick_Wait10ms(unsigned long delay);
