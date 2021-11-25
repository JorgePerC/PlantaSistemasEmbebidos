#ifndef __KEYPAD__
#define __KEYPAD__

//#include "stm32f1xx_hal_def.h"
#include "stm32h7xx_hal.h"


#define		ROW1_PIN		GPIO_PIN_2		//	PE2
#define  	ROW1_PORT		GPIOE

#define		ROW2_PIN		GPIO_PIN_4		//	PE4
#define  	ROW2_PORT		GPIOE

#define		ROW3_PIN		GPIO_PIN_5		//	PE5
#define  	ROW3_PORT		GPIOE			// TODO: Invert row 3 && 4

#define		ROW4_PIN		GPIO_PIN_6		//	PE6
#define  	ROW4_PORT		GPIOE

#define		COL1_PIN		GPIO_PIN_3		//	PE3
#define  	COL1_PORT		GPIOE

#define		COL2_PIN		GPIO_PIN_8		//	PF8
#define  	COL2_PORT		GPIOF

#define		COL3_PIN		GPIO_PIN_7		//	PF7
#define  	COL3_PORT		GPIOF

#define		COL4_PIN		GPIO_PIN_9		//	PA5
#define  	COL4_PORT		GPIOF

void keypad_init(void);
char keypad_read(void);
#endif
