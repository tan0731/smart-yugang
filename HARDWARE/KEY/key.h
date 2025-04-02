#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"  	 

#define BEEP PAout(8)	 
#define BJ_LED PBout(11)
#define juli PBout(10)        //led

#define 	KEY1 PBin(12)
#define 	KEY2 PBin(13)
#define 	KEY3 PBin(14)
#define 	KEY4 PBin(15)


void fengmingqi_Init(void);//IO≥ı ºªØ
void BEEP_GPIO_Config(void);
void KEY_IO_Init(void);
void RELAY_Init(void);
				    
#endif
