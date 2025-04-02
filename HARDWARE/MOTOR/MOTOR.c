#include "MOTOR.h"
#include "delay.h"

unsigned char Time=3;

void MOTOR_GPIO_Init(void)
{
	  GPIO_InitTypeDef  GPIO_InitStructure;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;				 // 端口配置
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		      //上拉输入
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
	  GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

//顺时针转动
void MotorCW(void)
{
    unsigned short i;
    for(i=0; i<100; i++)
    {
        Coil_A1;
        delay_ms(3);
        Coil_B1;
        delay_ms(3);
        Coil_C1;
        delay_ms(3);
        Coil_D1;
        delay_ms(3);
    }
}

//逆时针转动
void MotorCCW(void)
{
    unsigned short i;
    for(i=0; i<100; i++)
    {
        Coil_D1;
       delay_ms(3);
        Coil_C1;
       delay_ms(3);
        Coil_B1;
       delay_ms(3);
        Coil_A1;
        delay_ms(3);
    }
}

void MotorStop(void)
{
		Coil_OFF;
}


