#include "key.h"
#include "delay.h"	 
 	   
		 
void KEY_IO_Init(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;
 	
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14| GPIO_Pin_15;				 //继电器输出 端口配置
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; 		
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
   GPIO_Init(GPIOB, &GPIO_InitStructure);	
}

void fengmingqi_Init(void)
{
	
	GPIO_InitTypeDef GPIO_InitStructure;

 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB,ENABLE);//使能PORTA,PORTC时钟

	GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE);//关闭jtag，使能SWD，可以用SWD模式调试
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				//PORTA0 推挽输出
 	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		  
 	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
 	GPIO_Init(GPIOA, &GPIO_InitStructure);
 	GPIO_ResetBits(GPIOA,GPIO_Pin_8);
} 
void BEEP_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB, ENABLE);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOB, GPIO_Pin_10);	 // turn off all led
	GPIO_ResetBits(GPIOB, GPIO_Pin_11);	 // turn off all led
	
}
void RELAY_Init(void)
{
   GPIO_InitTypeDef  GPIO_InitStructure;
 	
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	 //使能PB,PA端口时钟
	
	 RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//GPIO复用功能时钟使能
   GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable, ENABLE); //将A15,B3,B4这些引脚配置成普通IO口
	
   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;				 //继电器输出 端口配置
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
   GPIO_Init(GPIOA, &GPIO_InitStructure);	
	 GPIO_ResetBits(GPIOA,GPIO_Pin_15); 						
}



