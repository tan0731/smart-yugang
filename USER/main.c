#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "ds18b20.h"
#include "LCD1602.h"
#include "key.h"
#include "adc.h"
#include "timer.h"
#include "string.h"
#include "esp8266.h"
#include <stdbool.h>
#include "stdio.h"
#include "MOTOR.h"
#include <stdlib.h>

int miao_flag=0,fen_flag=1,shi_flag=0; 
bool usart_send_flag=0;
int PH1,PH2;
extern int miao,fen,shi;
short temperature = 0;//温度
u8 setN = 0;//设置标志
u16 Wrtb_Max = 3000;//水浊度报警值
u8 T_max = 40;//温度报警值
u8 led=0;
float PH  = 0.0;
char send_data[]="Wrtb:0000,Temp:00.0C\r\n";
unsigned int cnt;
float T;
char buffer[10];
unsigned char Alarm_Buf[16]="Warning";
char buf[30];
long int adcx = 0;
unsigned int adc = 0;
unsigned char setn = 0;

void che_Init() 
{ 

	
	 GPIO_InitTypeDef  GPIO_InitStructure;
 	
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);	 //使能PB端口时钟
	
		GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;  //配置IO口
   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		
   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
   GPIO_Init(GPIOB, &GPIO_InitStructure);		
}


void KEY_SCAN(void)//按键扫描
{
	  if(!KEY1)//设置键
		{
		   delay_ms(10);//消抖
			 if(!KEY1)
		   {
					while(!KEY1);//等待按键松开
					setn++;
				  if(setn > 2)
					{
							setn = 0;
						LCD_Write_String(0,0,"Temperature:   C");
						LCD_Write_String(0,1,"Wrtb:    ");
						LCD_Write_String(9,1,(u8*)"PH:  ");
					}
					if(setn == 1)
					{
             LCD_Clear();
						 LCD_Write_String(0,0,"====Set Temp====");
							
						 LCD_Write_Char(7,1, T_max/10+0x30);//温度上限
							LCD_Write_Char(8,1, T_max%10+0x30);
							LCD_Write_Char(9,1,'C');
					}
					if(setn == 2)
					{
						  	LCD_Write_String(0,0,"====Set Wrtb====");
							LCD_Write_String(0,1,"       00%      ");
								LCD_Write_Char(6,1, Wrtb_Max/1000%10+0x30);
							LCD_Write_Char(7,1, Wrtb_Max/100%10+0x30);
							LCD_Write_Char(8,1, Wrtb_Max/10%10+0x30);
							LCD_Write_Char(9,1, Wrtb_Max%10+0x30);
					}
		   }
		 }
    if(!KEY2)//加键
		{
		   delay_ms(10);
			 if(!KEY2)
		   {
					while(!KEY2);//等待按键松开
					if(setn == 1)
					{
						  if(T_max < 99)T_max++;
							LCD_Write_Char(7,1,'0'+T_max/10);
						  LCD_Write_Char(8,1,'0'+T_max%10);
					}
					if(setn == 2)
					{
						  if(Wrtb_Max < 99)Wrtb_Max++;
						if(Wrtb_Max < 3500)
								Wrtb_Max = Wrtb_Max + 10;
								
								LCD_Write_Char(6,1, Wrtb_Max/1000%10+0x30);
								LCD_Write_Char(7,1, Wrtb_Max/100%10+0x30);
								LCD_Write_Char(8,1, Wrtb_Max/10%10+0x30);
								LCD_Write_Char(9,1, Wrtb_Max%10+0x30);
					}
		   }
		 }
    if(!KEY3)//减键
		{
		   delay_ms(10);
			 if(!KEY3)
		   {
					while(!KEY3);//等待按键松开
		      if(setn == 1)
					{
						  if(T_max > 0)T_max--;
							LCD_Write_Char(7,1,'0'+T_max/10);
						  LCD_Write_Char(8,1,'0'+T_max%10);
					}
					if(setn == 2)
					{
						 if(Wrtb_Max > 0)
								Wrtb_Max = Wrtb_Max - 10;
								
								LCD_Write_Char(6,1, Wrtb_Max/1000%10+0x30);
								LCD_Write_Char(7,1, Wrtb_Max/100%10+0x30);
								LCD_Write_Char(8,1, Wrtb_Max/10%10+0x30);
								LCD_Write_Char(9,1, Wrtb_Max%10+0x30);
					}
		   }
		 }		 
}

u8 biaozhiwei=0;
void Get_PH(void)    //获取PH
{
		float ph_ad;
	
		ph_ad = Get_Adc_Average(ADC_Channel_8,20);//取20次平均值
		PH = (ph_ad*3.3/4096);
		PH = PH * 5.9647;
		PH = 22.255 - PH;
		PH = PH * 100;						
		if(PH > 1400)PH = 1400;
}
int main(void)
{		
		 u8 count = 0;
		delay_init();	    	 //延时函数初始化	   
		NVIC_Configuration(); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级  		
		TIM3_Int_Init(9999,7199);//10Khz的计数频率，计数到10000为1000ms为1s 
	  LCD_Init();    //1602初始化
		Adc_Init();		  		//ADC初始化	
	  fengmingqi_Init();  //按键初始化
	  RELAY_Init();//继电器初始化
	  BEEP_GPIO_Config(); //蜂鸣器初始化
		KEY_IO_Init();
	  MOTOR_GPIO_Init();   // 电机初始化
  	che_Init();
	
		 ESP8266_Init();	
		while(DS18B20_Init())//ds18b20初始化
		{
			LCD_Write_String(0,0,(u8*)"  DS18B20 Erro  ");
			LCD_Write_String(0,1,"                ");
		}
		DS18B20_Get_Temp();//读取温度
		LCD_Write_String(0,0,"Temperature:   C");
		LCD_Write_String(0,1,(u8*)"Wrtb:    ");
		LCD_Write_String(9,1,(u8*)"PH:");
		TIM_Cmd(TIM3,ENABLE);  //使能TIMx
		while(1)
		{
			  KEY_SCAN();
			  if(count++ >= 100 && setn == 0)
				{
						count = 0;
					
					 if(cnt++ > 10)
						{
								cnt = 0;
							  usart_send_flag = 1;
						}					
						temperature = DS18B20_Get_Temp();//读取温度
						if(temperature<0)
						{
							LCD_Write_Char(12,0,'-');			//显示负号
							temperature=-temperature;					//转为正数
						}else LCD_Write_Char(12,0,' ');			//去掉负号
						//显示温度
						LCD_Write_Char(12,0,temperature/100+'0');
						LCD_Write_Char(13,0,temperature%100/10+'0');
						
						send_data[15] =temperature/100+'0';
					  send_data[16] =temperature/100+'0';
						
	
						Get_PH() ;
						adcx = Get_Adc_Average(ADC_Channel_9,20);//读取通道9的AD值，20MS读取一次
						//***********************计算水浊度****************************//
						T = adcx;
						T = T*(3.3/4096)+1.72;
						if(T < 2.5)
						{
								T = 3000;	
						}
						else 
						{
								//Tul=-0.43*Tul*Tul+112.6*Tul-85.35;
								T = (-1120.4*T*T+5742.3*T-4352.9);	//Tul是AD值
						}
						if(T < 0)
						{
							T = 0;
						}
						adcx = (u16)T;
						//显示水浊度
						LCD_Write_Char(5,1, adcx/1000%10+0x30);
						LCD_Write_Char(6,1, adcx/100%10+0x30);
						LCD_Write_Char(7,1, adcx/10%10+0x30);
						LCD_Write_Char(8,1, adcx%10+0x30);
						
					  send_data[5] =adcx/1000%10+0x30;
					  send_data[6] = adcx/100%10+0x30;
					  send_data[7] = adcx/10%10+0x30;
						send_data[8] = adcx%10+0x30;
						//PH
						PH1=((int)PH/100+'0');
						PH2=((int)PH%100/10+'0');
						
						LCD_Write_Char(12,1,PH1);
						LCD_Write_String(13,1,(u8*)".");
						LCD_Write_Char(14,1,PH2);
						
						
						if(adcx >= Wrtb_Max || temperature/10 >= T_max)//超限蜂鸣器报警
						{
								BEEP = 1;
								BJ_LED =1;
						}
						else
						{
								BEEP = 0;
								BJ_LED =0;
						}
						
						if(!KEY4)//减键
						{
						   delay_ms(10);
							 if(!KEY4)
							 {
									biaozhiwei=!biaozhiwei;
									while(!KEY4);//等待按键松开
								}
						}
						if( biaozhiwei==0||led==10)
						{
						   juli=0;
						}
						if( biaozhiwei==1||led==11)
						{
						    juli=1;
						}
						if(fen==fen_flag&&shi==shi_flag)//计时时间到报警
		          {
							BEEP=1;
							delay_ms(2000);
							BEEP=0;
		          MotorCW();
				      fen =0;
				      shi=0;
							TIM_Cmd(TIM3,ENABLE);  //使能TIMx	
		          }	
				}
		}
}


