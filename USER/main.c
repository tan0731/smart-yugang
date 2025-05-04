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

int miao_flag = 0, fen_flag = 1, shi_flag = 0;
bool usart_send_flag = 0;
int PH1, PH2;
extern int miao, fen, shi;
short temperature = 0; // �¶�
u8 setN = 0;		   // ���ñ�־
u16 Wrtb_Max = 3000;   // ˮ�Ƕȱ���ֵ
u8 T_max = 40;		   // �¶ȱ���ֵ
u8 led = 0;
float PH = 0.0;
char send_data[] = "Wrtb:0000,Temp:00.0C\r\n";
unsigned int cnt;
float T;
char buffer[10];
unsigned char Alarm_Buf[16] = "Warning";
char buf[30];
long int adcx = 0;
unsigned int adc = 0;
unsigned char setn = 0;

void che_Init()
{

	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); // ʹ��PB�˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; // ����IO��
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; // IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void KEY_SCAN(void) // ����ɨ��
{
	if (!KEY1) // ���ü�
	{
		delay_ms(10); // ����
		if (!KEY1)
		{
			while (!KEY1)
				; // �ȴ������ɿ�
			setn++;
			if (setn > 2)
			{
				setn = 0;
				LCD_Write_String(0, 0, "Temperature:   C");
				LCD_Write_String(0, 1, "Wrtb:    ");
				LCD_Write_String(9, 1, (u8 *)"PH:  ");
			}
			if (setn == 1)
			{
				LCD_Clear();
				LCD_Write_String(0, 0, "====Set Temp====");

				LCD_Write_Char(7, 1, T_max / 10 + 0x30); // �¶�����
				LCD_Write_Char(8, 1, T_max % 10 + 0x30);
				LCD_Write_Char(9, 1, 'C');
			}
			if (setn == 2)
			{
				LCD_Write_String(0, 0, "====Set Wrtb====");
				LCD_Write_String(0, 1, "       00%      ");
				LCD_Write_Char(6, 1, Wrtb_Max / 1000 % 10 + 0x30);
				LCD_Write_Char(7, 1, Wrtb_Max / 100 % 10 + 0x30);
				LCD_Write_Char(8, 1, Wrtb_Max / 10 % 10 + 0x30);
				LCD_Write_Char(9, 1, Wrtb_Max % 10 + 0x30);
			}
		}
	}
	if (!KEY2) // �Ӽ�
	{
		delay_ms(10);
		if (!KEY2)
		{
			while (!KEY2)
				; // �ȴ������ɿ�
			if (setn == 1)
			{
				if (T_max < 99)
					T_max++;
				LCD_Write_Char(7, 1, '0' + T_max / 10);
				LCD_Write_Char(8, 1, '0' + T_max % 10);
			}
			if (setn == 2)
			{
				if (Wrtb_Max < 99)
					Wrtb_Max++;
				if (Wrtb_Max < 3500)
					Wrtb_Max = Wrtb_Max + 10;

				LCD_Write_Char(6, 1, Wrtb_Max / 1000 % 10 + 0x30);
				LCD_Write_Char(7, 1, Wrtb_Max / 100 % 10 + 0x30);
				LCD_Write_Char(8, 1, Wrtb_Max / 10 % 10 + 0x30);
				LCD_Write_Char(9, 1, Wrtb_Max % 10 + 0x30);
			}
		}
	}
	if (!KEY3) // ����
	{
		delay_ms(10);
		if (!KEY3)
		{
			while (!KEY3)
				; // �ȴ������ɿ�
			if (setn == 1)
			{
				if (T_max > 0)
					T_max--;
				LCD_Write_Char(7, 1, '0' + T_max / 10);
				LCD_Write_Char(8, 1, '0' + T_max % 10);
			}
			if (setn == 2)
			{
				if (Wrtb_Max > 0)
					Wrtb_Max = Wrtb_Max - 10;

				LCD_Write_Char(6, 1, Wrtb_Max / 1000 % 10 + 0x30);
				LCD_Write_Char(7, 1, Wrtb_Max / 100 % 10 + 0x30);
				LCD_Write_Char(8, 1, Wrtb_Max / 10 % 10 + 0x30);
				LCD_Write_Char(9, 1, Wrtb_Max % 10 + 0x30);
			}
		}
	}
}

u8 biaozhiwei = 0;
void Get_PH(void) // ��ȡPH
{
	float ph_ad;

	ph_ad = Get_Adc_Average(ADC_Channel_8, 20); // ȡ20��ƽ��ֵ
	PH = (ph_ad * 3.3 / 4096);
	PH = PH * 5.9647;
	PH = 22.255 - PH;
	PH = PH * 100;
	if (PH > 1400)
		PH = 1400;
}
int main(void)
{
	u8 count = 0;
	delay_init();			   // ��ʱ������ʼ��
	NVIC_Configuration();	   // ����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	TIM3_Int_Init(9999, 7199); // 10Khz�ļ���Ƶ�ʣ�������10000Ϊ1000msΪ1s
	LCD_Init();				   // 1602��ʼ��
	Adc_Init();				   // ADC��ʼ��
	fengmingqi_Init();		   // ������ʼ��
	RELAY_Init();			   // �̵�����ʼ��
	BEEP_GPIO_Config();		   // ��������ʼ��
	KEY_IO_Init();
	MOTOR_GPIO_Init(); // �����ʼ��
	che_Init();

	ESP8266_Init();
	while (DS18B20_Init()) // ds18b20��ʼ��
	{
		LCD_Write_String(0, 0, (u8 *)"  DS18B20 Erro  ");
		LCD_Write_String(0, 1, "                ");
	}
	DS18B20_Get_Temp(); // ��ȡ�¶�
	LCD_Write_String(0, 0, "Temperature:   C");
	LCD_Write_String(0, 1, (u8 *)"Wrtb:    ");
	LCD_Write_String(9, 1, (u8 *)"PH:");
	TIM_Cmd(TIM3, ENABLE); // ʹ��TIMx
	while (1)
	{
		KEY_SCAN();
		if (count++ >= 100 && setn == 0)
		{
			count = 0;

			if (cnt++ > 10)
			{
				cnt = 0;
				usart_send_flag = 1;
			}
			temperature = DS18B20_Get_Temp(); // ��ȡ�¶�
			if (temperature < 0)
			{
				LCD_Write_Char(12, 0, '-'); // ��ʾ����
				temperature = -temperature; // תΪ����
			}
			else
				LCD_Write_Char(12, 0, ' '); // ȥ������
			// ��ʾ�¶�
			LCD_Write_Char(12, 0, temperature / 100 + '0');
			LCD_Write_Char(13, 0, temperature % 100 / 10 + '0');

			send_data[15] = temperature / 100 + '0';
			send_data[16] = temperature / 100 + '0';

			Get_PH();
			adcx = Get_Adc_Average(ADC_Channel_9, 20); // ��ȡͨ��9��ADֵ��20MS��ȡһ��
			//***********************����ˮ�Ƕ�****************************//
			T = adcx;
			T = T * (3.3 / 4096) + 1.72;
			if (T < 2.5)
			{
				T = 3000;
			}
			else
			{
				// Tul=-0.43*Tul*Tul+112.6*Tul-85.35;
				T = (-1120.4 * T * T + 5742.3 * T - 4352.9); // Tul��ADֵ
			}
			if (T < 0)
			{
				T = 0;
			}
			adcx = (u16)T;
			// ��ʾˮ�Ƕ�
			LCD_Write_Char(5, 1, adcx / 1000 % 10 + 0x30);
			LCD_Write_Char(6, 1, adcx / 100 % 10 + 0x30);
			LCD_Write_Char(7, 1, adcx / 10 % 10 + 0x30);
			LCD_Write_Char(8, 1, adcx % 10 + 0x30);

			send_data[5] = adcx / 1000 % 10 + 0x30;
			send_data[6] = adcx / 100 % 10 + 0x30;
			send_data[7] = adcx / 10 % 10 + 0x30;
			send_data[8] = adcx % 10 + 0x30;
			// PH
			PH1 = ((int)PH / 100 + '0');
			PH2 = ((int)PH % 100 / 10 + '0');

			LCD_Write_Char(12, 1, PH1);
			LCD_Write_String(13, 1, (u8 *)".");
			LCD_Write_Char(14, 1, PH2);

			if (adcx >= Wrtb_Max || temperature / 10 >= T_max) // ���޷���������
			{
				BEEP = 1;
				BJ_LED = 1;
			}
			else
			{
				BEEP = 0;
				BJ_LED = 0;
			}

			if (!KEY4) // ����
			{
				delay_ms(10);
				if (!KEY4)
				{
					biaozhiwei = !biaozhiwei;
					while (!KEY4)
						; // �ȴ������ɿ�
				}
			}
			if (biaozhiwei == 0 || led == 10)
			{
				juli = 0;
			}
			if (biaozhiwei == 1 || led == 11)
			{
				juli = 1;
			}
			if (fen == fen_flag && shi == shi_flag) // ��ʱʱ�䵽����
			{
				BEEP = 1;
				delay_ms(2000);
				BEEP = 0;
				MotorCW();
				fen = 0;
				shi = 0;
				TIM_Cmd(TIM3, ENABLE); // ʹ��TIMx
			}
		}
	}
}
