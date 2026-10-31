#include "stm32f10x.h"               
#include "Delay.h"
#include "OLED.h"
#include "AD.h"
#include "Serial.h"

uint16_t AD1,AD2;			//定义AD值变量
float S1_Angle,S2_Angle;				//定义电压变量
float S1_Filtered,S2_Filtered;


static void Bluetooth_Send_Angle(void)
{
	uint16_t S1_Int = (uint16_t)(S1_Filtered*100);
	uint16_t S2_Int = (uint16_t)(S2_Filtered*100);
	//打包成数组
	uint8_t ArrayDu[6] = {
		0xFF,						//帧头
		(uint8_t)S1_Int,			//低八位
		(uint8_t)(S1_Int >> 8),		//高八位
		(uint8_t)S2_Int,			//低八位
		(uint8_t)(S2_Int >> 8),		//高八位
		0xFE						//帧尾
	};
	Serial_SendArray(ArrayDu,6);

}	

int main(void)
{
	/*模块初始化*/
	OLED_Init();			//OLED初始化
	AD_Init();				//AD初始化
	Serial_Init();			//串口初始化
	Delay_ms(50);
	/*显示静态字符串*/
	OLED_ShowString(00, 10, "AD1:",OLED_8X16);
	OLED_ShowString(72, 10, "A1:",OLED_8X16);
	OLED_ShowString(00, 30, "AD2:",OLED_8X16);
	OLED_ShowString(72, 30, "A2:",OLED_8X16);
	OLED_Update();
	while (1)
	{
		AD1 = AD_GetValue(ADC_Channel_2);		//获取AD转换的值
		AD2 = AD_GetValue(ADC_Channel_3);
		
		S1_Angle = (float)AD1 / 4095 * 180.0f;		//将AD值线性变换到0~180的范围，表示舵机角度
		S2_Angle = (float)AD2 / 4095 * 180.0f;
		
		//一阶低通滤波
		S1_Filtered += 0.3f*(S1_Angle - S1_Filtered);
		S2_Filtered += 0.3f*(S2_Angle - S2_Filtered);
		
		
		//发送数据
		Bluetooth_Send_Angle();
		

		OLED_ShowNum(36, 10 , AD1, 4 , OLED_8X16);				
		OLED_ShowNum(96, 10 , (uint32_t)S1_Filtered , 3 , OLED_8X16);			
		OLED_ShowNum(36, 30 , AD2, 4 , OLED_8X16);				
		OLED_ShowNum(96, 30 , (uint32_t)S1_Filtered , 3 , OLED_8X16);	
		OLED_Update();

		Delay_ms(10);			//延时10ms，手动增加一些转换的间隔时间
	}
}
