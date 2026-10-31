#include "stm32f10x.h"                
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include <math.h>


uint8_t ID;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义加速度
float AX_g,AY_g,AZ_g;		//定义重力加速度
float ThetaX,ThetaY;		//定义倾角
float S1_Angle,S2_Angle;		//定义舵机对应角
float S1_Filtered,S2_Filtered;		//滤波后舵机对应角

static void Sensor_Update(void)
{
	MPU6050_GetData(&AX, &AY, &AZ, &GX, &GY, &GZ);		//获取MPU6050的数据
											//转化重力加速度
	AX_g = (float)AX * 4/65535;
	AY_g = (float)AY * 4/65535;
	AZ_g = (float)AZ * 4/65535;
	
											//转化倾角
	ThetaX = atan2f(AX_g,AZ_g)* 57.29578f;
	ThetaY = atan2f(AY_g,AZ_g)* 57.29578f;
	
	//规定倾角范围
	if(ThetaX < -30.0f){
			ThetaX = -30.0f;}
	if(ThetaX > 30.0f){
			ThetaX = 30.0f;}
	if(ThetaY < -30.0f){
			ThetaY = -30.0f;}
	if(ThetaY > 30.0f){
			ThetaY = 30.0f;}
	
											//将传感器倾角转化为舵机角度
	S1_Angle = 3*(ThetaX + 30.0f);
	S2_Angle = 3*(ThetaY + 30.0f);
}

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
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	Serial_Init();		//串口初始化

	Delay_ms(50);		//等待传感器唤起

	Sensor_Update();	//换算参数
	//滤波器初始化
	S1_Filtered = S1_Angle;
	S2_Filtered = S2_Angle;
	
	/*显示ID号*/
	OLED_ShowString(0, 0, "ID:",OLED_8X16);		//显示静态字符串
	OLED_ShowString(40, 24, "TX:",OLED_8X16);		//显示静态字符串
	OLED_ShowString(40, 48, "TY:",OLED_8X16);		//显示静态字符串
	ID = MPU6050_GetID();				//获取MPU6050的ID号
	
    OLED_Update();
	
	while (1)
	{
		
		Sensor_Update();	//换算参数
	
		//一阶低通滤波
		S1_Filtered += 0.3f*(S1_Angle - S1_Filtered);
		S2_Filtered += 0.3f*(S2_Angle - S2_Filtered);
		
		//发送数据
		Bluetooth_Send_Angle();
		
		//显示数据,16ms刷新一次
		static uint8_t Showagain;
		if(Showagain < 2)
		{
			OLED_ShowHexNum(30, 0, ID, 2,OLED_8X16);		//OLED显示ID号
			if (ID != 0x74) {
			OLED_ShowString(0, 16, "MPU6050 ERROR", OLED_8X16);} 	//如果I2C数据出错，显示error
			OLED_ShowSignedNum(70,24, S1_Filtered, 5,OLED_8X16);	//OLED显示数据
			OLED_ShowSignedNum(70,48, S2_Filtered, 5,OLED_8X16);
			OLED_Update();
			Showagain++;
		}
		else{
			Showagain = 0;
		}
		//主函数8ms发送一次数据
		Delay_ms(8);
		
	}
}
