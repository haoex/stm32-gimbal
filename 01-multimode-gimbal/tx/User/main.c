#include "stm32f10x.h"                
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include <math.h>
#include "Key.h"
#include "AD.h"

//工作模式定义
#define Mode_MPU	0
#define Mode_POT	1
#define Filter_MPU		0.3f
#define Filter_POT		0.6f

uint8_t ID;								//定义用于存放ID号的变量
int16_t AX, AY, AZ, GX, GY, GZ;			//定义加速度
float AX_g,AY_g,AZ_g;		//定义重力加速度
float ThetaX,ThetaY;		//定义倾角
float S1_Angle_MPU,S2_Angle_MPU;		//定义舵机对应角
float S1_Filtered_MPU,S2_Filtered_MPU;		//滤波后舵机对应角
float S1_Angle_POT,S2_Angle_POT;		//定义舵机对应角
float S1_Filtered_POT,S2_Filtered_POT;		//滤波后舵机对应角

uint8_t currentMode,lastMode;
uint8_t lastMode = 0x01;
uint16_t AD1,AD2;




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
	S1_Angle_MPU = 3*(ThetaX + 30.0f);
	S2_Angle_MPU = 3*(ThetaY + 30.0f);
}

//打包发送角度帧
static void Bluetooth_Send_Angle(float S1_Filtered,float S2_Filtered)
{
	uint8_t ArrayDu[32];
	uint16_t S1_Int = (uint16_t)(S1_Filtered*100);
	uint16_t S2_Int = (uint16_t)(S2_Filtered*100);
	//打包成数组
	ArrayDu[0] = 0xAA;                                  /* 帧头1 */
    ArrayDu[1] = 0x55;                                  /* 帧头2 */
    ArrayDu[2] = 4;                                     /* LEN = 数据字节数 */
    ArrayDu[3] = currentMode;                           /* 模式（放在 CMD 位置）*/
	ArrayDu[4] = (uint8_t)S1_Int;
	ArrayDu[5] = (uint8_t)(S1_Int >> 8);
	ArrayDu[6] = (uint8_t)S2_Int;
	ArrayDu[7] = (uint8_t)(S2_Int >> 8);

	Serial_SendArray(ArrayDu,8);

}


//打包发送外设帧
//static void Bluetooth_Send_Device(float S1_Filtered,float S2_Filtered)

//打包发送角度帧
//static void Bluetooth_Send_Vision(float S1_Filtered,float S2_Filtered)
	
//OLED按模式发送函数
void OLED_Mode(void)
{
	static uint8_t Showagain;
	if(Showagain < 2)
	{
		if(currentMode == Mode_MPU)
		{
			OLED_ShowString(32, 0, "MPU Mode", OLED_8X16);
			OLED_ShowString(27, 16, "MPU ID:", OLED_8X16);
			OLED_ShowHexNum(87, 16, ID, 2, OLED_8X16);
			OLED_ShowString(45, 32, "X:", OLED_8X16);
			OLED_ShowString(45, 48, "Y:", OLED_8X16);
			OLED_ShowNum(65, 32, (uint16_t)S1_Filtered_MPU, 3, OLED_8X16);
			OLED_ShowNum(65, 48, (uint16_t)S2_Filtered_MPU, 3, OLED_8X16);
		}
		else if(currentMode == Mode_POT)
		{
			OLED_ShowString(33, 0, "Pot Mode", OLED_8X16);
			OLED_ShowString(38, 24, "S1:", OLED_8X16);
			OLED_ShowString(38, 48, "S2:", OLED_8X16);
			OLED_ShowNum(70, 24, (uint16_t)S1_Filtered_POT, 3, OLED_8X16);
			OLED_ShowNum(70, 48, (uint16_t)S2_Filtered_POT, 3, OLED_8X16);	
		}
			OLED_Update();
			Showagain++;
	}
	else{
			Showagain = 0;
		}
}

//模式更换刷新
void Mode_ResetState(void)
{
	if(currentMode != lastMode)
	{
		if(currentMode == Mode_MPU)
		{
			Sensor_Update();	//换算参数
			//滤波器初始化
			S1_Filtered_MPU = S1_Angle_MPU;
			S2_Filtered_MPU = S2_Angle_MPU;
		
		}
		else if(currentMode == Mode_POT)
		{
			S1_Angle_POT = (float)AD1 / 4095 * 180.0f;		//将AD值线性变换到0~180的范围，表示舵机角度
			S2_Angle_POT = (float)AD2 / 4095 * 180.0f;
			//一阶低通滤波
			S1_Filtered_POT += Filter_POT*(S1_Angle_POT - S1_Filtered_POT);
			S2_Filtered_POT += Filter_POT*(S2_Angle_POT - S2_Filtered_POT);
			//滤波器初始化
			S1_Filtered_POT = S1_Angle_POT;
			S2_Filtered_POT = S2_Angle_POT;
		}
		OLED_Clear();
	}
	lastMode = currentMode;
}

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	MPU6050_Init();		//MPU6050初始化
	Serial_Init();		//串口初始化
	Key_Init();
	AD_Init();				//AD初始化
	Delay_ms(50);		//等待传感器唤起

	
	ID = MPU6050_GetID();
	
	currentMode = Mode_MPU;
	Mode_ResetState();
	OLED_Mode();
	
	while (1)
	{
		uint8_t Key = Key_GetNum();
		if(Key == 1)
		{
			currentMode = Mode_MPU;
		}
		else if(Key == 2)
		{
			currentMode = Mode_POT;
		}
		//模式更换刷新
		Mode_ResetState();
		//MPU mode
		if(currentMode == Mode_MPU)
		{
			Sensor_Update();	//换算参数
			//一阶低通滤波
			S1_Filtered_MPU += Filter_MPU*(S1_Angle_MPU - S1_Filtered_MPU);
			S2_Filtered_MPU += Filter_MPU*(S2_Angle_MPU - S2_Filtered_MPU);
			//发送数据
			Bluetooth_Send_Angle(S1_Filtered_MPU,S2_Filtered_MPU);
		}
		//POT mode
		else if(currentMode == Mode_POT)
		{
			AD1 = AD_GetValue(ADC_Channel_1);		//获取AD转换的值
			AD2 = AD_GetValue(ADC_Channel_2);
		
			S1_Angle_POT = (float)AD1 / 4095 * 180.0f;		//将AD值线性变换到0~180的范围，表示舵机角度
			S2_Angle_POT = (float)AD2 / 4095 * 180.0f;
			//一阶低通滤波
			S1_Filtered_POT += Filter_POT*(S1_Angle_POT - S1_Filtered_POT);
			S2_Filtered_POT += Filter_POT*(S2_Angle_POT - S2_Filtered_POT);
			//发送数据
			Bluetooth_Send_Angle(S1_Filtered_POT,S2_Filtered_POT);
		}
		
		//显示数据,16ms刷新一次
		OLED_Mode();
		//主函数8ms发送一次数据
		Delay_ms(8);
		
	}
}
