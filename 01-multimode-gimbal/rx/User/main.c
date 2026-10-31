#include "stm32f10x.h"                
#include "Delay.h"
#include "OLED.h"
#include "MPU6050.h"
#include "Serial.h"
#include "Servo.h"
#include "LED.h"
#include <math.h>

//定义步长
#define Angle_Step_Small		0.5f
#define Angle_Step_Large		6.0f
#define Angle_Small				5.0f
#define Angle_Large			   15.0f
#define Angle_Deadzone          0.5f
// 系统工作模式定义
#define Mode_MPU 0    // MPU6050姿态控制舵机模式
#define Mode_POT 1    // 电位器调节舵机模式
#define Mode_DEVICE 2      //红外蜂鸣器外设控制模式

static float S1_Target,S2_Target;		//滤波后舵机目标角
static float S1_Actual,S2_Actual;		//舵机实际角
static uint8_t FirstFrame = 1;

//接收数据解析
static void Parse_DualAngle()		
{
	uint16_t Rx_Int1,Rx_Int2;
	Rx_Int1 = Serial_RxPacket[0] | (Serial_RxPacket[1] << 8);
	Rx_Int2 = Serial_RxPacket[2] | (Serial_RxPacket[3] << 8);
	S1_Target = (float)Rx_Int1/100.0f;
	S2_Target = (float)Rx_Int2/100.0f;
	//范围保护
	if(S1_Target < 0.0f){
			S1_Target = 0.0f;}
	if(S1_Target > 180.0f){
			S1_Target = 180.0f;}
	if(S2_Target < 0.0f){
			S2_Target = 0.0f;}
	if(S2_Target > 180.0f){
			S2_Target = 180.0f;}
	
	if (FirstFrame) {
        S1_Actual = S1_Target;      // 首个目标直接对齐，不扫动 
        S2_Actual = S2_Target;
        FirstFrame = 0;
    }
}

//步长判断
static float DecideStep(float Diff)
{
	if(Diff < Angle_Small){
		return Angle_Step_Small;}
	else if(Diff > Angle_Large){
		return Angle_Step_Large;}
	else
	{
		return Angle_Step_Small + (Diff - Angle_Small)*(Angle_Step_Large - Angle_Step_Small)/(Angle_Large -Angle_Small);
	}	
}

//舵机动态步长
static void Servo_SmoothControl(void)
{
	float Diff1 = S1_Target - S1_Actual;
	float Step1 = DecideStep(fabs(Diff1));
	if(fabs(Diff1)>Angle_Deadzone)		//死区
	{
		if(Diff1 > Step1){
		S1_Actual += Step1;}
		else if(Diff1 < -Step1){
		S1_Actual -= Step1;}
		else{
		S1_Actual = S1_Target;}
	}	
	Servo_SetAngle1(S1_Actual);
	
	//同理
	
	float Diff2 = S2_Target - S2_Actual;
	float Step2 = DecideStep(fabs(Diff2));
	if(fabs(Diff2)>Angle_Deadzone)
	{
		if(Diff2 > Step2){
		S2_Actual += Step2;}
		else if(Diff2 < -Step2){
		S2_Actual -= Step2;}
		else{
		S2_Actual = S2_Target;}
	}	
	Servo_SetAngle2(S2_Actual);
}

////MPU、电位器模式解析
//void Parse_ServoFrame(void)
//{
//	Parse_DualAngle();	//数据包解析
//	Servo_SmoothControl();	//舵机平滑控制

//}

//外设模式解析
//void Parse_DeviceFrame()

//LED随模式更新
void LED_Updatemode(void)
{
	LED1_OFF();
	LED2_OFF();
	LED3_OFF();
	switch (CurrentFrameMode)
	{
		case Mode_MPU:
			LED1_ON();
			break;
		case Mode_POT:
			LED2_ON();
			break;
		case Mode_DEVICE:
			LED3_ON();
			break;
			
	}	
}

int main(void)
{
	/*模块初始化*/
	OLED_Init();		//OLED初始化
	Serial_Init();		//串口初始化
	Servo_Init();		//舵机初始化
	LED_Init();
	Delay_ms(50);		//等待传感器唤起
	// 初始显示
    OLED_ShowString(25, 0, "Servo Ctrl", OLED_8X16);
    OLED_ShowString(22, 24, "Mode:Servo", OLED_8X16);
    OLED_ShowString(15, 48, "A:", OLED_8X16);
    OLED_ShowString(65, 48, "B:", OLED_8X16);
    OLED_Update();
	
	while (1)
	{
		
		//接收数据
		 if (Serial_GetRxFlag() == 1)
		{
			if(CurrentFrameMode == Mode_MPU)
			{
				Parse_DualAngle();
			}
			else if(CurrentFrameMode == Mode_POT)
			{
				Parse_DualAngle();
			}
//		else if(CurrentFrameMode == Mode_DEVICE)
//		{
//			Parse_DeviceFrame();
//		}
			Serial_RxFlag = 0; 
		}
		LED_Updatemode();
		// MPU姿态控制模式逻辑
        if (CurrentFrameMode == Mode_MPU) { 
			OLED_Clear();
            Servo_SmoothControl();                              // 舵机平滑追位
            OLED_ClearArea(0, 48, 128, 16);                     // 清空角度显示区域
			OLED_ShowString(25, 0, "Servo Ctrl", OLED_8X16);
            OLED_ShowString(22, 24, "Mode:Mpu", OLED_8X16);   // 显示当前模式
            OLED_ShowString(15, 48, "A:", OLED_8X16);           // 舵机1标签
            OLED_ShowString(65, 48, "B:", OLED_8X16);           // 舵机2标签
            OLED_ShowNum(35, 48, (uint16_t)S1_Actual, 3, OLED_8X16); // 显示舵机1角度
            OLED_ShowNum(85, 48, (uint16_t)S2_Actual, 3, OLED_8X16); // 显示舵机2角度
        } 
		// 电位器控制模式逻辑
        else if (CurrentFrameMode == Mode_POT) {
			OLED_Clear();		 
            Servo_SmoothControl();        
					// 舵机平滑追位
			OLED_ShowString(25, 0, "Servo Ctrl", OLED_8X16);
            OLED_ClearArea(0, 48, 128, 16);                     // 清空角度显示区域
            OLED_ShowString(30, 24, "Mode: Pot", OLED_8X16);     // 显示当前模式
            OLED_ShowString(15, 48, "A:", OLED_8X16);           // 舵机1标签
            OLED_ShowString(65, 48, "B:", OLED_8X16);           // 舵机2标签
            OLED_ShowNum(35, 48, (uint16_t)S1_Actual, 3, OLED_8X16); // 显示舵机1角度
            OLED_ShowNum(85, 48, (uint16_t)S2_Actual, 3, OLED_8X16); // 显示舵机2角度
        }
		OLED_Update();  // 刷新OLED显示
		//主函数8ms发送一次数据
		Delay_ms(8);
		
	}
}
