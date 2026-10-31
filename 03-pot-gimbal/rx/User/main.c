#include "stm32f10x.h"               
#include "Delay.h"
#include "OLED.h"
#include "Serial.h"
#include "Servo.h"
#include <math.h>

//定义步长
#define Angle_Step_Small		0.5f
#define Angle_Step_Large		6.0f
#define Angle_Small				5.0f
#define Angle_Large			   15.0f
#define Angle_Deadzone          0.5f

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

int main(void)
{
	/*模块初始化*/
	OLED_Init();			//OLED初始化
	Serial_Init();			//串口初始化
	Servo_Init();		//舵机初始化
	Delay_ms(50);
	/*显示静态字符串*/
	OLED_ShowString(00, 10, "RX:",OLED_8X16);
	OLED_ShowString(00, 30, "RY:",OLED_8X16);
	OLED_Update();
	while (1)
	{
		//接收数据
		 if (Serial_GetRxFlag() == 1)
    {
        Parse_DualAngle();
    }
		//写入舵机
		Servo_SmoothControl();
	
		OLED_ShowNum(30, 10 , (uint32_t)S1_Actual, 4 , OLED_8X16);				
		OLED_ShowNum(30, 30 , (uint32_t)S2_Actual , 3 , OLED_8X16);			
		OLED_Update();

		Delay_ms(10);			//延时10ms，手动增加一些转换的间隔时间
	}
}
