/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//用于做各种简易计数器计数
short fric_flag = 0;//摩擦轮电机初始化标志
RobotInit_Struct Infantry;
char Robot_ID;
char Judge_Lost;
extern short FrictionWheel_speed;
extern short KalMan_doneflag;
/**********************************************************************************************************
*函 数 名: main
*功能说明: 主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/

int main()
{

	BSP_Init();
	delay_ms(3000);
	Robot_ID=3;
	Robot_Init();
	startTast();
  vTaskStartScheduler();
	
	while(1)
	{

	}
}
	

/**********************************************************************************************************
*函 数 名: System_Config
*功能说明: 系统初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BSP_Init(void)
{
	USART3_Configuration();
	delay_ms(100);
	USART6_Configuration();
	delay_ms(100);
	My_GPIO_Init();	//用于辅瞄做相机同步
	delay_ms(100);
	MicroSwConfigration();
	delay_ms(100);
  SteeringEngine_Configuration();
	delay_ms(100);
	TIM7_Configuration();
	delay_ms(100);
	CAN1_Configuration();
	delay_ms(100);
	CAN2_Configuration();
	delay_ms(100);
	IWDG_Config(IWDG_Prescaler_128 ,625);
	delay_ms(100);
	VOFA_USART_Configuration();
}
/**********************************************************************************************************
*函 数 名: System_Init
*功能说明: 系统参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Robot_Init(void)
{
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
  PidGimbalMotor_Init();
	Pid_BodanMotor_Init();
	Pid_Friction_Init();
  FuzzyPidGimbalMotor_Init();
	
//	KalmanFilter_Init(&pitch_Kalman, &K);
//	KalmanFilter_Init(&yaw_Kalman, &K);//PC先传数据再运行STM32会卡在arm32文件内(未知)
//	KalMan_doneflag = 1;
}

/**********************************************************************************************************
*函 数 名: Infantry_Init
*功能说明: 步兵参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Infantry_Init(void)
{	
///*舵轮*/	
//	Infantry.Yaw_init=1040; 
//	Infantry.Pitch_init=5300;
//	Infantry.MagOpen=1000;
//	Infantry.MagClose=2350;
//	Infantry.Solo_Yaw_init = 20;
	
/*自适应3号*/
switch(Robot_ID)
{	
/***************************************** 3 号车 **************************************************************/	
	case 3:
{	
	Infantry.Yaw_init=5445;                  //  3号车 
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2000;
	Infantry.MagClose=540;
	Infantry.Solo_Yaw_init = 20;
} break;

/***************************************** 4 号车 **************************************************************/	
	case 4:
{	
	Infantry.Yaw_init=6873;            // 4号车
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2400;
	Infantry.MagClose=600;
	Infantry.Solo_Yaw_init = 20;
} break;

/***************************************** 5 号车 **************************************************************/	
	case 5:
{	
	Infantry.Yaw_init=6873;            //  5号车
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2400;
	Infantry.MagClose=600;
	Infantry.Solo_Yaw_init = 20;
} break;
/**************************************************************************************************************/
	default:
{
  Infantry.Yaw_init=5445;                  // 信号丢失 
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2000;
	Infantry.MagClose=540;
	Infantry.Solo_Yaw_init = 20;
}
}
}

/**********************************************************************************************************
*函 数 名: delay_ms
*功能说明: ms延时
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void delay_ms(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=10300;
 		while(a--);
	}
}
void delay_us(unsigned long t)
{
	int i;
	for( i=0;i<t;i++)
	{
		int a=37;
 		while(a--);
	}
}
/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern Disconnect Robot_Disconnect;
void Offline_Check_task(void *pvParameters)
{
   while (1) {

		/*电机\IMU掉线检测*/
	if(Robot_Disconnect.YawMotor_DisConnect>10||Robot_Disconnect.PitchMotor_DisConnect>10||Robot_Disconnect.Gyro_DisConnect)
		{
	    Robot_Stop();
		}
	else
		{
			Robot_Recover();
		}
	Robot_Disconnect.Gyro_DisConnect++;
	Robot_Disconnect.PitchMotor_DisConnect++;
	Robot_Disconnect.YawMotor_DisConnect++;
	
	 /*发射机构掉线 */
	 if(Robot_Disconnect.Friction_DisConnect[0]>10||Robot_Disconnect.Friction_DisConnect[1]>10||Robot_Disconnect.Pluck_DisConnect>10)
		{
		 Shoot_Stop();
		}
		else
		{
		 Shoot_Recover();
		}
	Robot_Disconnect.Friction_DisConnect[0]++;
	Robot_Disconnect.Friction_DisConnect[1]++;
	Robot_Disconnect.Pluck_DisConnect++;
		
	/*遥控器掉线检测*/
	if(Robot_Disconnect.RC_DisConnect>10)
		{
			RC_Rst();
		}
	Robot_Disconnect.RC_DisConnect++;
		
	/*底盘板或者裁判系统掉线检测*/
	if(Robot_Disconnect.F105_DisConect>15||Judge_Lost==1)
		{
		F105_Rst();
		}
	Robot_Disconnect.F105_DisConect++;
	
	IWDG_Feed();	 
  vTaskDelay(5); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



