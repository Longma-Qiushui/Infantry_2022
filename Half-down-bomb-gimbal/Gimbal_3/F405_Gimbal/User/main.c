/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"

char Robot_ID = 3;         // ��ͬ�������ò�ͬ�����

extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//���������ּ��׼���������
short fric_flag = 0;//Ħ���ֵ����ʼ����־
RobotInit_Struct Infantry;
char Judge_Lost;
extern short FrictionWheel_speed;
extern short KalMan_doneflag;
/**********************************************************************************************************
*�� �� ��: main
*����˵��: ������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

int main()
{

	BSP_Init();
	delay_ms(3000);
	SteeringEngine_Set(Infantry.MagOpen);
	Robot_Init();
	startTast();
  vTaskStartScheduler();
	
	while(1)
	{
	
	}
}
	

/**********************************************************************************************************
*�� �� ��: System_Config
*����˵��: ϵͳ��ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BSP_Init(void)
{
	USART1_Configuration();
	delay_ms(100);
	USART2_Configuration();
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
//	VOFA_USART_Configuration();
}
/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
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
//	KalmanFilter_Init(&yaw_Kalman, &K);//PC�ȴ�����������STM32�Ῠ��arm32�ļ���(δ֪)
//	KalMan_doneflag = 1;
}

/**********************************************************************************************************
*�� �� ��: Infantry_Init
*����˵��: ����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Infantry_Init(void)
{	
///*����*/	
//	Infantry.Yaw_init=1040; 
//	Infantry.Pitch_init=5300;
//	Infantry.MagOpen=1000;
//	Infantry.MagClose=2350;
//	Infantry.Solo_Yaw_init = 20;
	
/*����Ӧ3��*/
switch(Robot_ID)
{	
/***************************************** 3 �ų� **************************************************************/	
	case 3:
{	
	Infantry.Yaw_init=2750;                  //  3�ų� 
	Infantry.Pitch_init=7440;
	Infantry.MagOpen=2400;
	Infantry.MagClose=530;
	Infantry.Solo_Yaw_init = 7715;
	Infantry.PitchMotorID = 0x206;
	Infantry.YawMotorID = 0x205;
	Infantry.FricMotorID[0]=0x202;
	Infantry.FricMotorID[1]=0x203;
	Infantry.BodanMotorID=0x201;
  Infantry.pn=-1;
	
} break;

/***************************************** 4 �ų� **************************************************************/	
	case 4:
{	
	Infantry.Yaw_init=6865;            // 4�ų�
	Infantry.Pitch_init=4695;
	Infantry.MagOpen=1500;
	Infantry.MagClose=2550;
	Infantry.Solo_Yaw_init = 20;
	Infantry.PitchMotorID = 0x205;
	Infantry.YawMotorID = 0x206;
	Infantry.FricMotorID[0]=0x202;
	Infantry.FricMotorID[1]=0x201;
	Infantry.BodanMotorID=0x203;
	Infantry.pn=-1;
} break;

/***************************************** 5 �ų� **************************************************************/	
	case 5:
{	
	Infantry.Yaw_init=6873;            //  5�ų�
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2400;
	Infantry.MagClose=600;
	Infantry.Solo_Yaw_init = 20;
} break;
/**************************************************************************************************************/
	default:
{
  Infantry.Yaw_init=5445;                  // �źŶ�ʧ 
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2000;
	Infantry.MagClose=540;
	Infantry.Solo_Yaw_init = 20;
}
}
}

/**********************************************************************************************************
*�� �� ��: delay_ms
*����˵��: ms��ʱ
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Offline_Check_task
*����˵��: ���߼������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
extern Disconnect Robot_Disconnect;
void Offline_Check_task(void *pvParameters)
{
   while (1) {

		/*���\IMU���߼��*/
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
	
	 /*����������� */
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
		
	/*ң�������߼��*/
	if(Robot_Disconnect.RC_DisConnect>10)
		{
			RC_Rst();
		}
	Robot_Disconnect.RC_DisConnect++;
		
	/*���̰���߲���ϵͳ���߼��*/
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



