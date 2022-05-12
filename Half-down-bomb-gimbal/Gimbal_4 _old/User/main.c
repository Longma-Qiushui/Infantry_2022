/**********************************************************************************************************
 * @�ļ�     main.c
 * @˵��     ���ļ�
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
extern KalmanFilter_t pitch_Kalman, yaw_Kalman;
extern KalmanFilter_Init_t K;
unsigned volatile long run_time_check = 0;	//���������ּ��׼���������
short fric_flag = 0;//Ħ���ֵ����ʼ����־
RobotInit_Struct Infantry;

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

	System_Config();
	delay_ms(3000);
	System_Init();
	
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
void System_Config(void)
{
	USART1_Configuration();
	delay_ms(100);
	USART6_Configuration();
	delay_ms(100);
	My_GPIO_Init();	//���ڸ��������ͬ��
	delay_ms(100);
	MicroSwConfigration();
	delay_ms(100);
//  SteeringEngine_Configuration();
//	delay_ms(100);
	TIM7_Configuration();
	delay_ms(100);
	CAN1_Configuration();
	delay_ms(100);
	CAN2_Configuration();
	delay_ms(100);
//	IWDG_Config(IWDG_Prescaler_128 ,625);
	delay_ms(100);
	VOFA_USART_Configuration();
}
/**********************************************************************************************************
*�� �� ��: System_Init
*����˵��: ϵͳ������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void System_Init(void)
{
	ZeroCheck_Init();
	Infantry_Init();
	Pid_ChassisPosition_Init();
  PidGimbalMotor_Init();
	Pid_BodanMotor_Init();
	Pid_Friction_Init();
  FuzzyPidGimbalMotor_Init();
	KalmanFilter_Init(&pitch_Kalman, &K);
	KalmanFilter_Init(&yaw_Kalman, &K);//PC�ȴ�����������STM32�Ῠ��arm32�ļ���(δ֪)
	KalMan_doneflag = 1;
}

/**********************************************************************************************************
*�� �� ��: Infantry_Init
*����˵��: ����������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Infantry_Init(void)
{	
/*����*/
//	Infantry.Yaw_init=4730;
//	Infantry.Pitch_init=6880;
//	Infantry.MagOpen=2295;
//	Infantry.MagClose=783;
//	Infantry.Solo_Yaw_init = 3760;
//	Infantry.Low_FrictionSpeed = 5560;
//	Infantry.Medium_FrictionSpeed = 5750;
//	Infantry.High_FrictionSpeed = 6780;	

///*����*/	
//	Infantry.Yaw_init=1040; 
//	Infantry.Pitch_init=5300;
//	Infantry.MagOpen=1000;
//	Infantry.MagClose=2350;
//	Infantry.Solo_Yaw_init = 20;
//	
///*����Ӧ1��*/	
//	Infantry.Yaw_init=1402;    //1352; 
//	Infantry.Pitch_init=2058;
//	Infantry.MagOpen=2400;
//	Infantry.MagClose=600;
//	Infantry.Solo_Yaw_init = 20;

/*����Ӧ1��*/	
	Infantry.Yaw_init=6873;    //1352; 
	Infantry.Pitch_init=2058;
	Infantry.MagOpen=2400;
	Infantry.MagClose=600;
	Infantry.Solo_Yaw_init = 20;
}

/**********************************************************************************************************
*�� �� ��: delay_ms
*����˵��: ms��ʱ
*��    ��: ��
*�� �� ֵ: ��
************************** ++********************************************************************************/
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
	/*���̰�ͨ�ŵ��߼��*/
	if(Robot_Disconnect.F105_DisConect>15)
		{
		F105_Rst();
		}
	Robot_Disconnect.F105_DisConect++;
	
	
	if(run_time_check >100000) run_time_check = 0;
		 
  vTaskDelay(5); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}



