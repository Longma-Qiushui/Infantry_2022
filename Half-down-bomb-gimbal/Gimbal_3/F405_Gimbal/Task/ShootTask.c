/**********************************************************************************************************
 * @�ļ�     ShootTask.c
 * @˵��     ��������
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
# include "main.h"
/*----------------------------------�ڲ�����---------------------------*/
float MirocPosition;
short FrictionWheel_speed=0,BulletSpeed,BodanMotorCurrent,ShootAct_Init_Flag;
short BodanMotorCurrent;
int SendToTx2BullectCnt;
int Shoot_Init_flag = 0;
char ShootContinue=0;
/*----------------------------------�ṹ��------------------------------*/
Pid_Typedef PidBodanMotorPos,PidBodanMotorSpeed,PidFrictionSpeed[2];
PID FuzzyBodanMotorPos;
float Onegrid=32940.0f; 
/*----------------------------------�ⲿ����---------------------------*/
extern RC_Ctl_t RC_Ctl;
extern int Bodan_Pos;
extern F105_Typedef F105;//F105
extern F405_typedef F405;
extern Status_t Status;
extern Gimbal_Typedef Gimbal;
extern PC_Receive_t PC_Receive;
extern BodanMotorReceive_Typedef BodanReceive;
extern short armor_state;
extern RobotInit_Struct Infantry;
extern short FrictionReceive[2];
extern char Robot_ID;
#define PullerSpeed  3000
#define checkPullerSpeed  3000
/**********************************************************************************************************
*�� �� ��: FrictionSpeedChoose
*����˵��: Ħ����ת��ѡ��
*��    ��: flag
*�� �� ֵ: ��
**********************************************************************************************************/
void FrictionSpeedChoose(void)
{
	switch(F105.BulletSpeedLevel)
	{
		case 0:
		{
			FrictionWheel_speed = Infantry.Low_FrictionSpeed;
			break;
		}
		case 1:
		{
				FrictionWheel_speed = Infantry.Medium_FrictionSpeed;
			break;
		}
		case 2:
		{
				FrictionWheel_speed = Infantry.High_FrictionSpeed;
			break;
		}
		default:
		{
			FrictionWheel_speed = Infantry.Low_FrictionSpeed;
			break;
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Shoot_Fire_Cal
*����˵��: ����ģʽ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern char ReverseRotation;
char Reverse_Flag=0;
void Shoot_Fire_Cal()
{
	if(ShootAct_Init_Flag!=1)
		ShootAct_Init_Flag=1;
	if(F105.IsShootAble==1)
	{
//		if(Status.GimbalMode==Gimbal_Armor_Mode)
//		{
//			if(armor_state==ARMOR_NO_AIM)
//			{
//			goto Stop_Shoot;
//			}
//		}
			if(ShootContinue)
			{
				PidBodanMotorSpeed.SetPoint=PullerSpeed;
			}
			else
			{
				 //���Ʋ��̷�ת��������
				if(ReverseRotation)
				{
						if(Reverse_Flag==0)
						{
							PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint-Onegrid;
							Reverse_Flag=1;
						}
						if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000)
						{
							ReverseRotation=0;
							Reverse_Flag=0;
						}
				
				}
	
				else
				{

						if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<8000)
						{
							PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+MirocPosition; 
						}
			
				}
				PidBodanMotorSpeed.SetPoint = PID_Calc(&PidBodanMotorPos,Bodan_Pos);			
			}
		
	}
	//��������
	else
	{
		Stop_Shoot:	PidBodanMotorSpeed.SetPoint=0;	
	}
		MirocPosition = 0;
}

/**********************************************************************************************************
*�� �� ��: Shoot_Check_Cal
*����˵��: ��¼ģʽ[�򿪼��⣬Ħ����/��Ƶ3~4��]
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
int delay_time =-1000;
void Shoot_Check_Cal()
{ 
	delay_time++;
	if(ShootAct_Init_Flag!=2)
	ShootAct_Init_Flag=2;
	MirocPosition = 0;
	 if(delay_time>0)
	 {
		if(F105.IsShootAble==1)
		{
      if(Shoot_Init_flag == 1)
			{
				PidBodanMotorSpeed.SetPoint=checkPullerSpeed;
				delay_time=0;	
			}
		}
		else
		{
				PidBodanMotorSpeed.SetPoint=0;	
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Shoot_Test_Cal
*����˵��: ����ģʽ�����ڸ�����λ������
*��    ��: 
*�� �� ֵ: ��
**********************************************************************************************************/
void Shoot_Test_Cal()
{ 
  delay_time++;
	if(ShootAct_Init_Flag!=0xcf)
	ShootAct_Init_Flag=0xcf;
	MirocPosition = 0;
	if(delay_time>0)
	{ 
	if(F105.IsShootAble==1 && armor_state == ARMOR_AIMED)
		if(ABS(Bodan_Pos-PidBodanMotorPos.SetPoint)<3000)
		{
			PidBodanMotorPos.SetPoint = PidBodanMotorPos.SetPoint+Onegrid; 
     armor_state=ARMOR_NO_AIM;			
		}
	}	
	 PidBodanMotorSpeed.SetPoint = PID_Calc(&PidBodanMotorPos,Bodan_Pos);		
 
}

/**********************************************************************************************************
*�� �� ��: Shoot_Powerdown_Cal
*����˵��: ����ģʽ[΢������/Ħ����/����ת��]
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
void Shoot_Powerdown_Cal(void)
{
	if(ShootAct_Init_Flag!=4)
	  ShootAct_Init_Flag=4;
	PidBodanMotorPos.SetPoint=Bodan_Pos;
	PidBodanMotorSpeed.SetPoint=PID_Calc(&PidBodanMotorPos,Bodan_Pos);
}

/**********************************************************************************************************
*�� �� ��: Shoot_Tx2_Cal
*����˵��: ����ģʽ
*��    ��: rc
*�� �� ֵ: ��
**********************************************************************************************************/
extern short armor_state;
extern float k_onegrid;
void Shoot_Tx2_Cal()
{
	if(ShootAct_Init_Flag!=3)
	{
		MirocPosition = 0;
		ShootAct_Init_Flag=3;
		SendToTx2BullectCnt=PC_Receive.ReceiveFromTx2BullectCnt=0;
	}
/***********************************************Armor******************************************/	
	if(Status.GimbalMode == Gimbal_Armor_Mode)
	{
		if(F105.IsShootAble==1)
  	{
			if(ABS(PC_Receive.RCPitch - (-Gimbal.Pitch.Gyro))<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<1.5f)	//�Ѿ����鵽λ���Զ�����
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<5000)
				{
					MirocPosition = k_onegrid * Onegrid;
					SendToTx2BullectCnt++;
				}
			}
    }
	}
	
	if(Status.GimbalMode == Gimbal_Buff_Mode)
	{
		if(F105.IsShootAble==1 && armor_state == ARMOR_AIMED)
  	{
			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//�Ѿ����鵽λ���Զ�����
			{
				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
				{
					MirocPosition = k_onegrid * Onegrid;
					SendToTx2BullectCnt++;
				}
			}
    }
	}	
//����
//		if(armor_state == ARMOR_AIMED)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<1.5f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.MotorTransAngle)<1.5f)	//�Ѿ����鵽λ���Զ�����
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	/*�����Ʒ���ʱ����*/
//	if(Status.GimbalMode == Gimbal_Buff_Mode)
//	{
//		if(F105.IsShootAble==1)
//  	{
//			if(ABS(PC_Receive.RCPitch - Gimbal.Pitch.MotorTransAngle)<0.8f && ABS(PC_Receive.RCYaw - Gimbal.Yaw.Gyro)<0.8f)	//���ҵ�Ŀ�겢����һ����Χ��
//			{
//				if(ABS(PidBodanMotorPos.SetPoint-Bodan_Pos)<8000)
//				{
//					MirocPosition = k_onegrid * Onegrid;
//					SendToTx2BullectCnt++;
//				}
//			}
//    }
//	}
	
//	/*���Ʒ���ʱ����*/
//	if(Status.GimbalMode == Gimbal_Buff_Mode)
//	{
//		if(idata.IsShootAble==1)
//  	{
//			if(PC_Receive.ReceiveFromTx2BullectCnt>SendToTx2BullectCnt)
//			{
//				  if(idata.IsShootAble==1)
//					{
//						if(DaFuBodanTick %300 == 0)
//						{
//							DaFuBodanTick =0;
//							if(ABS(PidBodanMotorPos.SetPoint- ZeroCheck_BodanPosOutPut())<8000)
//							{
//					      MirocPosition = Onegrid;
//					      SendToTx2BullectCnt++;
//							}
//						}
//					}
//					DaFuBodanTick++;
//			}
//			else
//				DaFuBodanTick = 0;
//    }
//	}
	
//	if(F105.IsShootAble==1)
		PidBodanMotorPos.SetPoint=PidBodanMotorPos.SetPoint+MirocPosition;  
		MirocPosition = 0;
}

/**********************************************************************************************************
*�� �� ��: BodanMotor_CurrentPid_Cal
*����˵��: �������λ�û�˫���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void BodanMotor_CurrentPid_Cal(void)
{
	switch(Status.ShootMode)//���ģʽѡ��
	{
		case Shoot_Check_Mode:
			Shoot_Check_Cal();
			break;
		case Shoot_Fire_Mode:
			Shoot_Fire_Cal();
			break;
		case Shoot_Tx2_Mode:
			Shoot_Test_Cal();			//210424����
//			Shoot_Tx2_Cal();
			break;
		case Shoot_Powerdown_Mode:
			Shoot_Powerdown_Cal();
			break;
		default:
			break;
	}
	if(Shoot_Init_flag == 0)
	{
  	Shoot_Powerdown_Cal();
		delay_time=-1000;
	}

	BodanMotorCurrent = (short)PID_Calc(&PidBodanMotorSpeed,BodanReceive.RealSpeed);
if(Status.ShootMode==Shoot_Powerdown_Mode)
{
  BodanMotorCurrent =0.0f;
}
}

/**********************************************************************************************************
*�� �� ��: Pid_BodanMotor
*����˵��: �������λ���ٶȻ�pid������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_BodanMotor_Init(void)
{
	
	PidBodanMotorPos.P=0.15f;
	PidBodanMotorPos.I=0.0f;
	PidBodanMotorPos.D=0.0f;
	PidBodanMotorPos.IMax=1500.0f;
	PidBodanMotorPos.SetPoint=0.0f;
	PidBodanMotorPos.OutMax=6000.0f;

	PidBodanMotorSpeed.P=15.0f;  //5.0f
	PidBodanMotorSpeed.I=0.5f;//0.01f;
	PidBodanMotorSpeed.D=0.0f;
	PidBodanMotorSpeed.DeadZone=50.0f;
	PidBodanMotorSpeed.IMax=1000.0f;
	PidBodanMotorSpeed.SetPoint=0.0f;
	PidBodanMotorSpeed.OutMax = 10000.0f;
}

/**********************************************************************************************************
*�� �� ��: Pid_Friction_Init
*����˵��: �������λ���ٶȻ�pid������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_Friction_Init(void)
{
	switch(Robot_ID)
{
/********************************************* 3�ų� *******************************************************/	
		case 3:
		{
			Infantry.Low_FrictionSpeed = 4850;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed =16000;
		} break;
/********************************************* 4�ų� *******************************************************/	
		case 4:
		{
			Infantry.Low_FrictionSpeed = 4850;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed = 16000;
		} break;
/********************************************* 5�ų� *******************************************************/	
		case 5:
		{
			Infantry.Low_FrictionSpeed = 4850;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed = 16000;
		} break;		
/********************************************* ȱʡֵ ******************************************************/		
		default:
		{
			Infantry.Low_FrictionSpeed = 4850;
			Infantry.Medium_FrictionSpeed = 5800;
			Infantry.High_FrictionSpeed = 16000;
		}
}
  PidFrictionSpeed[0].P=60.0f;
	PidFrictionSpeed[0].I=0.0f;
	PidFrictionSpeed[0].D=0.0f;
	PidFrictionSpeed[0].IMax=1500.0f;
	PidFrictionSpeed[0].SetPoint=0.0f;
	PidFrictionSpeed[0].OutMax = 10000.0f;
	
  PidFrictionSpeed[1].P=60.0f;
	PidFrictionSpeed[1].I=0.0f;
	PidFrictionSpeed[1].D=0.0f;
	PidFrictionSpeed[1].IMax=1500.0f;
	PidFrictionSpeed[1].SetPoint=0.0f;
	PidFrictionSpeed[1].OutMax = 10000.0f;
}


/**********************************************************************************************************
*�� �� ��: Shoot_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
#define POWER_ON 1
#define POWER_OFF 0
uint32_t Shoot_high_water;
int Shoot_Power = POWER_ON;		//��ȡ���������ѹ
extern short FrictionCurrent[2];
void Shoot_task(void *pvParameters)
{
  portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
  while (1) {
	  xLastWakeTime = xTaskGetTickCount();
	 
	//	 Shoot_Power = GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_11);
	//	  Shoot_Power=POWER_ON;
		 if(Shoot_Power == POWER_ON)
		 {

			if(Status.ShootMode == Shoot_Powerdown_Mode)
			{
				F405.Fric_Flag=0;
				FrictionWheel_speed=0;
				Shoot_Init_flag = 0;				//��ֹ����ת
			}else		//�Ѿ���ʼ���ɹ�һ���ˣ������л��˷ǵ���ģʽ
			{
			F405.Fric_Flag=1;
			FrictionSpeedChoose();
			Shoot_Init_flag = 1;
			}
    		FrictionWheel_Set(FrictionWheel_speed);
		 }
		 else if(Shoot_Power == POWER_OFF)
		 {
				Shoot_Init_flag = 0;				//��ֹ����ת����
		 }
			
		 BodanMotor_CurrentPid_Cal();
		 FrictionBodanCan2Send(FrictionCurrent[0],FrictionCurrent[1],BodanMotorCurrent);
//		 VOFA_Send();
		 IWDG_Feed();
     vTaskDelayUntil(&xLastWakeTime,xFrequency); 
	
#if  INCLUDE_uxTaskGetStackHighWaterMark
        Shoot_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif

    }
}
