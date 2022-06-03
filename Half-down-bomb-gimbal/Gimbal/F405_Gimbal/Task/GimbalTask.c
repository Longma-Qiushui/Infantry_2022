/**********************************************************************************************************
 * @文件     GimbalTask.c
 * @说明     云台控制
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2020.1
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/
int gimbal_pitch_max = 34;		//初始化PITCH限位
int gimbal_pitch_min = -11;

int inttoshort[4];
short GimbalAct_Init_Flag=0;

short YawCurrent,PitchCurrent;
float GimbalYawPos,GimbalPitchPos;

float Recent_Angle_Buff;
float Recent_Yaw_Angle_Armor;
float Recent_Pitch_Angle_Armor;
/*-----------------------------------结构体-----------------------------*/
Pid_Typedef PidPitchSpeed,PidPitchPos,PidYawSpeed,PidYawPos;
Pid_Typedef PidPitchAidPos,PidPitchAidSpeed,PidYawAidPos,PidYawAidSpeed;
PID FuzzyPidPitchPos,FuzzyAidPidPitchPos,FuzzyAidPidYawPos;
/*----------------------------------外部变量---------------------------*/
extern RC_Ctl_t RC_Ctl; 
extern Status_t Status;
extern char PitchMotor_ReceiveFlag;
extern Gimbal_Typedef Gimbal;
extern Gyro_Typedef GyroReceive;//陀螺仪数据
extern RobotInit_Struct Infantry;
extern short FrictionWheel_speed;
extern F405_typedef F405;
extern PC_Receive_t PC_Receive;
extern char Robot_ID;

/**********************************************************************************************************
*函 数 名: Gimbal_Powerdown_Cal
*功能说明: 云台掉电模式
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Gimbal_Powerdown_Cal()
{
	if(GimbalAct_Init_Flag!=Gimbal_Powerdown_Mode)
	{
		Laser_Off();
	  GimbalAct_Init_Flag=Gimbal_Powerdown_Mode;
	}
	
	/***********************************************************************************/
	FuzzyPidPitchPos.SetPoint = gimbal_pitch_min;
	FuzzyPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  /**************************************计算电流值**************************************/
//PITCH                 
  PidPitchSpeed.SetPoint=FuzzyPID_Calc(&FuzzyPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,-GyroReceive.GY));//旧陀螺仪
	PitchCurrent=(short)(-inttoshort[0]);

	YawCurrent=0;
	
}

///**********************************************************************************************************
//*函 数 名: Gimbal_Act_Cal
//*功能说明: 云台正常模式
//*形    参: rc  mouse  Pc_RecvData
//*返 回 值: 无
//**********************************************************************************************************/
//float test_pitch = 0;
//float test_yaw = 0;
//int aaaaa;
//void Gimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
//{
//	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
//	{
//		Laser_On();
//		GimbalPitchPos = -GyroReceive.PITCH;				//从大符模式切回，保持pitch电机角，yaw陀螺仪角，不乱动
//		GimbalYawPos = Gimbal.Yaw.Gyro;
//		GimbalAct_Init_Flag=Gimbal_Act_Mode;
//	}

//	
//	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
//	{
//	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
//	  GimbalPitchPos -= (1024-rc.ch3)*0.0003f;//旧陀螺仪
//	}
//	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
//	{
//    GimbalPitchPos -= mouse.y*0.005f;
//    GimbalYawPos   -= mouse.x*0.005f;
//		GimbalPitchPos += mouse.z*0.001f;
//	}

//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)	//限住大小，因此可用MotorTransangle来pid
////YAW/***********************************************************************************/
//	PidPitchPos.SetPoint = GimbalPitchPos;
//	PidYawPos.SetPoint = GimbalYawPos;

//	
//  /**************************************计算电流值**************************************/
////PITCH                
////  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);   
//  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,-GyroReceive.PITCH);   	
//	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,-GyroReceive.GY));//旧陀螺仪
//	PitchCurrent=(short)inttoshort[0];
////YAW
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//需要YAW轴陀螺仪角度做ActualValue
//	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
//	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
//	YawCurrent=(short)inttoshort[1];
//	/* pid_test 负数*/
////	test_gyro_pitch = -Gimbal.Pitch.Gyro;
////	test_gyro_yaw = -Gimbal.Yaw.Gyro;
//}
/**********************************************************************************************************
*函 数 名: FuzzyGimbal_Act_Cal
*功能说明: 模糊云台正常模式(电机角)
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/
void FuzzyMotorGimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;				//从大符模式切回，保持pitch电机角，yaw陀螺仪角，不乱动
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag=Gimbal_Act_Mode;
	}

	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0003f;
	  GimbalPitchPos -= (1024-rc.ch3)*0.0010f;//旧陀螺仪
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.01f;
		GimbalPitchPos += mouse.z*0.002f;
	}

	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)	//限住大小，因此可用MotorTransangle来pid
	/***********************************************************************************/
	FuzzyPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	PidYawPos.SetPoint = GimbalYawPos;
	
  /**************************************计算电流值**************************************/
//PITCH                 
  PidPitchSpeed.SetPoint=FuzzyPID_Calc(&FuzzyPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,-GyroReceive.GY));//旧陀螺仪
	PitchCurrent=(short)(-inttoshort[0]);

//YAW
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//需要YAW轴陀螺仪角度做ActualValue
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=(short)inttoshort[1];
	/* pid_test 负数*/
//	test_gyro_pitch = -Gimbal.Pitch.Gyro;
//	test_gyro_yaw = -Gimbal.Yaw.Gyro;
}
/**********************************************************************************************************
*函 数 名: FuzzyGimbal_Act_Cal
*功能说明: 模糊云台正常模式(IMU角)
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/

void FuzzyGyroGimbal_Act_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if( GimbalAct_Init_Flag!=Gimbal_Act_Mode)
	{
		Laser_On();
		GimbalPitchPos = -GyroReceive.PITCH;				
		GimbalYawPos = Gimbal.Yaw.Gyro;
		GimbalAct_Init_Flag=Gimbal_Act_Mode;
	}

	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
	  GimbalPitchPos -= (1024-rc.ch3)*0.0003f;//旧陀螺仪
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.005f;
		GimbalPitchPos += mouse.z*0.001f;
	}

	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)	//限住大小，因此可用MotorTransangle来pid
	/***********************************************************************************/
	FuzzyPidPitchPos.SetPoint = -GimbalPitchPos;
	FuzzyPidPitchPos.ActPoint = Gimbal.Pitch.Gyro;
	PidYawPos.SetPoint = GimbalYawPos;

	
  /**************************************计算电流值**************************************/
//PITCH                
//  PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);   
  PidPitchSpeed.SetPoint=FuzzyPID_Calc(&FuzzyPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//旧陀螺仪
	PitchCurrent=(short)inttoshort[0];

//YAW
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.Gyro);					//需要YAW轴陀螺仪角度做ActualValue
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=(short)inttoshort[1];
	/* pid_test 负数*/
//	test_gyro_pitch = -Gimbal.Pitch.Gyro;
//	test_gyro_yaw = -Gimbal.Yaw.Gyro;
}

/**********************************************************************************************************
*函 数 名: Gimbal_Armor_Cal
*功能说明: 云台辅瞄模式
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/
float speed_limit = 30.0f;
float Inte_z;
void Gimbal_Armor_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if(GimbalAct_Init_Flag!=Gimbal_Armor_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_Armor_Mode;
		GimbalYawPos = Gimbal.Yaw.Gyro;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCPitch=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCYaw = Gimbal.Yaw.Gyro;
		Inte_z = 0;
	}

	/**解决过零问题***/
	Recent_Pitch_Angle_Armor = Pc_Recv->RCPitch;
	while(ABS(Recent_Pitch_Angle_Armor) >= 90 )
	{
		if(Recent_Pitch_Angle_Armor<0)
			 Recent_Pitch_Angle_Armor=Recent_Pitch_Angle_Armor+180;  
		else
			 Recent_Pitch_Angle_Armor=-(Recent_Pitch_Angle_Armor-180);
	}
	
	
	Recent_Yaw_Angle_Armor = Pc_Recv->RCYaw;
	while(ABS(Gimbal.Yaw.Gyro - Recent_Yaw_Angle_Armor) >= 180 )
	{
		if(Gimbal.Yaw.Gyro<0)
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor-360;  
		else
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor+360;
	}
	Inte_z += mouse.z*0.002f;
	GimbalPitchPos= Recent_Pitch_Angle_Armor + Inte_z;
	GimbalYawPos = Recent_Yaw_Angle_Armor;
	
//	GimbalYawPos = 20.0f;

	
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)	//限住大小，因此可用MotorTransangle来pid
	/***********************************************************************************/
	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyAidPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  PidPitchAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,-GyroReceive.GY));//旧陀螺仪
	PitchCurrent=-(short)inttoshort[0];
		
	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	FuzzyAidPidYawPos.ActPoint = Gimbal.Yaw.Gyro;
	
	PidYawAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidYawPos); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}


/**********************************************************************************************************
*函 数 名: Gimbal_Buff_Cal
*功能说明: 云台打符模式
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/
void Gimbal_Buff_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if(GimbalAct_Init_Flag!=Gimbal_BigBuf_Mode)
	{
		GimbalAct_Init_Flag=Gimbal_BigBuf_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;
	  GimbalPitchPos=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCPitch=Gimbal.Pitch.MotorTransAngle;
		Pc_Recv->RCYaw = Gimbal.Yaw.MotorTransAngle;
	}

	
	/**解决过零问题***/
	Recent_Pitch_Angle_Armor = Pc_Recv->RCPitch;
	while(ABS(Recent_Pitch_Angle_Armor) >= 90 )
	{
		if(Recent_Pitch_Angle_Armor<0)
			 Recent_Pitch_Angle_Armor=Recent_Pitch_Angle_Armor+180;  
		else
			 Recent_Pitch_Angle_Armor=-(Recent_Pitch_Angle_Armor-180);
	}
	
	
	Recent_Yaw_Angle_Armor = Pc_Recv->RCYaw;
	while(ABS(Gimbal.Yaw.MotorTransAngle - Recent_Yaw_Angle_Armor) >= 180 )
	{
		if(Gimbal.Yaw.MotorTransAngle<0)
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor-360;  
		else
			 Recent_Yaw_Angle_Armor=Recent_Yaw_Angle_Armor+360;
	}
	
	GimbalPitchPos= Recent_Pitch_Angle_Armor;
	GimbalYawPos = Recent_Yaw_Angle_Armor;
	
	GimbalPitchPos = LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位
	/***************************************************************************************/
	FuzzyAidPidPitchPos.SetPoint = GimbalPitchPos;
	FuzzyAidPidPitchPos.ActPoint = Gimbal.Pitch.MotorTransAngle;
	
  PidPitchAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidPitchPos);   	
	inttoshort[0]=(PID_Calc(&PidPitchAidSpeed,-GyroReceive.GY));//旧陀螺仪
	PitchCurrent=-(short)inttoshort[0];
		
	FuzzyAidPidYawPos.SetPoint = GimbalYawPos;
	FuzzyAidPidYawPos.ActPoint = Gimbal.Yaw.MotorTransAngle;
	
	PidYawAidSpeed.SetPoint=FuzzyPID_Calc(&FuzzyAidPidYawPos); 
	inttoshort[1] = PID_Calc(&PidYawAidSpeed,GyroReceive.GZ);
	YawCurrent = inttoshort[1];
}

/**********************************************************************************************************
*函 数 名: Gimbal_DropShot_Cal
*功能说明: 云台吊射模式
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/

void Gimbal_DropShot_Cal(Remote rc,Mouse mouse,PC_Receive_t *Pc_Recv)
{
	if( GimbalAct_Init_Flag!=Gimbal_DropShot_Mode)
	{
		Laser_On();
		GimbalAct_Init_Flag=Gimbal_DropShot_Mode;
		GimbalYawPos = Gimbal.Yaw.MotorTransAngle;//定轴模式
		GimbalPitchPos = Gimbal.Pitch.MotorTransAngle;
	}
	
	if(Status.ControlMode==Control_RC_Mode)//Rc_Control
	{
	  GimbalYawPos   += (1024-rc.ch2)*0.0005f;
		GimbalPitchPos -= (1024-rc.ch3)*0.0005f;//陀螺仪
	}
	if(Status.ControlMode==Control_MouseKey_Mode)//Mouse_Key
	{
    GimbalPitchPos -= mouse.y*0.005f;
    GimbalYawPos   -= mouse.x*0.005f;
	}
	
	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)
  /***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
  /**************************************计算电流值**************************************/
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  
	inttoshort[0]=-(PID_Calc(&PidPitchSpeed,-GyroReceive.GY));//旧陀螺仪
	PitchCurrent=(short)inttoshort[0];
	
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);	
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=inttoshort[1];	
}

/**********************************************************************************************************
*函 数 名: Gimbal_SI_Cal
*功能说明: 云台系统辨识模式
*形    参: rc  mouse  Pc_RecvData
*返 回 值: 无
**********************************************************************************************************/
short F_Change_flag = 0;		//切换频率标志
float	Gimbal_direct;

int T;//周期
int T_cnt = 0;//计数
int T_Time_cnt = 0;//周期次数计数
int F_cnt = 0,F_cnt_last;//指向F的指针
float F = 1;

void Gimbal_SI_Cal(float Gimbal_pitch,float Gimbal_yaw)
{
	if(GimbalAct_Init_Flag!=Gimbal_SI_Mode)
	{
		GimbalPitchPos = 0;				
		GimbalYawPos = Gimbal.Yaw.Gyro;
		F = 1;		
		GimbalAct_Init_Flag = Gimbal_SI_Mode;
	}
		
	T_change();		
//	if(Gimbal_yaw >0.1 &&Gimbal_pitch == 0)
//	{
//		GimbalYawPos = (Gimbal_direct*Gimbal_yaw);
//	}
//	else if(Gimbal_yaw < 0.1&&Gimbal_pitch != 0)
//	{
		GimbalPitchPos = LIMIT_MAX_MIN((Gimbal_direct*Gimbal_pitch),gimbal_pitch_max,gimbal_pitch_min);
//	}
	
//	GimbalPitchPos=LIMIT_MAX_MIN(GimbalPitchPos,gimbal_pitch_max,gimbal_pitch_min);//限位(用电机角度)
  /***********************************************************************************/
	PidPitchPos.SetPoint = GimbalPitchPos;
	PidYawPos.SetPoint = GimbalYawPos;
  /**************************************计算电流值**************************************/	
//	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,Gimbal.Pitch.MotorTransAngle);  		//和手动操作正负不一样
	PidPitchSpeed.SetPoint=PID_Calc(&PidPitchPos,GyroReceive.PITCH);  		//和手动操作正负不一样
	inttoshort[0]=(PID_Calc(&PidPitchSpeed,GyroReceive.GY));//旧陀螺仪
	PitchCurrent=(short)inttoshort[0];
	
//	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,Gimbal.Yaw.MotorTransAngle);	
	PidYawSpeed.SetPoint=PID_Calc(&PidYawPos,GyroReceive.YAW);	
	PidYawSpeed.SetPoint=LIMIT_MAX_MIN(PidYawSpeed.SetPoint,5.5f,-5.5f);    
	inttoshort[1]=PID_Calc(&PidYawSpeed,GyroReceive.GZ);
	YawCurrent=inttoshort[1];	
}

void get_F(void)
{
	static int j = 0;
	if(F < 22)
	{
		j++;
		F += 0.5f;
	}
	else if(F == 22)
	{
		F = 24;
	}
	else if(F >= 24&&F < 40)
	{
		F = F + 2;
	}
	else if(F == 40)
	{
		F = 50;
	}
	else if(F >= 50&&F < 120)
	{
		F = F + 10;
	}
	else if(F == 120)
	{
		F = 200;
	}
	else if(F == 250)
	{
		F = 333;
	}
	else if(F == 333)
	{
		F = 500;
	}
}
/**********************************************************************************************************
*函 数 名: T_Change
*功能说明: 根据不同的频率获得对应不同周期下sin信号,F低于50重复10周期，否则20周期
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void T_change(void)
{
	T = round(1000/F);
	if(GimbalAct_Init_Flag == Gimbal_SI_Mode)
	{
		if(F_Change_flag == 0)
		{
			Gimbal_direct = sin(2*3.14*T_cnt/T);
			if(T_cnt >= T)
			{
				T_cnt = 0;
				T_Time_cnt++;
			}
			T_cnt++;
			F_cnt_last = F_cnt;

			if(T_Time_cnt >= 10 && F <22)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 10 && F >= 22 && F <50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			else if(T_Time_cnt >= 20 && F >= 50)
			{
				T_Time_cnt = 0;
				F_cnt++; 
			}
			if(F_cnt != F_cnt_last)		//频率改变
			{
				F_Change_flag  = 1;
			}
		}
		else if(F_Change_flag == 1)
		{
			Gimbal_direct = 0;
			if((ABS(PidPitchPos.SetPoint - PidPitchPos.ActualValue) < 0.2f) || (ABS(PidYawPos.SetPoint - PidYawPos.ActualValue) < 0.2f))		//回到初始位置
			{
				get_F();
				T_cnt = 0;
				F_Change_flag = 0;
			}
		}
	}
	else if(GimbalAct_Init_Flag != Gimbal_SI_Mode)
	{
		F = 1;
	}
}


/**********************************************************************************************************
*函 数 名: Gimbal_CurrentPid_Cal
*功能说明: 发送电流值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Gimbal_CurrentPid_Cal(void)
{
	switch (Status.GimbalMode)
	{
		case Gimbal_Powerdown_Mode:
			Gimbal_Powerdown_Cal();
			break;
		case Gimbal_Act_Mode:
			FuzzyMotorGimbal_Act_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;  
		case Gimbal_Armor_Mode:
		case Gimbal_AntiSP_Mode:
			Gimbal_Armor_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;
		case Gimbal_BigBuf_Mode:
		case Gimbal_SmlBuf_Mode:
			Gimbal_Buff_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break;
		case Gimbal_DropShot_Mode:
			Gimbal_DropShot_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
			break; 
		case Gimbal_SI_Mode:
//			Gimbal_SI_Cal(20.0, 0);			//pitch
//			Gimbal_SI_Cal(0.0, 30.0);		//yaw
			break;
		case Gimbal_Jump_Mode:
			FuzzyGyroGimbal_Act_Cal(RC_Ctl.rc,RC_Ctl.mouse,&PC_Receive);
		default:
			break;
	}
	F405.Gimbal_Flag = Status.GimbalMode;
	GimbalCan2Send(PitchCurrent,YawCurrent);

}


/**********************************************************************************************************
*函 数 名: Pid_Yaw_MotorPos_GyroSpeed
*功能说明: Yaw轴辅瞄双环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void PidGimbalMotor_Init(void)
{ 
	switch(Robot_ID)
{
/********************************************* 3号车 ********************************************************/	
		case 3:
{
  //手动pitch速度环                 
  PidPitchSpeed.P=6000.0f;  //5000.0f;  
	PidPitchSpeed.I=12.0f;   //10.0f; 
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//手动yaw双环                                 // 3号车
  PidYawPos.P=0.23f;     
	PidYawPos.I=0.00f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	PidYawPos.DeadZone=0.2f;
	PidYawSpeed.P=9000.0f;   //30000
	PidYawSpeed.I=5.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//辅瞄pitch速度环
  PidPitchAidSpeed.P=13000.0f;	  //18000
	PidPitchAidSpeed.I=18.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//辅瞄yaw速度环
	PidYawAidSpeed.P=24000.0f;			//24000
	PidYawAidSpeed.I=10.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
}break;
/********************************************* 4号车 ********************************************************/	
		case 4:
{
  PidPitchSpeed.P=7000.0f;  //5000.0f;  
	PidPitchSpeed.I=12.0f;   //10.0f; 
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//手动yaw双环
  PidYawPos.P=0.25f;     
	PidYawPos.I=0.01f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	PidYawPos.DeadZone=0.0f;
	PidYawSpeed.P=10000.0f;   //30000
	PidYawSpeed.I=10.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//辅瞄pitch速度环
  PidPitchAidSpeed.P=10000.0f;	  //10000
	PidPitchAidSpeed.I=14.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//辅瞄yaw速度环
	PidYawAidSpeed.P=24000.0f;			//12000
	PidYawAidSpeed.I=8.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
}break;

/********************************************* 5号车 ********************************************************/	
		case 5:
{
   PidPitchSpeed.P=6000.0f;  //5000.0f;  
	PidPitchSpeed.I=12.0f;   //10.0f; 
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//手动yaw双环
  PidYawPos.P=0.30f;     
	PidYawPos.I=0.03f;
	PidYawPos.D=0.1f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	PidYawPos.DeadZone=0.2f;
	PidYawSpeed.P=10000.0f;   //30000
	PidYawSpeed.I=0.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//辅瞄pitch速度环
  PidPitchAidSpeed.P=10000.0f;	  //18000
	PidPitchAidSpeed.I=5.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//辅瞄yaw速度环
	PidYawAidSpeed.P=10000.0f;			//12000
	PidYawAidSpeed.I=10.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
}break;

/********************************************* 缺省值 ********************************************************/	
		default:
		{
		 //手动pitch速度环                 
  PidPitchSpeed.P=6000.0f;  //5000.0f;  
	PidPitchSpeed.I=12.0f;   //10.0f; 
	PidPitchSpeed.D=0.0f;
	PidPitchSpeed.IMax=550.0f;
	PidPitchSpeed.SetPoint=0.0f;
	PidPitchSpeed.OutMax=30000.0f;
  
	//手动yaw双环                                 // 信号丢失
  PidYawPos.P=0.30f;     
	PidYawPos.I=0.03f;
	PidYawPos.D=0.0f;       
	PidYawPos.IMax=10.0f;
	PidYawPos.SetPoint=0.0f;
	PidYawPos.OutMax=5.5f;
	PidYawPos.DeadZone=0.2f;
	PidYawSpeed.P=10000.0f;   //30000
	PidYawSpeed.I=0.0f;
	PidYawSpeed.D=0.0f;
	PidYawSpeed.IMax=2000.0f;
	PidYawSpeed.SetPoint=0.0f;
	PidYawSpeed.OutMax=30000.0f;
	
	//辅瞄pitch速度环
  PidPitchAidSpeed.P=10000.0f;	  //18000
	PidPitchAidSpeed.I=5.0f; 
	PidPitchAidSpeed.D=0.0f;
	PidPitchAidSpeed.IMax=250.0f;
	PidPitchAidSpeed.SetPoint=0.0f;
	PidPitchAidSpeed.OutMax=30000.0f;
	
	//辅瞄yaw速度环
	PidYawAidSpeed.P=9000.0f;			//12000
	PidYawAidSpeed.I=10.0f; 
	PidYawAidSpeed.D=0.0f;
	PidYawAidSpeed.IMax=2000.0f;
	PidYawAidSpeed.SetPoint=0.0f;
	PidYawAidSpeed.OutMax=30000.0f;
		
		}
	}
}
/**********************************************************************************************************
*函 数 名: FuzzyPid_PitchPosSpeed_Init
*功能说明: Pitch轴双环PID
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void FuzzyPidGimbalMotor_Init(void)
{
	switch(Robot_ID)
{
/*************************************** 3号车 *********************************************************/	
		case 3:
		{
			//手动pitch角度环
			FuzzyPidPitchPos.Kp=0.25f;  //-0.33f;
			FuzzyPidPitchPos.Ki=0.0003f;  //0.005f;
			FuzzyPidPitchPos.Kd=0.0f;
			FuzzyPidPitchPos.IMax=40.0f;
			FuzzyPidPitchPos.SetPoint=0.0f;
	
			//辅瞄pitch角度环
			FuzzyAidPidPitchPos.Kp=0.2f;
			FuzzyAidPidPitchPos.Ki=0.0001f;  //0.005f;
			FuzzyAidPidPitchPos.Kd=0.0f;
			FuzzyAidPidPitchPos.IMax=40.0f;
			FuzzyAidPidPitchPos.SetPoint=0.0f;
	
			//辅瞄yaw角度环
			FuzzyAidPidYawPos.Kp=0.26f;
			FuzzyAidPidYawPos.Ki=0.0001f;  //0.005f;
			FuzzyAidPidYawPos.Kd=0.0f;
			FuzzyAidPidYawPos.IMax=40.0f;
			FuzzyAidPidYawPos.SetPoint=0.0f;

		}break;
/*********************************************** 4号车 **************************************************/		
				case 4:
		{
				//手动pitch角度环
			FuzzyPidPitchPos.Kp=0.25f;  //-0.33f;
			FuzzyPidPitchPos.Ki=0.0003f;  //0.005f;
			FuzzyPidPitchPos.Kd=0.0f;
			FuzzyPidPitchPos.IMax=40.0f;
			FuzzyPidPitchPos.SetPoint=0.0f;
	
				//辅瞄pitch角度环
			FuzzyAidPidPitchPos.Kp=0.23f;       //0.25
			FuzzyAidPidPitchPos.Ki=0.0005f;  //0.0003f;
			FuzzyAidPidPitchPos.Kd=0.0f;
			FuzzyAidPidPitchPos.IMax=40.0f;
			FuzzyAidPidPitchPos.SetPoint=0.0f;
	
			 //辅瞄yaw角度环
			FuzzyAidPidYawPos.Kp=0.26f;
			FuzzyAidPidYawPos.Ki=0.0001f;  //0.005f;
			FuzzyAidPidYawPos.Kd=0.0f;
			FuzzyAidPidYawPos.IMax=40.0f;
			FuzzyAidPidYawPos.SetPoint=0.0f;

		}break;
/*********************************************** 5号车 **************************************************/		
				case 5:
		{
			//手动pitch角度环
			FuzzyPidPitchPos.Kp=0.3f;  //-0.33f;
			FuzzyPidPitchPos.Ki=0.0003f;  //0.005f;
			FuzzyPidPitchPos.Kd=0.0f;
			FuzzyPidPitchPos.IMax=40.0f;
			FuzzyPidPitchPos.SetPoint=0.0f;
	
			//辅瞄pitch角度环
			FuzzyAidPidPitchPos.Kp=0.25f;
			FuzzyAidPidPitchPos.Ki=0.0003f;  //0.005f;
			FuzzyAidPidPitchPos.Kd=0.0f;
			FuzzyAidPidPitchPos.IMax=40.0f;
			FuzzyAidPidPitchPos.SetPoint=0.0f;
	
			//辅瞄yaw角度环
			FuzzyAidPidYawPos.Kp=0.32f;
			FuzzyAidPidYawPos.Ki=0.0005f;  //0.005f;
			FuzzyAidPidYawPos.Kd=0.0f;
			FuzzyAidPidYawPos.IMax=40.0f;
			FuzzyAidPidYawPos.SetPoint=0.0f;

		}break;	
/******************************************** 缺省值 ************************************************************/		
		default:
		{
					//手动pitch角度环
			FuzzyPidPitchPos.Kp=0.3f;  //-0.33f;
			FuzzyPidPitchPos.Ki=0.0003f;  //0.005f;
			FuzzyPidPitchPos.Kd=0.0f;
			FuzzyPidPitchPos.IMax=40.0f;
			FuzzyPidPitchPos.SetPoint=0.0f;
	
			//辅瞄pitch角度环
			FuzzyAidPidPitchPos.Kp=0.25f;
			FuzzyAidPidPitchPos.Ki=0.0002f;  //0.005f;
			FuzzyAidPidPitchPos.Kd=0.0f;
			FuzzyAidPidPitchPos.IMax=40.0f;
			FuzzyAidPidPitchPos.SetPoint=0.0f;
	
			//辅瞄yaw角度环
			FuzzyAidPidYawPos.Kp=0.30f;
			FuzzyAidPidYawPos.Ki=0.0003f;  //0.005f;
			FuzzyAidPidYawPos.Kd=0.0f;
			FuzzyAidPidYawPos.IMax=40.0f;
			FuzzyAidPidYawPos.SetPoint=0.0f;
		}
	}
}


/**********************************************************************************************************
*函 数 名: Gimbal_task
*功能说明: 云台任务函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Gimbal_high_water;
UBaseType_t GGGG;
void Gimbal_task(void *pvParameters)
{
	portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	vTaskDelay(1000);
   while (1) {
		xLastWakeTime = xTaskGetTickCount();
  
		 if(PitchMotor_ReceiveFlag)
	{
		ZeroCheck_cal(); 
		Gimbal_CurrentPid_Cal();
	}	 
	
		GGGG=uxTaskGetStackHighWaterMark(NULL);
		
		IWDG_Feed();//喂狗
		
    vTaskDelayUntil(&xLastWakeTime,xFrequency); 

#if INCLUDE_uxTaskGetStackHighWaterMark
        Gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

