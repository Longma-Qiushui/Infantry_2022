/**********************************************************************************************************
 * @文件     ChassisTask.c
 * @说明     底盘控制+功率限制
 * @版本  	 V3.0
 * @作者     赵业权
 * @日期     2021.4.26
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/
short WheelCurrentSend[4];
short Set_Jump[4] = {0};
float Current_Change[4] = {0};//电流环增量
float Current_f[4] = {0};//输出电流f
float Flow[4] = {0};//实际电流f
float speed[4] = {0};//实际速度f

//功率限制系数 PowerLimit
short Actual_P_max;						//实际最大功率
short Self_Protect_Limit;			//小陀螺转速限制
float k_BAT;
short Excess_P_max;
short CurrentMax;
short Follow_W;

const float k_chassis_t = 0.0168f;			
//24*0.0007 24：chassis口输出电压 电池满电24 一格电21；
//0.0007：电机控制电流值对应的单位chassis口输出电流

/*----------------------------------结构体-----------------------------*/
Pid_Typedef Pid_Current[4];
Pid_Typedef pidChassisWheelSpeed[4];
Pid_Typedef pidChassisPosition_Speed;
F105_Typedef F105;
Power_Typedef Power_method[METHOD_NUM];

/*----------------------------------外部变量---------------------------*/
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassis;
extern RM820RReceive_Typedef ChassisMotorCanReceive[4];
extern F405_typedef F405;
extern enum POWERSTATE_Typedef PowerState;

extern float Input[4];
extern float Output[4];

float k_xy = 3.0f,y_lim=1.0f;
short carSpeedw = 0;
float ABSready_flag=0; 
float Goready_flag=0;
float T_ABS=500.0f;//刹车时间
float T_SETUP=600.0f;//启动时间

uint16_t  MyMaxPower=0;
/**********************************************************************************************************
*函 数 名: ABS_Cal
*功能说明: 缓速启停
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void ABS_Cal(void)
{
static TickType_t StartTick=0,NowTick;
static TickType_t StopTick=0;
static char StartInitFlag=0,StopInitFlag=0;
if(chassis.carSpeedx!=0||chassis.carSpeedy!=0) //正常行驶，开始进入加速
{
	
	if(Goready_flag==1)//开始启动，进入起步态
	{
	if(!StartInitFlag)
	{
	StartTick=xTaskGetTickCount();
	StartInitFlag=1;
	}
  NowTick=xTaskGetTickCount();
	pidChassisWheelSpeed[0].SetPoint= k_xy*(+((NowTick-StartTick)/T_SETUP)*chassis.carSpeedy+((NowTick-StartTick)/T_SETUP)*(+chassis.carSpeedx))-carSpeedw;
	pidChassisWheelSpeed[1].SetPoint= k_xy*(+((NowTick-StartTick)/T_SETUP)*chassis.carSpeedy+((NowTick-StartTick)/T_SETUP)*(-chassis.carSpeedx))-carSpeedw;
	pidChassisWheelSpeed[2].SetPoint= k_xy*(-((NowTick-StartTick)/T_SETUP)*chassis.carSpeedy+((NowTick-StartTick)/T_SETUP)*(+chassis.carSpeedx))-carSpeedw;
	pidChassisWheelSpeed[3].SetPoint= k_xy*(-((NowTick-StartTick)/T_SETUP)*chassis.carSpeedy+((NowTick-StartTick)/T_SETUP)*(-chassis.carSpeedx))-carSpeedw;
	
	   if((NowTick-StartTick)>=T_SETUP)
	     {
         Goready_flag=0;  
         StartInitFlag=0;	 
	     }			
	}
	
	else if(Goready_flag==0)//进入平稳态
	{
	pidChassisWheelSpeed[0].SetPoint= k_xy*(+chassis.carSpeedy+chassis.carSpeedx)-carSpeedw;
	pidChassisWheelSpeed[1].SetPoint= k_xy*(+chassis.carSpeedy-chassis.carSpeedx)-carSpeedw;
	pidChassisWheelSpeed[2].SetPoint= k_xy*(-chassis.carSpeedy+chassis.carSpeedx)- carSpeedw;
	pidChassisWheelSpeed[3].SetPoint= k_xy*(-chassis.carSpeedy-chassis.carSpeedx)- carSpeedw;
	ABSready_flag=1; //准备好缓速刹车 注意：只有在满速后才缓速刹车
  StopInitFlag=0;
	}
	chassis.ABSLastcarSpeedx=chassis.carSpeedx;
	chassis.ABSLastcarSpeedy=chassis.carSpeedy;
}

else if((ABS(chassis.carSpeedx) <100)&&(ABS(chassis.carSpeedy) <100))//开始刹车
{

		
	if(ABSready_flag==1)  //只有在刹车状态下才开始计时
	{
		if(!StopInitFlag)
		{
		StopTick=xTaskGetTickCount();
		StopInitFlag=1;
		}
		NowTick=xTaskGetTickCount();
	//每个刹车状态只执行一次
	pidChassisWheelSpeed[0].SetPoint= k_xy * (+(1-(NowTick-StopTick)/T_ABS)*chassis.ABSLastcarSpeedy+(1-(NowTick-StopTick)/T_ABS)*(+chassis.ABSLastcarSpeedx))-carSpeedw;
	pidChassisWheelSpeed[1].SetPoint= k_xy * (+(1-(NowTick-StopTick)/T_ABS)*chassis.ABSLastcarSpeedy+(1-(NowTick-StopTick)/T_ABS)*(-chassis.ABSLastcarSpeedx))-carSpeedw;
	pidChassisWheelSpeed[2].SetPoint= k_xy * (-(1-(NowTick-StopTick)/T_ABS)*chassis.ABSLastcarSpeedy+(1-(NowTick-StopTick)/T_ABS)*(+chassis.ABSLastcarSpeedx))- carSpeedw;
	pidChassisWheelSpeed[3].SetPoint= k_xy * (-(1-(NowTick-StopTick)/T_ABS)*chassis.ABSLastcarSpeedy+(1-(NowTick-StopTick)/T_ABS)*(-chassis.ABSLastcarSpeedx))- carSpeedw;
	
	}
		
 if((NowTick-StopTick)>=T_ABS ||ABSready_flag==0 || ABS(ChassisMotorCanReceive[3].RealSpeed)<=200)  //完全刹住车
 {
 	pidChassisWheelSpeed[0].SetPoint= -carSpeedw;
	pidChassisWheelSpeed[1].SetPoint= -carSpeedw;
	pidChassisWheelSpeed[2].SetPoint= -carSpeedw;
	pidChassisWheelSpeed[3].SetPoint= -carSpeedw;
  ABSready_flag=0;   //刹车完毕，原地自旋
	StopInitFlag=0;
	 
	chassis.ABSLastcarSpeedx=0;
	chassis.ABSLastcarSpeedy=0;
 }
  Goready_flag=1;// 准备进入启动态
	StartInitFlag=0;
}

 }
/**********************************************************************************************************
*函 数 名: ABS_Cal
*功能说明: 缓速启停
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float test_watch;
char filter_en=1;
 void Filter_Cal(void)
{
//	if(filter_en)
//	{
  LowPass_SetChassis(&pidChassisWheelSpeed[0].SetPoint,k_xy*(+chassis.carSpeedy+(+chassis.carSpeedx))-carSpeedw);
	LowPass_SetChassis(&pidChassisWheelSpeed[1].SetPoint,k_xy*(+chassis.carSpeedy+(-chassis.carSpeedx))-carSpeedw);
	LowPass_SetChassis(&pidChassisWheelSpeed[2].SetPoint,k_xy*(-chassis.carSpeedy+(+chassis.carSpeedx))-carSpeedw);
	LowPass_SetChassis(&pidChassisWheelSpeed[3].SetPoint,k_xy*(-chassis.carSpeedy+(-chassis.carSpeedx))-carSpeedw);
//	}
//	else
//	{
//	pidChassisWheelSpeed[0].SetPoint=k_xy*(+chassis.carSpeedy+(+chassis.carSpeedx))-carSpeedw;
//	pidChassisWheelSpeed[1].SetPoint=k_xy*(+chassis.carSpeedy+(-chassis.carSpeedx))-carSpeedw;
//	pidChassisWheelSpeed[2].SetPoint=k_xy*(-chassis.carSpeedy+(+chassis.carSpeedx))-carSpeedw;
//	pidChassisWheelSpeed[3].SetPoint=k_xy*(-chassis.carSpeedy+(-chassis.carSpeedx))-carSpeedw;
//	}
}

/**********************************************************************************************************
*函 数 名: Method_Check
*功能说明: 根据不同的功率选择不同的控制参数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
extern volatile TickType_t SampleTick;
TickType_t MyNowTick;
void Method_Check(void)
{
	short i;
	short choose_PN = 0;
	
	static short PN;
	static short last_PN;
	
//	MyNowTick=xTaskGetTickCount();
//	if(MyNowTick<SampleTick+60)
//	{
////	MyMaxPower=JudgeReceive.MaxPower+20
//		MyMaxPower=JudgeReceive.MaxPower;

//	}
//	else
//	{
//	MyMaxPower=JudgeReceive.MaxPower;
//	}
	MyMaxPower=JudgeReceive.MaxPower;
	for(choose_PN = 1;choose_PN < METHOD_NUM; choose_PN++)		//0用于储存默认参数
	{
		if(Power_method[choose_PN].Actual_P_max == MyMaxPower)
		{
			PN = choose_PN;
			break;
		}	
	}
	if(choose_PN >= METHOD_NUM)	//没找到匹配的最大功率参数
		PN = PN_Default;	//默认参数

	if(last_PN != choose_PN)			//说明参数有变化
	{
		Actual_P_max = Power_method[PN].Actual_P_max;
		Self_Protect_Limit = Power_method[PN].Self_Protect_Limit;
		k_BAT = Power_method[PN].k_BAT;
		Excess_P_max = Power_method[PN].Excess_P_max;
		CurrentMax = Power_method[PN].CurrentMax;
		Follow_W = Power_method[PN].Follow_W;
		for(i=0; i<4; i++)
		{
			pidChassisWheelSpeed[i].OutMax = CurrentMax;
		}
	}
	last_PN = PN;
}

/**********************************************************************************************************
*函 数 名: Chassis_Speed_Cal
*功能说明: 根据xyw向速度计算目标速度值
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
short test_Self_Protect_Limit = 3600;
float test_k_BAT = 1.0f;
short pre_in[2];
void Chassis_Speed_Cal(void)
{
	static float k_CAP = 2.0f;
	static short Angular_Velocity;
	float rotation_lim=1.0f;
	
//xy向速度与w向速度配比
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			k_xy = 0;
			carSpeedw = 0;
		break;
		
		case Chassis_Act_Mode:
						
		if((ABS(chassis.carSpeedx)>100) && (ABS(chassis.carSpeedy) >100))
				k_xy = 1.0f;
			else
				k_xy = 3;
			
		if((ABS(chassis.carSpeedx) <100) && (ABS(chassis.carSpeedy)>100))
				k_xy =1.5f;
		
//    if((ABS(chassis.carSpeedx)>500)  && (ABS(chassis.carSpeedy)<500))
//    {
//			if(ABS(chassis.carSpeedw)>2000)
//			{
//				k_xy =1.5f;
//				chassis.carSpeedw*=0.7;
//			}
//		}			

		carSpeedw =LIMIT_MAX_MIN(chassis.carSpeedw,Follow_W, -Follow_W);

		break;
		
		case Chassis_SelfProtect_Mode:
		{
				if((ABS(chassis.carSpeedx) >100) || (ABS(chassis.carSpeedy) >100))
				{
					 if((ABS(chassis.carSpeedx) >100) && (ABS(chassis.carSpeedy) >100))
					 {
						k_xy = 1.5f;
						rotation_lim=0.85f;
					 }
					 else
					 {
					 	k_xy = 1.8f;
						rotation_lim=0.98f;				 
					 }	 
				}
				else
				{
					k_xy=0.0f;
					rotation_lim=1.0f;
				}		
				if(PowerState == BAT)
				{
//				carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, test_Self_Protect_Limit, -test_Self_Protect_Limit);
					carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, rotation_lim*Self_Protect_Limit, -rotation_lim*Self_Protect_Limit);
				}
				else if(PowerState == CAP)
				{
					carSpeedw = chassis.carSpeedw;
				}
			}
		break;
			
		case Chassis_Solo_Mode:
		{
			if((ABS(chassis.carSpeedx) >100) && (ABS(chassis.carSpeedy) >100))
				k_xy = 1.4f;
			else
				k_xy = 2;
			
			Angular_Velocity = 0.026f*(ChassisMotorCanReceive[0].RealSpeed+ChassisMotorCanReceive[1].RealSpeed+ChassisMotorCanReceive[2].RealSpeed+ChassisMotorCanReceive[3].RealSpeed);
			pidChassisPosition_Speed.SetPoint = -chassis.carSpeedw;
			carSpeedw = PID_Calc(&pidChassisPosition_Speed, Angular_Velocity); 
		}
		break;
		
		case Chassis_NoFollow_Mode:
			k_xy = 2.5f;
			carSpeedw = 0;
		break;
		
		default: 
			k_xy = 0;
			carSpeedw = 0;
			break;
	}
	
	
//根据不同功率 对应不同xy向速度系数
	if(PowerState == BAT)
	{
//		k_xy *= test_k_BAT;
		k_xy *= k_BAT;
	}
	else if(PowerState == CAP)
	{
		k_xy *= k_CAP;
	}
	//ABS_Cal();
	Filter_Cal();
//  pre_in[0]=pidChassisWheelSpeed[0].SetPoint;
//	pre_in[1]=pidChassisWheelSpeed[1].SetPoint;
}

/**********************************************************************************************************
*函 数 名: PowerLimit
*功能说明: 功率限制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
float test_W_Chassis_t = 0;	//估算功率修正
//short test_Excess_P_max = 320;
//short test_speed_out[4];
short test_Jump[4];
//short test_wheel_send[4];
float	W_k[10] = {0.95f,0.95f,0.95f,0.9f,0.9f,0.85f,0.85f,0.8f,0.8f,0.7f};
void PowerLimit(void)
{
	short i = 0;
	short DescendFlag = 0;
	
	float W_Chassis_t = 0;//底盘功率(计算用)
	
	static short EnergyMargin = 15;		//留有的缓存能量余量
	static float My_P_max;				//计算的当前最大功率
	static float k_ExcessPower;
		
//设置最大功率
	if(JudgeReceive.remainEnergy <= EnergyMargin)
	{
		My_P_max = MyMaxPower*0.8f;
	}
	else
	{
		My_P_max = LIMIT_MAX_MIN(((Excess_P_max-MyMaxPower)*(JudgeReceive.remainEnergy-EnergyMargin)/(60-EnergyMargin)+MyMaxPower), Excess_P_max, MyMaxPower);
//		My_P_max = LIMIT_MAX_MIN(Excess_P_max*(JudgeReceive.remainEnergy-EnergyMargin)/(60-EnergyMargin), test_Excess_P_max, JudgeReceive.MaxPower);
	}
	
//接收电流滤波
	Current_Filter_Excu();
	
	for(i = 0;i < 4;i ++)
	{
		Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
		Current_Set_Jump();
		Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
		
//		test_speed_out[i] = Pid_Current[i].SetPoint;
//		test_current_out[i] = Current_Change[i];
		
		if(Set_Jump[i] == 0)
		{
			Current_f[i] += Current_Change[i];
		}
		else if(Set_Jump[i] == 1)
		{
			Current_f[i] = Pid_Current[i].SetPoint;
		}
		WheelCurrentSend[i] = Current_f[i];	
//计算当前功率
		W_Chassis_t += ABS(Pid_Current[i].SetPoint)*k_chassis_t;//功率计算20*24/16384 = 0.029
	}
	test_W_Chassis_t = W_Chassis_t;	
	
//当前功率超过最大功率 分次削减速度,最多削减7次
	while(W_Chassis_t > My_P_max && DescendFlag < 10)
	{
		W_Chassis_t = 0;
		
		for(i=0;i<4;i++)//通过削减速度减小电流值
		{
			pidChassisWheelSpeed[i].SetPoint *= W_k[DescendFlag];
			
			//速度环+电流环
			Pid_Current[i].SetPoint = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
			Current_Filter_Excu();
			Current_Set_Jump();
			Current_Change[i] = PID_Calc(&Pid_Current[i],Flow[i]);
			if(Set_Jump[i] == 0)
			{
				Current_f[i] += Current_Change[i];
			}
			else if(Set_Jump[i] == 1)
			{
				Current_f[i] = Pid_Current[i].SetPoint;
			}
			WheelCurrentSend[i] = Current_f[i];
			
			W_Chassis_t += ABS(Pid_Current[i].SetPoint)*k_chassis_t;
		}	
		DescendFlag++;
	}
	
//超出迭代次数，估算的功率仍超出：直接削减电流	
	if(W_Chassis_t > My_P_max)//超出迭代次数，直接削减电流
	{
			k_ExcessPower = My_P_max/W_Chassis_t;//超功率系数
			for(i=0;i<4;i++)
			{
				WheelCurrentSend[i] *= k_ExcessPower;
				W_Chassis_t += ABS(WheelCurrentSend[i])*k_chassis_t;
			}
			test_W_Chassis_t = W_Chassis_t;
	}
}

/**********************************************************************************************************
*函 数 名: Chassis_CurrentPid_Cal
*功能说明: 底盘操作
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	int i=0;
	Method_Check();			//设置当前功率参数
	Chassis_Speed_Cal();//根据xyw向速度计算目标速度值
	if(PowerState == BAT)
	{
		PowerLimit();
	}
	else if(PowerState == CAP)
	{
		for(i=0;i<4;i++)
		{
			WheelCurrentSend[i] = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
		}
	}
	else
	{
		for(i=0;i<4;i++)
		{
			WheelCurrentSend[i] = 0;
		}
	}
	//发送电流值在任务函数中
}

/**********************************************************************************************************
*函 数 名: Current_Filter_Excu
*功能说明: 将四个轮子的电流反馈值分别滤波
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Current_Filter_Excu(void)
{
	for(int i = 0;i < 4;i++)
	{
		Input[i] = (float)ChassisMotorCanReceive[i].Current; 
	}
	Fir(Input,Output);
}

/**********************************************************************************************************
*函 数 名: Current_Set_Jump
*功能说明: 判断是否用电流环
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//short Current_Set_Flag;			//四个轮子都可用电流环时，才用电流环
//210513 分别各自开电流环 效果更好
void Current_Set_Jump(void)
{
	int i;
	for(i = 0;i < 4;i ++)
	{
		if(F405.Chassis_Flag == Chassis_Act_Mode)		//正常模式
		{
			if(ABS(Pid_Current[i].SetPoint - Pid_Current[i].SetPointLast) > 1500)
				Set_Jump[i] = 0;   // 0
			else
				Set_Jump[i] = 1;
		}
		else
			Set_Jump[i] = 1;
	}
}
/**********************************************************************************************************
*函 数 名: Chassis_Power_Control_Init
*功能说明: 底盘功率限制参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Chassis_Power_Control_Init(void)
{
	int num = 0;
	/****************默认参数********************/
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max = 1500;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************40W********************/
	num++;
	Power_method[num].Actual_P_max = 40;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.6f;
	Power_method[num].Excess_P_max = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************45W********************/
	num++;
	Power_method[num].Actual_P_max = 45;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************50W********************/
	num++;
	Power_method[num].Actual_P_max = 50;
	Power_method[num].Self_Protect_Limit = 2800;
	Power_method[num].k_BAT = 0.7f;   //0.6f
	Power_method[num].Excess_P_max = 1500;  //600
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 5000;
	/****************60W********************/
	num++;
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 3400;  //小陀螺控制转速
	Power_method[num].k_BAT = 0.75f;   // 0.9f              //xy向速度系数
	Power_method[num].Excess_P_max = 2000;  //3000   750  //太小起步较慢匀速直行时会缓存能量太少会无法转弯，较大的值可以保证起步后可以保留缓存能量可以转弯，但连续转弯有限
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************80W********************/
	num++;
	Power_method[num].Actual_P_max = 80;
	Power_method[num].Self_Protect_Limit = 4500;
	Power_method[num].k_BAT = 0.95f;   //
	Power_method[num].Excess_P_max = 1400;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W =5000;
	/****************100W********************/
	num++;
	Power_method[num].Actual_P_max = 100;
	Power_method[num].Self_Protect_Limit = 5400;
	Power_method[num].k_BAT = 1.1f;
	Power_method[num].Excess_P_max = 1200;
	Power_method[num].CurrentMax = 12000;
	Power_method[num].Follow_W = 5600;
	/****************120W********************/
	num++;
	Power_method[num].Actual_P_max = 120;
	Power_method[num].Self_Protect_Limit = 6000;
	Power_method[num].k_BAT = 2.3f;
	Power_method[num].Excess_P_max = 450;
	Power_method[num].CurrentMax = 10000;
		Power_method[num].Follow_W = 10000;
}

/**********************************************************************************************************
*函 数 名: Pid_ChassisWheelInit
*功能说明: 底盘XY向运动PID参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Pid_ChassisWheelInit(void)
{
	short i=0;
	
	for(i = 0;i < 4;i ++)
	{
		//电流环
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//速度环
		pidChassisWheelSpeed[i].P = 12.0f;
		pidChassisWheelSpeed[i].I = 0.0f;
		pidChassisWheelSpeed[i].D = 0.0f;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 10000.0f;	

		//跟随速度环
		pidChassisPosition_Speed.P = 3.0f;//w向速度环
		pidChassisPosition_Speed.I = 0.5f;
		pidChassisPosition_Speed.D = 0.0f;
		pidChassisPosition_Speed.IMax = 200.0f;
		pidChassisPosition_Speed.OutMax = 9000.0f;//8000.0f
	}
}

/**********************************************************************************************************
*函 数 名: HeatControl
*功能说明: 热量控制
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
//变量定义
extern JudgeReceive_t JudgeReceive;

uint16_t HeatMax17, HeatCool17;
const short BulletHeat17 = 10;

short CurHeat17, LastHeat17, AvailableHeat17; //当前热量， 上一次热量, 自行计算热量

uint16_t Shooted17Cnt;	//一周期内已打出子弹数
uint16_t AvailableBullet17;	//下一周期允许打弹数

char ShootAbleFlag;

void HeatControl(void)
{
	if(JudgeReceive.HeatUpdateFlag == 1)	//热量更新
	{
		Shooted17Cnt = 0;
		AvailableHeat17 = LIMIT_MAX_MIN(HeatMax17 - CurHeat17 + HeatCool17,HeatMax17,0);
//		AvailableHeat17 = HeatMax17 - CurHeat17;
		if(JudgeReceive.ShootCpltFlag == 1)	//检测到发弹。为热量更新后打出的子弹
		{
			AvailableHeat17 = LIMIT_MAX_MIN(AvailableHeat17 - BulletHeat17,HeatMax17,0);
			JudgeReceive.ShootCpltFlag = 0;	//已处理完本次发弹
		}
		AvailableBullet17 = AvailableHeat17 / BulletHeat17;
		ShootAbleFlag = (AvailableBullet17 < 1)?0:1;		
	}	
	
	else if((JudgeReceive.ShootCpltFlag == 1) && (JudgeReceive.HeatUpdateFlag == 0))	//热量没有更新，但检测到发弹
	{
		JudgeReceive.ShootCpltFlag = 0;		//已处理完本次发弹
		Shooted17Cnt++;		//发射了一发子弹
		ShootAbleFlag = (Shooted17Cnt >= AvailableBullet17)?0:1;		
	}
}

/**********************************************************************************************************
*函 数 名: HeatUpdate
*功能说明: 热量更新
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
const float HeatControlThreshold = 0.8f;   	//开启热量控制的阈值

void HeatUpdate(void)
{
//	HeatMax17 = JudgeReceive.HeatMax17 + (short)(1250/JudgeReceive.maxHP) - BulletHeat17;		//榨干热量.jpg
	HeatMax17 = JudgeReceive.HeatMax17 - BulletHeat17;		//榨干热量
	HeatCool17 = JudgeReceive.HeatCool17/10;
	CurHeat17 = JudgeReceive.shooterHeat17;
	
	if(CurHeat17 != LastHeat17)
	{
		JudgeReceive.HeatUpdateFlag = 1;
		JudgeReceive.ShootCpltFlag = 0;			//热量更新则将发射标志位清零(没有代处理的打弹)
	}
	
	if(CurHeat17 < HeatControlThreshold*HeatMax17)
	{
		ShootAbleFlag = 1;
		JudgeReceive.ShootCpltFlag = 0;
	}
	else
	{
		if((JudgeReceive.ShootCpltFlag == 1) || (JudgeReceive.HeatUpdateFlag == 1))
		HeatControl();
	}
	
	JudgeReceive.HeatUpdateFlag = 0;		//已处理完本次热量更新
	LastHeat17 = CurHeat17;
	F105.IsShootAble = ShootAbleFlag;
}

/**********************************************************************************************************
*函 数 名: BuildF105
*功能说明: 构建要传给上层板的F105结构体
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void BuildF105(void)
{
	if(JudgeReceive.robot_id < 10)
		F105.RobotRed = 1;
	else
		F105.RobotRed = 0;			//0为蓝色，1为红色
	switch(JudgeReceive.BulletSpeedMax17)
	{
		case 15:
		{
			F105.BulletSpeedLevel = 0;
			break;
		}
		case 18:
		{
			F105.BulletSpeedLevel = 1;
			break;
		}
		case 30:
		{
			F105.BulletSpeedLevel = 2;
			break;
		}
		default:
		{
			F105.BulletSpeedLevel = 0;
			break;
		}
	}
	F105.ChassisSpeedw=0.026f*(ChassisMotorCanReceive[0].RealSpeed+ChassisMotorCanReceive[1].RealSpeed+ChassisMotorCanReceive[2].RealSpeed+ChassisMotorCanReceive[3].RealSpeed);

}


/**********************************************************************************************************
*函 数 名: Chassis_task
*功能说明: 底盘任务
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Chassis_high_water;
TickType_t TestNowTick=0;
void Chassis_task(void *pvParameters)
{
  portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	
  while (1) {
    TestNowTick=xLastWakeTime = xTaskGetTickCount();
		//电容充放电控制
	if(JudgeReceive.remainEnergy < 20 && PowerState == BAT)
    Charge_Off;
	else
		Charge_On;
		
		//功率限制
    Chassis_CurrentPid_Cal();
	
		ChassisCan1Send(WheelCurrentSend[0],WheelCurrentSend[1],WheelCurrentSend[2],WheelCurrentSend[3]); 
		
		//热量控制
		HeatUpdate();
		BuildF105();
		Can2Send0(&F105);
		
   //DataView
   	VOFA_Send();
	
		IWDG_Feed();//喂狗		
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
