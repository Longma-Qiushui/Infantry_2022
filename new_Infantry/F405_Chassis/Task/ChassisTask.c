/**********************************************************************************************************
 * @�ļ�     ChassisTask.c
 * @˵��     ���̿���+��������
 * @�汾  	 V3.0
 * @����     ��ҵȨ
 * @����     2021.4.26
**********************************************************************************************************/
#include "main.h"

#define CAP_MAX_W   7000
float k_CAP = 2.5f;
/*----------------------------------�ڲ�����---------------------------*/
short WheelCurrentSend[4];
short Set_Jump[4] = {0};
float Current_Change[4] = {0};//����������
float Current_f[4] = {0};//�������f
float Flow[4] = {0};//ʵ�ʵ���f
float speed[4] = {0};//ʵ���ٶ�f

//��������ϵ�� PowerLimit
short Actual_P_max;						//ʵ�������
short Self_Protect_Limit;			//С����ת������
float k_BAT;
short Excess_P_max;
short CurrentMax;
short Follow_W;

const float k_chassis_t = 0.0168f;			
//24*0.0007 24��chassis�������ѹ �������24 һ���21��
//0.0007��������Ƶ���ֵ��Ӧ�ĵ�λchassis���������

/*----------------------------------�ṹ��-----------------------------*/
Pid_Typedef Pid_Current[4];
Pid_Typedef pidChassisWheelSpeed[4];
Pid_Typedef pidChassisPosition_Speed;
F105_Typedef F105;
Power_Typedef Power_method[METHOD_NUM];

/*----------------------------------�ⲿ����---------------------------*/
extern JudgeReceive_t JudgeReceive;
extern ChassisSpeed_t chassis;
extern RM820RReceive_Typedef ChassisMotorCanReceive[4];
extern F405_typedef F405;
extern enum POWERSTATE_Typedef PowerState;
extern char Robot_ID;
extern float output_fil;
extern float Input[4];
extern float Output[4];
extern char slow_flag;

float k_xy = 3.0f,y_lim=1.0f;
short carSpeedw = 0;
float ABSready_flag=0; 
float Goready_flag=0;
float T_ABS=1000.0f;//ɲ��ʱ��
float T_SETUP=800.0f;//����ʱ��
extern char output_filter;
extern enum CHARGESTATE_Typedef ChargeState;
uint16_t  MyMaxPower=0;
/**********************************************************************************************************
*�� �� ��: ABS_Cal
*����˵��: ������ͣ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//void ABS_Cal(void)
//{
//static TickType_t StartTick=0,NowTick;
//static TickType_t StopTick=0;
//static char StartInitFlag=0,StopInitFlag=0;
//if(chassis.carSpeedx>50||chassis.carSpeedy>50) //������ʻ����ʼ�������
//{
//	
//	if(Goready_flag==1)//��ʼ������������̬
//	{
//	if(!StartInitFlag)
//	{
//	StartTick=xTaskGetTickCount();
//	StartInitFlag=1;
//	}
//  NowTick=xTaskGetTickCount();

//	   if((NowTick-StartTick)>=T_SETUP)
//	     {
//         Goready_flag=0;  
//         StartInitFlag=0;	 
//	     }	
//		 else
//		 {
//		 output_filter = 0;
//		 }
//	}
//	
//	else if(Goready_flag==0)//����ƽ��̬
//	{

//	ABSready_flag=1; //׼���û���ɲ�� ע�⣺ֻ�������ٺ�Ż���ɲ��
//  StopInitFlag=0;
//  output_filter = 1;
//	}

//}

//else if((ABS(chassis.carSpeedx) <50)&&(ABS(chassis.carSpeedy) <50))//��ʼɲ��
//{	
//	if(ABSready_flag==1)  //ֻ����ɲ��״̬�²ſ�ʼ��ʱ
//	{
//		if(!StopInitFlag)
//		{
//		StopTick=xTaskGetTickCount();
//		StopInitFlag=1;
//		}
//		NowTick=xTaskGetTickCount();
//		output_filter = 0;
//	}
//		
// if((NowTick-StopTick)>=T_ABS ||ABSready_flag==0 || ABS(ChassisMotorCanReceive[3].RealSpeed)<=200)  //��ȫɲס��
// {
//  ABSready_flag=0;   //ɲ�����
//	StopInitFlag=0;
// }
//  Goready_flag=1;// ׼����������̬
//	StartInitFlag=0;
//}

// }
/**********************************************************************************************************
*�� �� ��: ABS_Cal
*����˵��: ������ͣ
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Method_Check
*����˵��: ���ݲ�ͬ�Ĺ���ѡ��ͬ�Ŀ��Ʋ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
extern volatile TickType_t SampleTick;
TickType_t MyNowTick;
void Method_Check(void)
{
	short i;
	short choose_PN = 0;
	
	static short PN;
	static short last_PN;
	
  if(F405.Chassis_Flag==Chassis_SelfProtect_Mode)
	{
	output_filter=1;
	}
	else 
	{
	output_filter=0;
	}
	
	
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
	for(choose_PN = 1;choose_PN < METHOD_NUM; choose_PN++)		//0���ڴ���Ĭ�ϲ���
	{
		if(Power_method[choose_PN].Actual_P_max == MyMaxPower)
		{
			PN = choose_PN;
			break;
		}	
	}
	if(choose_PN >= METHOD_NUM)	//û�ҵ�ƥ�������ʲ���
		PN = PN_Default;	//Ĭ�ϲ���

	if(last_PN != choose_PN)			//˵�������б仯
	{
		Actual_P_max = Power_method[PN].Actual_P_max;
		Self_Protect_Limit = Power_method[PN].Self_Protect_Limit;
		k_BAT = Power_method[PN].k_BAT;
		CurrentMax = Power_method[PN].CurrentMax;
		Follow_W = Power_method[PN].Follow_W;
		for(i=0; i<4; i++)
		{
			pidChassisWheelSpeed[i].OutMax = CurrentMax;
		}
	}
	
		if(output_filter==1)
		{
		Excess_P_max = Power_method[PN].Excess_P_max_P;
		}
		else 
		{
		Excess_P_max = Power_method[PN].Excess_P_max_J;
		}
		
	last_PN = PN;
	if(PowerState==CAP)
	{
	CurrentMax = 16380.0f;
	}
}

/**********************************************************************************************************
*�� �� ��: Chassis_Speed_Cal
*����˵��: ����xyw���ٶȼ���Ŀ���ٶ�ֵ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
short test_Self_Protect_Limit = 3600;
float test_k_BAT = 1.0f;
short pre_in[2];
char SelfProtect_Cross_Flag;
void Chassis_Speed_Cal(void)
{
	static short Angular_Velocity;
	float rotation_lim=1.0f;	
	
//xy���ٶ���w���ٶ����
	switch(F405.Chassis_Flag)
	{
		case Chassis_Powerdown_Mode:
			k_xy = 0;
			carSpeedw = 0;
		break;
		
		case Chassis_Act_Mode:
		case Chassis_Jump_Mode:
			
		if((ABS(chassis.carSpeedx)>500) && (ABS(chassis.carSpeedy) >500))
				k_xy = 1.5f;
			else
				k_xy = 3;
			
		if((ABS(chassis.carSpeedx) <500) && (ABS(chassis.carSpeedy)>500))
				k_xy =2.0f;
				
   if(PowerState==BAT)
	 {	
		 carSpeedw =LIMIT_MAX_MIN(chassis.carSpeedw,Follow_W, -Follow_W);
	 }
	 else 
	 {
	   carSpeedw = chassis.carSpeedw;
	 }
	 
		break;
		
		case Chassis_SelfProtect_Mode:
		{
				if((ABS(chassis.carSpeedx) >100) || (ABS(chassis.carSpeedy) >100))
				{
					 if((ABS(chassis.carSpeedx) >100) && (ABS(chassis.carSpeedy) >100))
					 {
//						 if(MyMaxPower == 100)
//						 {
//						 	k_xy = 0.8f;              // 100W ����
//							rotation_lim=0.9f;
//						 }
//						 else
//						 {
							k_xy = 1.7f;            // 60W 80W ����
							rotation_lim=0.80f;
//						 }
					 }
					 if(SelfProtect_Cross_Flag)
					 {
//						 if(MyMaxPower == 100)
//						 {
//					 	k_xy = 0.8f;               // 100W ����
//						rotation_lim=0.9f;
//						 }
//						 else
//						 {
					 	k_xy = 1.85f;             // 60W 80W����
						rotation_lim=0.75f;
//						 }
					 }
				}
				else
				{
					k_xy=0.0f;
					rotation_lim=1.0f;
				}		
				if(PowerState == CAP)
				{
					carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw,CAP_MAX_W,-CAP_MAX_W);
				}
				else 
				{
				carSpeedw = LIMIT_MAX_MIN(chassis.carSpeedw, rotation_lim*Self_Protect_Limit, -rotation_lim*Self_Protect_Limit);
				}
			}
		break;
			
		case Chassis_Solo_Mode:
		{
			if((ABS(chassis.carSpeedx) >100) && (ABS(chassis.carSpeedy) >100))
				k_xy = 1.4f;
			else
				k_xy = 2;
			carSpeedw = chassis.carSpeedw; 
		}
		break;
		
		default: 
			k_xy = 0;
			carSpeedw = 0;
			break;
	}
	
	
//���ݲ�ͬ���� ��Ӧ��ͬxy���ٶ�ϵ��
	if(PowerState == CAP)
	{
		k_xy *= k_CAP;
	}
	else 
	{
		k_xy *= k_BAT;
	}
	
	// ȡ��ʱ�����ƶ�
	if(slow_flag)
	{
	  k_xy = 0.5;
	}
	
//	ABS_Cal();
	Filter_Cal();

}

/**********************************************************************************************************
*�� �� ��: PowerLimit
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
float test_W_Chassis_t = 0;	//���㹦������
//short test_Excess_P_max = 320;
//short test_speed_out[4];
short test_Jump[4];
//short test_wheel_send[4];
float	W_k[10] = {0.95f,0.95f,0.95f,0.9f,0.9f,0.85f,0.85f,0.8f,0.8f,0.7f};
void PowerLimit(void)
{
	short i = 0;
	short DescendFlag = 0;
	
	float W_Chassis_t = 0;//���̹���(������)
	
	static short EnergyMargin = 15;		//���еĻ�����������
	static float My_P_max;				//����ĵ�ǰ�����
	static float k_ExcessPower;
		
//���������
	if(JudgeReceive.remainEnergy <= EnergyMargin)
	{
		My_P_max = MyMaxPower*0.8f;
	}
	else
	{
		My_P_max = LIMIT_MAX_MIN(((Excess_P_max-MyMaxPower)*(JudgeReceive.remainEnergy-EnergyMargin)/(60-EnergyMargin)+MyMaxPower), Excess_P_max, MyMaxPower);
	}
	
//���յ����˲�
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
//���㵱ǰ����
		W_Chassis_t += ABS(Pid_Current[i].SetPoint)*k_chassis_t;//���ʼ���20*24/16384 = 0.029
	}
	test_W_Chassis_t = W_Chassis_t;	
	
//��ǰ���ʳ�������� �ִ������ٶ�,�������7��
	while(W_Chassis_t > My_P_max && DescendFlag < 10)
	{
		W_Chassis_t = 0;
		
		for(i=0;i<4;i++)//ͨ�������ٶȼ�С����ֵ
		{
			pidChassisWheelSpeed[i].SetPoint *= W_k[DescendFlag];
			
			//�ٶȻ�+������
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
	
//������������������Ĺ����Գ�����ֱ����������	
	if(W_Chassis_t > My_P_max)//��������������ֱ����������
	{
			k_ExcessPower = My_P_max/W_Chassis_t;//������ϵ��
			for(i=0;i<4;i++)
			{
				WheelCurrentSend[i] *= k_ExcessPower;
				W_Chassis_t += ABS(WheelCurrentSend[i])*k_chassis_t;
			}
			test_W_Chassis_t = W_Chassis_t;
	}
}

/**********************************************************************************************************
*�� �� ��: Chassis_CurrentPid_Cal
*����˵��: ���̲���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_CurrentPid_Cal(void)
{
	int i=0;
	Method_Check();			//���õ�ǰ���ʲ���
	Chassis_Speed_Cal();//����xyw���ٶȼ���Ŀ���ٶ�ֵ
	
	if(PowerState == CAP)
	{
		
		for(i=0;i<4;i++)
		{
			WheelCurrentSend[i] = PID_Calc(&pidChassisWheelSpeed[i],ChassisMotorCanReceive[i].RealSpeed);
		}
	}
	else
	{
		PowerLimit();
	}
	//���͵���ֵ����������
}

/**********************************************************************************************************
*�� �� ��: Current_Filter_Excu
*����˵��: ���ĸ����ӵĵ�������ֵ�ֱ��˲�
*��    ��: ��
*�� �� ֵ: ��
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
*�� �� ��: Current_Set_Jump
*����˵��: �ж��Ƿ��õ�����
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//short Current_Set_Flag;			//�ĸ����Ӷ����õ�����ʱ�����õ�����
//210513 �ֱ���Կ������� Ч������
void Current_Set_Jump(void)
{
	int i;
	for(i = 0;i < 4;i ++)
	{
		if(F405.Chassis_Flag == Chassis_Act_Mode)		//����ģʽ
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
*�� �� ��: Chassis_Power_Control_Init
*����˵��: ���̹������Ʋ�����ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Chassis_Power_Control_Init(void)
{
	int num = 0;
	switch(Robot_ID)
{
/****************************************  3�ų�   ************************************************************/
		case 3:
{	/****************Ĭ�ϲ���********************/       //3�ų�
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max_J = 300;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************40W********************/
	num++;
	Power_method[num].Actual_P_max = 40;                 //3�ų�
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.6f;
	Power_method[num].Excess_P_max_J = 200;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************45W********************/
	num++;
	Power_method[num].Actual_P_max = 45;                //3�ų�
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max_J = 250;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************50W********************/
	num++;
	Power_method[num].Actual_P_max = 50;               //3�ų�
	Power_method[num].Self_Protect_Limit = 2800;
	Power_method[num].k_BAT = 0.7f;   //0.6f
	Power_method[num].Excess_P_max_J = 250;  //600
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 5000;
	/****************60W********************/
	num++;                                             //3�ų�
	Power_method[num].Actual_P_max = 60;                   
	Power_method[num].Self_Protect_Limit = 3300;  //С���ݿ���ת��
	Power_method[num].k_BAT = 0.75f;   // 0.9f              //xy���ٶ�ϵ��
	Power_method[num].Excess_P_max_J = 300;  //�����ƶ�ʱ //̫С�𲽽�������ֱ��ʱ�Ỻ������̫�ٻ��޷�ת�䣬�ϴ��ֵ���Ա�֤�𲽺���Ա���������������ת�䣬������ת������
	Power_method[num].Excess_P_max_P = 2000; //С�����ƶ�ʱ����ԣ��
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************80W********************/
	num++;
	Power_method[num].Actual_P_max = 80;               //3�ų�
	Power_method[num].Self_Protect_Limit = 4200;
	Power_method[num].k_BAT = 0.95f;   //
	Power_method[num].Excess_P_max_J = 330;  //1350
	Power_method[num].Excess_P_max_P = 1000;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W =5000;
	/****************100W********************/
	num++;                                             //3�ų�
	Power_method[num].Actual_P_max = 100;
	Power_method[num].Self_Protect_Limit = 4700;
	Power_method[num].k_BAT = 1.05f;
	Power_method[num].Excess_P_max_J = 370;
	Power_method[num].Excess_P_max_P = 1100;
	Power_method[num].CurrentMax = 12000;
	Power_method[num].Follow_W = 5600;
	/****************120W********************/
	num++;                                            //3�ų�
	Power_method[num].Actual_P_max = 120;
	Power_method[num].Self_Protect_Limit = 6000;
	Power_method[num].k_BAT = 2.3f;
	Power_method[num].Excess_P_max_J = 450;
	Power_method[num].Excess_P_max_P = 1000;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 10000;
}break;
///****************************************  4�ų�   ************************************************************/
case 4:
{	/****************Ĭ�ϲ���********************/       //4�ų�
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max_J = 300;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************40W********************/
	num++;
	Power_method[num].Actual_P_max = 40;                 //4�ų�
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.6f;
	Power_method[num].Excess_P_max_J = 200;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************45W********************/
	num++;
	Power_method[num].Actual_P_max = 45;                //4�ų�
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max_J = 250;
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************50W********************/
	num++;
	Power_method[num].Actual_P_max = 50;               //4�ų�
	Power_method[num].Self_Protect_Limit = 2800;
	Power_method[num].k_BAT = 0.7f;   //0.6f
	Power_method[num].Excess_P_max_J = 250;  //600
	Power_method[num].Excess_P_max_P = 1500;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 5000;
	/****************60W********************/
	num++;                                             //4�ų�
	Power_method[num].Actual_P_max = 60;                   
	Power_method[num].Self_Protect_Limit = 3500;  //С���ݿ���ת��
	Power_method[num].k_BAT = 0.75f;   // 0.9f              //xy���ٶ�ϵ��
	Power_method[num].Excess_P_max_J = 300;  //3000   750  //̫С�𲽽�������ֱ��ʱ�Ỻ������̫�ٻ��޷�ת�䣬�ϴ��ֵ���Ա�֤�𲽺���Ա���������������ת�䣬������ת������
	Power_method[num].Excess_P_max_P = 2500;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
	/****************80W********************/
	num++;
	Power_method[num].Actual_P_max = 80;               //4�ų�
	Power_method[num].Self_Protect_Limit = 4300;
	Power_method[num].k_BAT = 0.95f;   //
	Power_method[num].Excess_P_max_J = 330;  //1350
	Power_method[num].Excess_P_max_P = 1600;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W =5000;
	/****************100W********************/
	num++;                                             //4�ų�
	Power_method[num].Actual_P_max = 100;
	Power_method[num].Self_Protect_Limit = 4900;
	Power_method[num].k_BAT = 1.05f;
	Power_method[num].Excess_P_max_J = 370;
	Power_method[num].Excess_P_max_P = 1400;
	Power_method[num].CurrentMax = 12000;
	Power_method[num].Follow_W = 5600;
	/****************120W********************/
	num++;                                            //4�ų�
	Power_method[num].Actual_P_max = 120;
	Power_method[num].Self_Protect_Limit = 6000;
	Power_method[num].k_BAT = 2.3f;
	Power_method[num].Excess_P_max_J = 450;
	Power_method[num].Excess_P_max_P = 1000;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 10000;
}break;
///****************************************  5�ų�   ************************************************************/
		case 5:
{	/****************Ĭ�ϲ���********************/       //5�ų�
/****************Ĭ�ϲ���********************/
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 4000;
	Power_method[num].k_BAT = 0.75f;
	Power_method[num].Excess_P_max_J = 400;
	Power_method[num].Excess_P_max_P = 1200;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************40W********************/
	num++;
	Power_method[num].Actual_P_max = 40;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.8f;
	Power_method[num].Excess_P_max_J = 350;
	Power_method[num].Excess_P_max_P = 1050;
	Power_method[num].CurrentMax = 8000;
		Power_method[num].Follow_W = 4000;
	/****************45W********************/
	num++;
	Power_method[num].Actual_P_max = 45;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.9f;
	Power_method[num].Excess_P_max_J = 300;
	Power_method[num].Excess_P_max_P = 1050;
	Power_method[num].CurrentMax = 8000;
		Power_method[num].Follow_W = 4000;
	/****************50W********************/
	num++;
	Power_method[num].Actual_P_max = 50;
	Power_method[num].Self_Protect_Limit = 3300;
	Power_method[num].k_BAT = 0.6f;   //0.6f
	Power_method[num].Excess_P_max_J = 300;  //600
	Power_method[num].Excess_P_max_P = 950;
	Power_method[num].CurrentMax = 8000;
	Power_method[num].Follow_W = 4000;
	/****************60W********************/
	num++;
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 4000;
	Power_method[num].k_BAT = 0.75f;
	Power_method[num].Excess_P_max_J = 300;
	Power_method[num].Excess_P_max_P = 1500;
  Power_method[num].CurrentMax = 12000;
	Power_method[num].Follow_W = 5200;
	/****************80W********************/
	num++;
	Power_method[num].Actual_P_max = 80;
	Power_method[num].Self_Protect_Limit = 5100;
	Power_method[num].k_BAT = 0.85f;
	Power_method[num].Excess_P_max_J = 350;
	Power_method[num].Excess_P_max_P = 1300;
	Power_method[num].CurrentMax = 14000;
	Power_method[num].Follow_W = 6500;
	/****************100W********************/
	num++;
	Power_method[num].Actual_P_max = 100;
	Power_method[num].Self_Protect_Limit = 6300;
	Power_method[num].k_BAT = 0.92f;
	Power_method[num].Excess_P_max_J = 400;
	Power_method[num].Excess_P_max_P = 1200;
	Power_method[num].CurrentMax = 16000;
	Power_method[num].Follow_W = 8000;

}break;
default:
{
/****************Ĭ�ϲ���********************/ 
	Power_method[num].Actual_P_max = 60;
	Power_method[num].Self_Protect_Limit = 2500;
	Power_method[num].k_BAT = 0.7f;
	Power_method[num].Excess_P_max_J = 500;
	Power_method[num].Excess_P_max_P = 1000;
	Power_method[num].CurrentMax = 10000;
	Power_method[num].Follow_W = 4000;
}
}
}
/**********************************************************************************************************
*�� �� ��: Pid_ChassisWheelInit
*����˵��: ����XY���˶�PID������ʼ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Pid_ChassisWheelInit(void)
{
	short i=0;
	
	for(i = 0;i < 4;i ++)
	{
		switch(Robot_ID)
	{
/******************* 3�ų�**************************************************************************************/	
		case 3:                            // 3�ų�
		{
		//������
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//�ٶȻ�
		pidChassisWheelSpeed[i].P = 3.0f;
		pidChassisWheelSpeed[i].I = 0.0f;
		pidChassisWheelSpeed[i].D = 0.0f;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 10000.0f;	
			}break;
		
/************************************* 4�ų� ***************************************************************/			
		case 4:{                               //    �ĺų�
					//������
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//�ٶȻ�
		pidChassisWheelSpeed[i].P = 3.0f;
		pidChassisWheelSpeed[i].I = 0.0f;
		pidChassisWheelSpeed[i].D = 0.0f;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 10000.0f;	
			}break;

/************************************* 5�ų� ***************************************************************/			
		case 5:{                               //    ��ų�
					//������
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//�ٶȻ�
		pidChassisWheelSpeed[i].P = 5.0f;
		pidChassisWheelSpeed[i].I = 0.0f;
		pidChassisWheelSpeed[i].D = 0.0f;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 10000.0f;	
			}break;
	
			default:
			{
									//������
		Pid_Current[i].P = 0.16f;
		Pid_Current[i].I = 0.0f;
		Pid_Current[i].D = 0.0f;
		Pid_Current[i].IMax = 2500;//2500
		Pid_Current[i].SetPoint = 0.0f;
		Pid_Current[i].OutMax = 8000.0f;	//8000.0f
		
		//�ٶȻ�
		pidChassisWheelSpeed[i].P = 12.0f;
		pidChassisWheelSpeed[i].I = 0.0f;
		pidChassisWheelSpeed[i].D = 0.0f;
		pidChassisWheelSpeed[i].ErrorMax = 1000.0f;
		pidChassisWheelSpeed[i].IMax = 4000.0f;
		pidChassisWheelSpeed[i].SetPoint = 0.0f;	
		pidChassisWheelSpeed[i].OutMax = 10000.0f;	
					
			}
		}
	}
}

/**********************************************************************************************************
*�� �� ��: HeatControl
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
//��������
extern JudgeReceive_t JudgeReceive;

uint16_t HeatMax17, HeatCool17;
const short BulletHeat17 = 10;

short CurHeat17, LastHeat17, AvailableHeat17; //��ǰ������ ��һ������, ���м�������

uint16_t Shooted17Cnt;	//һ�������Ѵ���ӵ���
uint16_t AvailableBullet17;	//��һ�����������

char ShootAbleFlag;

void HeatControl(void)
{
	if(JudgeReceive.HeatUpdateFlag == 1)	//��������
	{
		Shooted17Cnt = 0;
		AvailableHeat17 = LIMIT_MAX_MIN(HeatMax17 - CurHeat17 + HeatCool17,HeatMax17,0);
//		AvailableHeat17 = HeatMax17 - CurHeat17;
		if(JudgeReceive.ShootCpltFlag == 1)	//��⵽������Ϊ�������º������ӵ�
		{
			AvailableHeat17 = LIMIT_MAX_MIN(AvailableHeat17 - BulletHeat17,HeatMax17,0);
			JudgeReceive.ShootCpltFlag = 0;	//�Ѵ����걾�η���
		}
		AvailableBullet17 = AvailableHeat17 / BulletHeat17;
		ShootAbleFlag = (AvailableBullet17 < 1)?0:1;		
	}	
	
	else if((JudgeReceive.ShootCpltFlag == 1) && (JudgeReceive.HeatUpdateFlag == 0))	//����û�и��£�����⵽����
	{
		JudgeReceive.ShootCpltFlag = 0;		//�Ѵ����걾�η���
		Shooted17Cnt++;		//������һ���ӵ�
		ShootAbleFlag = (Shooted17Cnt >= AvailableBullet17)?0:1;		
	}
}

/**********************************************************************************************************
*�� �� ��: HeatUpdate
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
const float HeatControlThreshold = 0.8f;   	//�����������Ƶ���ֵ

void HeatUpdate(void)
{
//	HeatMax17 = JudgeReceive.HeatMax17 + (short)(1250/JudgeReceive.maxHP) - BulletHeat17;		//ե������.jpg
	HeatMax17 = JudgeReceive.HeatMax17 - BulletHeat17;		//ե��������ֻ����һ�ŵ��������
	HeatCool17 = JudgeReceive.HeatCool17/10;          // ����ÿ�μ�����ȴֵ
	CurHeat17 = JudgeReceive.shooterHeat17;          //���յ��Ĳ���ϵͳ����
	
	if(CurHeat17 != LastHeat17)
	{
		JudgeReceive.HeatUpdateFlag = 1;
		JudgeReceive.ShootCpltFlag = 0;			//���������򽫷����־λ����(û�д�����Ĵ�)
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
	
	JudgeReceive.HeatUpdateFlag = 0;		//�Ѵ����걾����������
	LastHeat17 = CurHeat17;
	F105.IsShootAble = ShootAbleFlag;
}

/**********************************************************************************************************
*�� �� ��: BuildF105
*����˵��: ����Ҫ�����ϲ���F105�ṹ��
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void BuildF105(void)
{
	if(JudgeReceive.robot_id < 10)
		F105.RobotRed = 1;
	else
		F105.RobotRed = 0;			//0Ϊ��ɫ��1Ϊ��ɫ
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
*�� �� ��: Chassis_task
*����˵��: ��������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
uint32_t Chassis_high_water;
TickType_t TestNowTick=0;

extern uint8_t JudgeReveice_Flag;
extern TaskHandle_t JudgeReceiveTask_Handler; //������

void Chassis_task(void *pvParameters)
{
  portTickType xLastWakeTime;
	const portTickType xFrequency = 1;
	
  while (1) {
    TestNowTick=xLastWakeTime = xTaskGetTickCount();
		
		if(JudgeReveice_Flag)
		{
		 xTaskNotifyGive(JudgeReceiveTask_Handler);
		}
		
		//���ݳ�ŵ����
		if(JudgeReceive.remainEnergy<40)
		{
		Charge_Off;
		ChargeState = ChargeOff ;
		}
		else
		{
			Charge_On;
			ChargeState = ChargeOn;
		}	
	
		//��������
    Chassis_CurrentPid_Cal();
	
		ChassisCan1Send(WheelCurrentSend[0],WheelCurrentSend[1],WheelCurrentSend[2],WheelCurrentSend[3]); 
		
		//��������
		HeatUpdate();
		BuildF105();
		Can2Send0(&F105);
		
   //DataView
   //	VOFA_Send();
	
		IWDG_Feed();//ι��		
		vTaskDelayUntil(&xLastWakeTime,xFrequency); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}
