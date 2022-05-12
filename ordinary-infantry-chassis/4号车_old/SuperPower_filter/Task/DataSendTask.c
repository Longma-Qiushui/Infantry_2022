/**********************************************************************************************************
 * @文件     DataSendTask.c
 * @说明     数据发送
 * @版本  	 V1.0
 * @作者     黄志雄
 * @日期     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------内部变量---------------------------*/

/*----------------------------------结构体-----------------------------*/

/*----------------------------------外部变量---------------------------*/
extern INA260 INA260_1;//输入
extern INA260 INA260_2;//输出
extern float AD_actual_value;//电容实际电压
extern F405_typedef F405;
extern JudgeReceive_t JudgeReceive;
/**********************************************************************************************************
*函 数 名: ChassisCan1Send
*功能说明: 底盘电机电流值发送
*形    参: 四个电机电流值
*返 回 值: 无
**********************************************************************************************************/
//TickType_t nowtick,lasttick,interval;
short last[4],lastsum,sum;
extern short CAP_CrossoverFlag;
void ChassisCan1Send(short a,short b,short c,short d)
{
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x200;
    a=LowPass_SetWheel(a);
	  b=LowPass_SetWheel(b);
	  c=LowPass_SetWheel(c);
	  d=LowPass_SetWheel(d);
	  sum=ABS(a)+ABS(b)+ABS(c)+ABS(d);
				if(sum-lastsum>2000)		//发生变化
			{
				CAP_CrossoverFlag = 1;																										//电容停止充电
			}
			lastsum=sum;
		
    tx_message.Data[0] = (unsigned char)((a>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(a&0xff);  
    tx_message.Data[2] = (unsigned char)((b>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(b&0xff);
    tx_message.Data[4] = (unsigned char)((c>>8)&0xff);
    tx_message.Data[5] = (unsigned char)(c&0xff);
    tx_message.Data[6] = (unsigned char)((d>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(d&0xff);
    CAN_Transmit(CAN1,&tx_message);
}
/**********************************************************************************************************
*函 数 名: Can2Send0
*功能说明: can2发送函数
*形    参: ChassisSpeedw, Remain_power, IsShootAble, RobotRed, BulletSpeedLevel
*返 回 值: 无
**********************************************************************************************************/
void Can2Send0(F105_Typedef *F105_Send)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x100;
	  
	  memcpy(&tx_message.Data[0],&F105_Send->ChassisSpeedw,2);
		memcpy(&tx_message.Data[2],&F105_Send->Remain_power,2);
		memcpy(&tx_message.Data[4],&F105_Send->IsShootAble,1);
		memcpy(&tx_message.Data[5],&F105_Send->RobotRed,1);
		memcpy(&tx_message.Data[6],&F105_Send->BulletSpeedLevel,1);	

	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*函 数 名: Can2Send1
*功能说明: can2发送函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Can2Send1(short *k,short *p,short *m, short *n)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x100;
	  
	  memcpy(tx_message.Data,k,2);	
	  memcpy(&tx_message.Data[2],p,2);	
	  memcpy(&tx_message.Data[4],m,2);	
	  memcpy(&tx_message.Data[6],n,2);	
	   
	  CAN_Transmit(CAN2,&tx_message);
}
void Can2Send2(void)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x095;
	  
	  memcpy(&tx_message.Data[0],&JudgeReceive.bulletSpeed,4);	

	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*函 数 名: USART2SEND
*功能说明: DataScope发送函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
extern float test_W_Chassis_t;
void USART2SEND(void)
{
		DataScope_Get_Channel_Data(test_W_Chassis_t, 1 );  
		DataScope_Get_Channel_Data(JudgeReceive.realChassispower, 2 );  
		DataScope_Get_Channel_Data(JudgeReceive.remainEnergy, 3 );  
		DataScope_Get_Channel_Data(JudgeReceive.MaxPower, 4 );  
		DataScope_Get_Channel_Data(0.0, 5 );  
		DataScope_Get_Channel_Data(0.0, 6 );  
		DataScope_Get_Channel_Data(0.0, 7 );  
		DataScope_Get_Channel_Data(0.0, 8 );  
		DataScope_Get_Channel_Data(0.0, 9 );  
		DataScope_Get_Channel_Data(0.0, 10 );  

		u8 Send_Count;
		Send_Count = DataScope_Data_Generate(10);
		for( int i = 0 ; i < Send_Count; i++)
		{
			while((USART2->SR&0X40)==0);  	
			USART2->DR = DataScope_OutPut_Buffer[i];    
		}
}
