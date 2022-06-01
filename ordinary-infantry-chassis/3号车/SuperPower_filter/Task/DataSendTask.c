/**********************************************************************************************************
 * @�ļ�     DataSendTask.c
 * @˵��     ���ݷ���
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2019.10
**********************************************************************************************************/
#include "main.h"
/*----------------------------------�ڲ�����---------------------------*/

/*----------------------------------�ṹ��-----------------------------*/

/*----------------------------------�ⲿ����---------------------------*/
extern INA260 INA260_1;//����
extern INA260 INA260_2;//���
extern float AD_actual_value;//����ʵ�ʵ�ѹ
extern F405_typedef F405;
extern JudgeReceive_t JudgeReceive;
extern char Robot_ID;
extern char Judge_Lost;
/**********************************************************************************************************
*�� �� ��: ChassisCan1Send
*����˵��: ���̵������ֵ����
*��    ��: �ĸ��������ֵ
*�� �� ֵ: ��
**********************************************************************************************************/
//TickType_t nowtick,lasttick,interval;
short last[4],lastsum,sum;
short post_filter[4];
extern short CAP_CrossoverFlag;
char output_filter;
void ChassisCan1Send(short a,short b,short c,short d)
{
	static short chassis_send[4];
    CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x08;    
    tx_message.StdId = 0x200;
  
	if(output_filter)
	{
		chassis_send[0] = LowPass_SetWheel(a,chassis_send[0]);
	  chassis_send[1] = LowPass_SetWheel(b,chassis_send[1]);
	  chassis_send[2] = LowPass_SetWheel(c,chassis_send[2]);
	  chassis_send[3] = LowPass_SetWheel(d,chassis_send[3]);
	}
	else 
	{
    chassis_send[0]= a;
    chassis_send[1]= b;
		chassis_send[2]= c;
		chassis_send[3]= d;	
	}
	  sum=ABS(chassis_send[0])+ABS(chassis_send[1])+ABS(chassis_send[2])+ABS(chassis_send[3]);
				if(sum-lastsum>2000)		//�����仯
			{
				CAP_CrossoverFlag = 1;																										//����ֹͣ���
			}
			lastsum=sum;

    tx_message.Data[0] = (unsigned char)((chassis_send[0]>>8)&0xff);
    tx_message.Data[1] = (unsigned char)(chassis_send[0]&0xff);  
    tx_message.Data[2] = (unsigned char)((chassis_send[1]>>8)&0xff);
    tx_message.Data[3] = (unsigned char)(chassis_send[1]&0xff);
    tx_message.Data[4] = (unsigned char)((chassis_send[2]>>8)&0xff);
    tx_message.Data[5] = (unsigned char)(chassis_send[2]&0xff);
    tx_message.Data[6] = (unsigned char)((chassis_send[3]>>8)&0xff);
    tx_message.Data[7] = (unsigned char)(chassis_send[3]&0xff);
    CAN_Transmit(CAN1,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: Can2Send0
*����˵��: can2���ͺ���
*��    ��: ChassisSpeedw, Remain_power, IsShootAble, RobotRed, BulletSpeedLevel
*�� �� ֵ: ��
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
	  memcpy(&tx_message.Data[7],&Robot_ID,1);

	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: Can2Send1
*����˵��: can2���ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Can2Send1(void)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x06;    
    tx_message.StdId = 0x094;
	  
	  memcpy(tx_message.Data,&JudgeReceive.HeatMax17,2);	
	  memcpy(&tx_message.Data[2],&JudgeReceive.HeatCool17,2);	
	  memcpy(&tx_message.Data[4],&JudgeReceive.shooterHeat17,2);	
	   
	  CAN_Transmit(CAN2,&tx_message);
}

/**********************************************************************************************************
*�� �� ��: Can2Send1
*����˵��: can2���ͺ���
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/

void Can2Send2(void)
{
	  CanTxMsg tx_message;
    tx_message.IDE = CAN_ID_STD;    
    tx_message.RTR = CAN_RTR_DATA; 
    tx_message.DLC = 0x06;    
    tx_message.StdId = 0x095;
	  
	  memcpy(&tx_message.Data[0],&JudgeReceive.bulletSpeed,4);	
    memcpy(&tx_message.Data[4],&Judge_Lost,1);
	  memcpy(&tx_message.Data[5],&JudgeReceive.bulletFreq,1);
	
	  CAN_Transmit(CAN2,&tx_message);
}
/**********************************************************************************************************
*�� �� ��: USART2SEND
*����˵��: DataScope���ͺ���
*��    ��: ��
*�� �� ֵ: ��
//**********************************************************************************************************/
//extern float test_W_Chassis_t;
//void USART2SEND(void)
//{
//		DataScope_Get_Channel_Data(test_W_Chassis_t, 1 );  
//		DataScope_Get_Channel_Data(JudgeReceive.realChassispower, 2 );  
//		DataScope_Get_Channel_Data(JudgeReceive.remainEnergy, 3 );  
//		DataScope_Get_Channel_Data(JudgeReceive.MaxPower, 4 );  
//		DataScope_Get_Channel_Data(0.0, 5 );  
//		DataScope_Get_Channel_Data(0.0, 6 );  
//		DataScope_Get_Channel_Data(0.0, 7 );  
//		DataScope_Get_Channel_Data(0.0, 8 );  
//		DataScope_Get_Channel_Data(0.0, 9 );  
//		DataScope_Get_Channel_Data(0.0, 10 );  

//		u8 Send_Count;
//		Send_Count = DataScope_Data_Generate(10);
//		for( int i = 0 ; i < Send_Count; i++)
//		{
//			while((USART2->SR&0X40)==0);  	
//			USART2->DR = DataScope_OutPut_Buffer[i];    
//		}
//}
