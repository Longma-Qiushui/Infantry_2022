#include "main.h"
extern unsigned char JudgeSend[SEND_MAX_SIZE];
extern JudgeReceive_t JudgeReceive;
extern F405_typedef F405;
extern ChassisSpeed_t chassis;
int Char_Change_Array[5];					//0��ʾû�仯����0��ʾ�б仯
ext_student_interactive_char_header_data_t custom_char_draw;  //�Զ����ַ�����

/**********************************************************************************************************
*�� �� ��: JudgementCustomizeChar
*����˵��: ͼ�����ݰ��ֶδ��
*��    ��: ��������
*�� �� ֵ: ��
**********************************************************************************************************/
void JudgementCustomizeChar(int Op_type)
{
		custom_char_draw.data_cmd_id=0x0110;//�����ַ�

		custom_char_draw.sender_ID=JudgeReceive.robot_id;//������ID�������˶�ӦID
		if(JudgeReceive.robot_id == 103)
				custom_char_draw.receiver_ID = 0x0167;
		if(JudgeReceive.robot_id == 104)
				custom_char_draw.receiver_ID = 0x0168;
		if(JudgeReceive.robot_id == 105)
				custom_char_draw.receiver_ID = 0x0169;
		if(JudgeReceive.robot_id == 3)
				custom_char_draw.receiver_ID = 0x0103;	
		if(JudgeReceive.robot_id == 4)
				custom_char_draw.receiver_ID = 0x0104;
		if(JudgeReceive.robot_id == 5)
				custom_char_draw.receiver_ID = 0x0105;

/*********************************�Զ����ַ�����***********************************/
		referee_data_load_String(Op_type);
}

/**********************************************************************************************************
*�� �� ��: referee_data_load_String
*����˵��: ͼ�����ݰ�װ���ַ���
*��    ��: ��������
*�� �� ֵ: ��
**********************************************************************************************************/
float c_pos_x[10] = {0.57,0.34,0.4,0.52,0.34,0.42,0.62,0.5,0.40,0.53};
float c_pos_y[10] = {0.65,0.15,0.8,0.1,0.1,0.15,0.1,0.8,0.1,0.15};
extern enum POWERSTATE_Typedef PowerState;
void referee_data_load_String(int Op_type)
{
	static int tick=0;
	static char Mag_State[2][6] = {"CLOSE","OPEN"};
	static char Gimbal_State[5][7] = {"OFF","NORMAL","HACKER","BUFF","DROP"};
	static char Chassis_State[5][9] = {"OFF","FOLLOW","SelfPro","SOLO","NOFOLLOW"};
	static char Power_State[2][4] = {"Bat","Cap"};
	static char Friction[2][9]={"Firc off","Fric on"};
	/*��ʼ����������������ͼ��*/
	if(Op_type == Op_Init)
	{
		switch(tick%10)
		{
			/*��̬�ַ�*/
			case 0:
			/*******************************pitch �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 0;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("PITCH:");
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[0]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[0]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"PITCH:");
			break;
			case 1:
			/*******************************Magazine �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 1;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("MAG:");
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[1]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[1]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"MAG:");
			break;
			case 2:
			/*******************************Chassis �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 2;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("CHASSIS:");
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[2]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[2]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"CHASSIS:");
			break;
			case 3:
			/*******************************Gimbal �ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 3;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("GIMBAL:");
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[3]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[3]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"GIMBAL:");
			break;
			case 4:
/*******************************����״̬*********************************/
PS:		custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 4;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=5;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(Power_State[PowerState]);
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[4]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[4]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,Power_State[PowerState]);
			goto PSBack;
			break;
			/*�ɱ�״̬�ַ�*/
			case 5:
			/*******************************���ָǿ���״̬*********************************/
MAG:  custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 5;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=5;
			custom_char_draw.char_custom.grapic_data_struct.color=Green;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(Mag_State[F405.Mag_Flag]);
			custom_char_draw.char_custom.grapic_data_struct.width=2;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[5]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[5]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,Mag_State[F405.Mag_Flag]);
			goto MagBack;
			break;
			case 6:
			/*******************************��̨״̬*********************************/
GIMBAL:custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 6;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=5;
			custom_char_draw.char_custom.grapic_data_struct.color=Green;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(Gimbal_State[F405.Gimbal_Flag]);
			custom_char_draw.char_custom.grapic_data_struct.width=2;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[6]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[6]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,Gimbal_State[F405.Gimbal_Flag]);
			goto GimbalBack;
			break;
			case 7:
			/*******************************����״̬*********************************/
CHASSIS:custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 7;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=5;
			custom_char_draw.char_custom.grapic_data_struct.color=F405.Follow_state?White:Green;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(Chassis_State[F405.Chassis_Flag]);
			custom_char_draw.char_custom.grapic_data_struct.width=F405.Follow_state?5:2;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[7]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[7]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,Chassis_State[F405.Chassis_Flag]);
			goto ChassisBack;
			break;
			case 8:
		/*******************************Ħ����*********************************/
Fric: 
			
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 8;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=5;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen(Friction[F405.Fric_Flag]);
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[8]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[8]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,Friction[F405.Fric_Flag]);
      goto FricBack;
			break;
			case 9:
			/*******************************���ݵ�ѹ�ַ�*********************************/
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
			custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 9;
			custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Op_type;
			custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;
			custom_char_draw.char_custom.grapic_data_struct.layer=9;
			custom_char_draw.char_custom.grapic_data_struct.color=Orange;
			custom_char_draw.char_custom.grapic_data_struct.start_angle=25;
			custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("CAP:     V");
			custom_char_draw.char_custom.grapic_data_struct.width=3;
			custom_char_draw.char_custom.grapic_data_struct.start_x=c_pos_x[9]*SCREEN_LENGTH;
			custom_char_draw.char_custom.grapic_data_struct.start_y=c_pos_y[9]*SCREEN_WIDTH;
			memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
			strcpy(custom_char_draw.char_custom.data,"CAP:     V");
			break;			
			default:
				break;
		}
		tick++;
	}else if(Op_type == Op_Change)		//����Ǳ�־Ϊ�޸�
	{
		/*Ѱ�����ĸ������˱仯*/
		if(Char_Change_Array[0] == Op_Change)
		{
			Char_Change_Array[0] = Op_None;
			goto MAG;
			MagBack:return;
		}else if(Char_Change_Array[1] == Op_Change)
		{
			Char_Change_Array[1] = Op_None;
			goto GIMBAL;
			GimbalBack:return;
		}else if(Char_Change_Array[2] == Op_Change)
		{
			Char_Change_Array[2] = Op_None;
			goto CHASSIS;
			ChassisBack: return;
		}else if(Char_Change_Array[3] == Op_Change)
		{
			Char_Change_Array[3] = Op_None;
			goto Fric;
			FricBack: return;
		}else if(Char_Change_Array[4] == Op_Change)
		{
			Char_Change_Array[4] = Op_None;
			goto PS;	
			PSBack: return;
		}
	}
}

/**********************************************************************************************************
*�� �� ��: Char_Change_Check
*����˵��: �����û���ַ��ı䶯�����߽����ַ�ͼ��ĳ�ʼ��
*��    ��: ��
*�� �� ֵ: 
**********************************************************************************************************/
int Char_Change_Check(void)
{
	int i;
	static char last_Mag,last_Gimbal,last_Chassis,last_Laser,last_Follow,last_Fric,last_PowerState;	//��¼�ϴ�״̬
	static int delete_flag;
	char Mag_flag,Gimbal_flag,Chassis_flag,Laser_flag,Follow_state,Fric_flag;


	if(F405.Graphic_Init_Flag == 0)		
	{
		return Op_Init;	//����Init,��һֱ����Add���������ͼ��
	}
		
	/*��ȡ��̨���͵ĸ���״̬*/
	Mag_flag = F405.Mag_Flag;
	Gimbal_flag = F405.Gimbal_Flag;
	Chassis_flag = F405.Chassis_Flag;
	Fric_flag = F405.Fric_Flag;
	Follow_state = F405.Follow_state;
	
	/*�б仯����־����λ*/
	if(last_Mag != Mag_flag) Char_Change_Array[0] = Op_Change;
	if(Gimbal_flag != last_Gimbal) Char_Change_Array[1] = Op_Change;
	if(Chassis_flag != last_Chassis || Follow_state != last_Follow) Char_Change_Array[2] = Op_Change;
  if(Fric_flag != last_Fric ) Char_Change_Array[3]=Op_Change;	
	if(PowerState != last_PowerState) Char_Change_Array[4]=Op_Change;
	
	/*������α�־���ϴαȽ�*/
	last_Mag = Mag_flag;
	last_Gimbal = Gimbal_flag;
	last_Chassis = Chassis_flag;
	last_Fric = Fric_flag;
	last_Follow = Follow_state;
	last_PowerState=PowerState;
	
	/*������û�з����仯������б仯�򷵻��޸�ͼ��*/
	for(i = 0;i<5;i++)
	{
		if(Char_Change_Array[i] == Op_Change)
			return Op_Change;
	}
	return Op_None;	//���򷵻ؿղ�������ʱ���ᷢ�Ͷ���
}
/**********************************************************************************************************
*�� �� ��: Load_Char_Init
*����˵��: װ���ַ���ʼ����ʾ
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void Load_Char_Init(char Init_Flag)
{
	custom_char_draw.data_cmd_id=0x0110;//�����ַ�

	custom_char_draw.sender_ID=JudgeReceive.robot_id;//������ID�������˶�ӦID
	if(JudgeReceive.robot_id == 103)
			custom_char_draw.receiver_ID = 0x0167;
	if(JudgeReceive.robot_id == 104)
			custom_char_draw.receiver_ID = 0x0168;
	if(JudgeReceive.robot_id == 105)
			custom_char_draw.receiver_ID = 0x0169;
	if(JudgeReceive.robot_id == 3)
			custom_char_draw.receiver_ID = 0x0103;	
	if(JudgeReceive.robot_id == 4)
			custom_char_draw.receiver_ID = 0x0104;
	if(JudgeReceive.robot_id == 5)
			custom_char_draw.receiver_ID = 0x0105;
	
	/*******************************Init:B �ַ�*********************************/
	custom_char_draw.char_custom.grapic_data_struct.graphic_name[0] = 0;  //��Ϊ�ͻ���������ͼ����
	custom_char_draw.char_custom.grapic_data_struct.graphic_name[1] = 41;
	custom_char_draw.char_custom.grapic_data_struct.graphic_name[2] = 9;
	custom_char_draw.char_custom.grapic_data_struct.operate_tpye=Init_Flag?Op_Delete:Op_Add;
	custom_char_draw.char_custom.grapic_data_struct.graphic_tpye=7;         //���ͣ��ַ�
	custom_char_draw.char_custom.grapic_data_struct.layer=9;                //ͼ������
	custom_char_draw.char_custom.grapic_data_struct.color=Green;
	custom_char_draw.char_custom.grapic_data_struct.start_angle=25;            //�����С
	custom_char_draw.char_custom.grapic_data_struct.end_angle=strlen("Initing Press B To Hide"); //�ַ�����
	custom_char_draw.char_custom.grapic_data_struct.width=4;                   //�������
	custom_char_draw.char_custom.grapic_data_struct.start_x=0.34*SCREEN_LENGTH; //���x����
	custom_char_draw.char_custom.grapic_data_struct.start_y=0.56*SCREEN_WIDTH;  //���y����
	custom_char_draw.char_custom.grapic_data_struct.end_x=0;                   
	custom_char_draw.char_custom.grapic_data_struct.end_y=0;
	custom_char_draw.char_custom.grapic_data_struct.radius = 0;
	memset(custom_char_draw.char_custom.data,'\0',sizeof(custom_char_draw.char_custom.data));
	strcpy(custom_char_draw.char_custom.data,"Initing Press B To Hide");
}
/**********************************************************************************************************
*�� �� ��: CharSendtask
*����˵��: �ַ���������
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void CharSendtask(void *pvParameters)
{
	 static int tick;
   while (1) {
    
		tick++;
//		if(0==tick%10)	
//		{
////			JudgementCustomizeChar();
////			referee_data_pack_handle(0xA5,0x0301,(uint8_t *)&custom_char_draw,sizeof(custom_char_draw));
//		}
	   VOFA_Send();
		 IWDG_Feed();//ι��
			vTaskDelay(1); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
		
#endif
    }
}
