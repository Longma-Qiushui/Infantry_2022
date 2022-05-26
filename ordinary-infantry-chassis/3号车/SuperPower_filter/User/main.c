/**********************************************************************************************************
 * @文件     main.c
 * @说明     主文件
 * @版本  	 V1.0
 * @作者     陈志鹏
 * @日期     2021.4
**********************************************************************************************************/
#include "main.h"

char Robot_ID;
char Judge_Lost;
//unsigned char PowerState = 0;
extern ChassisSpeed_t chassis;
short Judgement_DisConnect;
extern uint16_t  MyMaxPower;
extern JudgeReceive_t JudgeReceive;
/**********************************************************************************************************
*函 数 名: main
*功能说明: 主函数
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
int main()
{
  BSP_Init();
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
	while(SysTick_Config(72000));	
	ADC_Configuration();//ADC初始化要放在串口之前，不然串口不能接收数据
	DAC1_Init();
	CAN1_Configuration();
	CAN2_Configuration();
	VOFA_USART_Configuration();
	UART4_Configuration();
	TIM2_Configuration();
	TIM4_Configuration();
	IWDG_Config(IWDG_Prescaler_64 ,625);
	i2c_init();
	ChargeIO_Configuration();
	Charge_Off;
	delay_ms(300);//等主控板初始化完成，防止主控板初始化过程中向底盘发送错误数据
}
/**********************************************************************************************************
*函 数 名: System_Init
*功能说明: 系统参数初始化
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Robot_Init(void)
{
	Pid_ChargeCal_Init();
	Pid_ChassisWheelInit();
	Chassis_Power_Control_Init();		//这个和pid的顺序不能调换
}
/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务(任务优先级太低，在这里放串口通信会导致数据发不出去)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
void Judge_Rst()
{
 MyMaxPower=60;
 JudgeReceive.remainEnergy=40;
}
/**********************************************************************************************************
*函 数 名: Offline_Check_task
*功能说明: 掉线检测任务(任务优先级太低，在这里放串口通信会导致数据发不出去)
*形    参: 无
*返 回 值: 无
**********************************************************************************************************/
uint32_t Offline_Check_high_water;
char Chassis_DisConnect;
extern short F405_DisConnect;//主控板掉线检测

extern ext_student_interactive_header_data_t custom_grapic_draw;
extern uint8_t seq;
void Offline_Check_task(void *pvParameters)
{
   while (1) {
    

		/*主控板掉线检测*/
		if(F405_DisConnect>5)
		{
			F405_Rst();
		}else
		{
		
		}
		F405_DisConnect++;
		
		/*裁判系统掉线检测*/
		if(Judgement_DisConnect>100)
		{
			Judge_Rst();
			Judge_Lost=1;
		}else
		{
		  Judge_Lost=0;
		}
		Judgement_DisConnect++;
		
			/*底盘电机掉线检测*/
		if(Chassis_DisConnect>100)
		{
			
	
		}else
		{
		 
		}
	  Chassis_DisConnect++;
		
		
		
		IWDG_Feed();//喂狗

		vTaskDelay(5); 
		 
#if INCLUDE_uxTaskGetStackHighWaterMark
        Offline_Check_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


