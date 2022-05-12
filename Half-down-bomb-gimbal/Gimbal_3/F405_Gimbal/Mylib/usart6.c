/**********************************************************************************************************
 * @�ļ�     usart6.c
 * @˵��     usart6��ʼ����PCͨ��
 * @�汾  	 V1.0
 * @����     ��־��
 * @����     2020.1
**********************************************************************************************************/
#include "main.h"

unsigned char PCbuffer[PC_RECVBUF_SIZE]={0,0,0};
extern unsigned char SendToPC_Buff[PC_SENDBUF_SIZE];

/**********************************************************************************************************
*�� �� ��: USART6_Configuration
*����˵��: usart6���ú���(PCͨ��)
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
void USART6_Configuration(void)
{
    USART_InitTypeDef usart6;
		GPIO_InitTypeDef  gpio;
    NVIC_InitTypeDef  nvic;
	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
		GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); 
	
    gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
		gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_Speed = GPIO_Speed_100MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOC,&gpio);

		usart6.USART_BaudRate = 115200;
		usart6.USART_WordLength = USART_WordLength_8b;
		usart6.USART_StopBits = USART_StopBits_1;
		usart6.USART_Parity = USART_Parity_No;
		usart6.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
    usart6.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		USART_Init(UART4,&usart6);
		
		USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
		USART_Cmd(UART4,ENABLE);
		
		USART_Cmd(UART4,ENABLE);
    USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);	
		USART_DMACmd(UART4,USART_DMAReq_Tx,ENABLE);
		
    nvic.NVIC_IRQChannel = DMA1_Stream2_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 1;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
		
		{
			DMA_InitTypeDef   dma;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
			DMA_DeInit(DMA1_Stream2);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)PCbuffer;
			dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
			dma.DMA_BufferSize = PC_RECVBUF_SIZE;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream2,&dma);
			DMA_ITConfig(DMA1_Stream2,DMA_IT_TC,ENABLE);
			DMA_Cmd(DMA1_Stream2,ENABLE);
		}
		
		{
			DMA_InitTypeDef   dma;
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,ENABLE);
			DMA_DeInit(DMA1_Stream4);
			dma.DMA_Channel= DMA_Channel_4;
			dma.DMA_PeripheralBaseAddr = (uint32_t)&(UART4->DR);
			dma.DMA_Memory0BaseAddr = (uint32_t)SendToPC_Buff;
			dma.DMA_DIR = DMA_DIR_MemoryToPeripheral;
			dma.DMA_BufferSize = PC_SENDBUF_SIZE;
			dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
			dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
			dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
			dma.DMA_Mode = DMA_Mode_Circular;
			dma.DMA_Priority = DMA_Priority_VeryHigh;
			dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
			dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
			dma.DMA_MemoryBurst = DMA_Mode_Normal;
			dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream4,&dma);
			DMA_Cmd(DMA1_Stream4,DISABLE);
		}	
}
/**********************************************************************************************************
*�� �� ��: DMA2_Stream2_IRQHandler
*����˵��: usart6 DMA�����ж�
*��    ��: ��
*�� �� ֵ: ��
**********************************************************************************************************/
unsigned char tempPC[PC_RECVBUF_SIZE];//����Ҫ��Ϊȫ�ֱ�������Ȼcrc��ͨ��
short Crcpass,crcNopass;
extern PC_Receive_t PC_Receive;
unsigned char ErrorBuff[PC_RECVBUF_SIZE*4];
short buffindex;

/**********************************************��ͨ����****************************************************************/
void DMA1_Stream2_IRQHandler(void)
{
  static unsigned char temptemp[2*PC_RECVBUF_SIZE];
	short PackPoint,n;
	if(DMA_GetITStatus(DMA1_Stream2, DMA_IT_TCIF2))
	{
		memcpy(temptemp+PC_RECVBUF_SIZE,PCbuffer,PC_RECVBUF_SIZE);
	  for(PackPoint = 0; PackPoint < PC_RECVBUF_SIZE; PackPoint++)//��ֹ��λ����һ������Ԫ�صĵ�һ����Ϊ
		{
			if(temptemp[PackPoint] == '!')
			{
				for(n=0;n<PC_RECVBUF_SIZE;n++)
				{
					tempPC[n] = temptemp[(n+PackPoint)];
				}
			  crcNopass++;
				if(Verify_CRC8_Check_Sum(tempPC,PC_RECVBUF_SIZE))
				{
				   PCReceive(tempPC);
				}
				else
				{
					buffindex++;
					buffindex = buffindex%4;
				}
				break;
			}
	  }
		DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2);
		DMA_ClearITPendingBit(DMA1_Stream2, DMA_IT_TCIF2);
		memcpy(temptemp,temptemp+PC_RECVBUF_SIZE,PC_RECVBUF_SIZE);
  }
}
