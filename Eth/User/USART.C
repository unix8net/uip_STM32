#include "stm32f10x_usart.h"
unsigned char TxBuf[17] = "0123456789ABCDEF";

void USART1_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                         RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);
//====================串口引脚PA10，PA9==============================
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
	GPIO_Init(GPIOA, &GPIO_InitStructure);		   

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   
	GPIO_Init(GPIOA, &GPIO_InitStructure);

//===================USART1配置======================================
	
    USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	USART_Cmd(USART1, ENABLE); 		  //使能

}

/* ========USART打印信息============ */
void USART_Print(USART_TypeDef* USARTx, unsigned char* TxBuf)
{
	int i;
	
	for( i = 0; TxBuf[i] != '\0'; i++) {
		USART_SendData(USART1 , TxBuf[i]);
		while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
	}
}

void PrintShuzi(unsigned char temp)
{
	 USART_SendData(USART1 , TxBuf[temp/16]);
	 while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
	 USART_SendData(USART1 , TxBuf[temp%16]);
	 while(USART_GetFlagStatus(USART1, USART_FLAG_TC)==RESET);
}

void USART_PrintString(uint32_t DATA )		  //32bit，共8个16进制数
{
	uint32_t temp;
	unsigned char shuzi;

	temp = DATA >> 24;
	shuzi = temp & 0x0FF;
	PrintShuzi(shuzi);

	temp = DATA >> 16;
	shuzi = temp & 0x0FF;
	PrintShuzi(shuzi);

	temp = DATA >> 8;
	shuzi = temp & 0x0FF;
	PrintShuzi(shuzi);

	temp = DATA >> 0;
	shuzi = temp & 0x0FF;
	PrintShuzi(shuzi);
}

