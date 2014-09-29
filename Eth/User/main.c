/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
/**********************以下是UIP网络模块定义***********************************/
#include "uip.h"
#include "uip_arp.h"
#include "tapdev.h"
#include "timer.h"

#include "httpd.h"

#define BUF ((struct uip_eth_hdr *)&uip_buf[0])

#ifndef NULL
#define NULL (void *)0
#endif /* NULL */
/**********************UIP网络模块定义END***********************************/
extern void USART1_Config(void);
extern void USART_Print(USART_TypeDef* USARTx, unsigned char* TxBuf);
extern void USART_SendData(USART_TypeDef* USARTx, uint16_t Data);
extern void USART_PrintString(uint32_t DATA );

extern void etherdev_init(void);
extern unsigned int etherdev_read(void);
extern void etherdev_send(void);
extern void SPI2_Init(void);

void  Delay (u32 nCount)
{
    for(; nCount != 0; nCount--);
}


void GPIO_Config()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE |RCC_APB2Periph_AFIO, ENABLE);

//====================LED引脚PC2，PC3==============================
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

}

/*******************************************
 * 初始化NVIC中断控制器配置
 * 注意:STM32虽然多达60多个可用中断，但实际上每个中断只用了4bit
 * 就是这4bit对中断进行了分等级（分组），分为抢占优先级和子优先级。
 * 芯嵌stm32  @2013-6-2注释
*********************************************/
void NVIC_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	    	//配置2位高优先级（抢占优先级），2位子优先级（响应优先级）
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  			//外部中断线0,1分别对应PC0
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3; 	//io引脚中断抢占优先等级为第二级。
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2; 			//io引脚中断子优先等级为第二级。
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

}


void Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2 , ENABLE);
    TIM_DeInit(TIM2);

    TIM_TimeBaseStructure.TIM_Period=1000;		 					//自动重装载寄存器周期的值(计数值)
    //含义：定时器中断要跳几下才能进中断，这里2000表示要跳2000下进中断
    //累计 TIM_Period个频率后产生一个更新或者中断
    TIM_TimeBaseStructure.TIM_Prescaler= (720 - 1);				    //时钟预分频数   例如：时钟频率=72MHZ/(时钟预分频+1)
    //时钟频率：一秒钟可以跳几下。这里是72MHZ/720=100K，即1s会跳100K次
    //这个时钟Timer，一秒钟会跳100K次，而它每跳2000下就进中断，那么每次定时时间20ms
    TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 			//采样分频
    TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 		//向上计数模式
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);							//清除溢出中断标志
    //TIM_PrescalerConfig(TIM2,0x8C9F,TIM_PSCReloadMode_Immediate);	//时钟分频系数36000，所以定时器时钟为2K
    //TIM_ARRPreloadConfig(TIM2, DISABLE);							//禁止ARR预装载缓冲器
    TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_Trigger,ENABLE);
    TIM_Cmd(TIM2, ENABLE);											//开启时钟

}

int main(void)
{
    int i;
    uip_ipaddr_t ipaddr;
    struct timer periodic_timer, arp_timer;

    SystemInit();
    GPIO_Config();		//LED点灯引脚配置
    USART1_Config();	//用于打印调试信息
    NVIC_Config();
    SPI2_Init();
    Timer_Config();	   //用于uIP的时钟，每10ms跳一次

    /*设置 TCP超时处理时间和 ARP老化时间*/
    timer_set(&periodic_timer, CLOCK_SECOND / 2);
    timer_set(&arp_timer, CLOCK_SECOND * 10);
    etherdev_init();

    uip_init();
    uip_arp_init();

    uip_ipaddr(ipaddr, 192,168,0,169);
    uip_sethostaddr(ipaddr);
    uip_ipaddr(ipaddr, 192,168,0,1);
    uip_setdraddr(ipaddr);
    uip_ipaddr(ipaddr, 255,255,255,0);
    uip_setnetmask(ipaddr);
	//	httpd_init();
		
    while(1) {
        uip_len = etherdev_read(); 				  /*轮询从网卡读数据*/
        if(uip_len == 0) {
            continue;
        }
        if(uip_len>0) {   						  		/*如果存在数据则按协议处理*/
            if(BUF->type == htons(UIP_ETHTYPE_IP)) {	   /*收到的是IP数据，调用 uip_input()处理*/
                uip_arp_ipin();
                uip_input();
                if(uip_len>0) {					      /*处理完后，如果uip_buf中有数据，则调用 etherdev_send 发送出去*/
                    uip_arp_out();
                    etherdev_send();
                }
            } else if (BUF->type==htons(UIP_ETHTYPE_ARP)) { /*收到的是 ARP数据，调用 uip_arp_arpin()处理*/
                uip_arp_arpin();
							//如果是其他主机的ARP请求，我们还需要回应别人
                if(uip_len>0) {
                    etherdev_send();
                }
            }
        } else if(timer_expired(&periodic_timer)) {	//0.5秒定时器超时
            timer_reset(&periodic_timer);			//复位0.5秒定时器
            for(i=0; i<UIP_CONNS; i++) {
                uip_periodic(i);
                if(uip_len>0) {
									//在发送IIP包之前必须调用uip_arp_out
                    uip_arp_out();
                    etherdev_send();
                }
            }
#if UIP_UDP
            for(i=0; i<UIP_UDP_CONNS; i++) {
                uip_udp_periodic(i);
                if(uip_len > 0) {
                    uip_arp_out();
                    etherdev_send();
                }
            }
#endif
            if(timer_expired(&arp_timer)) {
                timer_reset(&arp_timer);
                uip_arp_timer();
            }
        }
    }

}



