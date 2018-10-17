
#include <rtthread.h>
#include "rtconfig.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "usart.h"
#include "rthw.h"
#include "rtdef.h"

/* user app entry */
rt_thread_t tid_uart1,tid_led_breath,tid_ADC1;
rt_timer_t timer1,timer2;
u8 stop_flag;
//////////////////////////////////////////////////


//====================uart1===========
#define CLOCK 72/8

//时钟配置，后续再详细捋时钟这块的东西，现在姑且按照这样设置
void RCC_Configuration(void)   
{   
    ErrorStatus HSEStartUpStatus;   
    //将RCC寄存器设置为默认值
    RCC_DeInit(); 
    //打开外部高速时钟
    RCC_HSEConfig(RCC_HSE_ON);
    //等待外部高速时钟晶振起振
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS){
        //设置PLL时钟时钟源及倍频系数   
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_16);
        //设置AHB时钟
        RCC_HCLKConfig(RCC_SYSCLK_Div1); 
        //设置APB1低速时钟
        RCC_PCLK1Config(RCC_HCLK_Div2);
        //设置APB2高速时钟
        RCC_PCLK2Config(RCC_HCLK_Div1); 
        //使能PLL
        RCC_PLLCmd(ENABLE);
        //等待PLL工作
        while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)    
        {   
        } 
        //设置系统时钟为PLL时钟
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        //等待系统时钟切换为PLL时钟
        while(RCC_GetSYSCLKSource() != 0x08)    
        {   
        }   
    }
    //打开需要使用的外设的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |   
    RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE, ENABLE);  
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);   
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);   
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   
}

//延时微妙
void delay_us(unsigned int us)
{
    u8 n;           
    while(us--)for(n=0;n<CLOCK;n++);     
}
///////////////////////////////////



//==================  呼吸灯  =======================
//高精度延时函数
void rt_hw_us_delay(rt_uint32_t us)
{	//其中入口参数us指示出需要延时的微秒数目，这个函数只能支持低于1 OS tick的延时，
	//否则SysTick会出现溢出而不能够获得指定的延时时间。周期也就是10ms。
	//(RT_TICK_PER_SECOND =100,也就是10ms内的延时-形参us小于10000us)
	rt_uint32_t delta;
	/* 获得延时经过的tick数 */
	us = us * (SysTick->LOAD/(1000000/RT_TICK_PER_SECOND));
	/* 获得当前时间 */
	delta = SysTick->VAL;
	/* 循环获得当前时间，直到达到指定的时间后退出循环 */
	while (delta - SysTick->VAL< us);
}

void Delay(__IO u32 nCount)
{
	//for(;nCount != 0; nCount--);
	rt_hw_us_delay( nCount );
}


#define LED1_GPIO_RCC           RCC_APB2Periph_GPIOC
#define LED1_GPIO_PORT          GPIOC
#define LED1_GPIO_PIN      		GPIO_Pin_13
#define LED1_ONOFF(x)     		GPIO_WriteBit(GPIOC,GPIO_Pin_13,x);

void led1_gpio_init(void)
{
    GPIO_InitTypeDef gpioInit;
	
	RCC_APB2PeriphClockCmd(LED1_GPIO_RCC, ENABLE);

    gpioInit.GPIO_Mode  = GPIO_Mode_Out_PP;
    gpioInit.GPIO_Pin   = LED1_GPIO_PIN;
    gpioInit.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED1_GPIO_PORT, &gpioInit);
}

void led_loop( u32 on_time, u32 off_time )
{
	LED1_ONOFF(Bit_RESET);		//D1(PC13)亮
	Delay(on_time);
//	rt_thread_delay(on_time);
	LED1_ONOFF(Bit_SET);		//D1(PC13)灭
	Delay(off_time);
//	rt_thread_delay(off_time);

}
#define MAX_TIME 0x2000
#define MIN_TMIE 0x100
#define SPEED 0x15*3

void led_breath(void)//呼吸灯
{
	u32 time;
	
	time = MAX_TIME;
	for( time = MAX_TIME; time>SPEED; time -= SPEED )//渐亮
	{
		led_loop (MAX_TIME-time,time);
	}	
	for( time = MAX_TIME; time>SPEED; time -= SPEED )//渐灭
	{
		led_loop (time,MAX_TIME-time);
	}

}
///////////////////////////

#define STOP  1
#define START 0
void uart1(void)
{
	extern u8 uart_data[256];
	extern u8 uart_len;
	extern u8 uart_non_empty;

	if (0 != uart_non_empty )
	{
		if ( START == stop_flag )	//关闭其他任务
		{
			if (tid_led_breath != RT_NULL)
				rt_thread_suspend( tid_led_breath );
			if (timer1 != RT_NULL)
				rt_timer_stop( timer1);	
			if (timer2 != RT_NULL)
				rt_timer_stop( timer2);	
			
			stop_flag = STOP;
		}	
		if (uart_non_empty == 1)
		{
			printf("\r\n ->:%s",uart_data);
			uart_non_empty = 0;
			rt_memset(uart_data,0,uart_len);
			uart_len = 0;		
		}
	}
	else
	{
		if ( STOP == stop_flag )	//恢复timer2任务，其他任务由timer2恢复
		{
			if (timer2 != RT_NULL)
				rt_timer_start( timer2 );
			
			stop_flag = START;
		}
	}

}
/////////////////////////////

void adc1_dma(void)
{
	/* ADC1转换的电压值通过DMA方式传到sram*/
	extern __IO vu16 ADC_ConvertedValue;
	/*计算后的温度值*/
	float Current_Temp;
	
	//计算方法一
	Current_Temp= (1.43- ADC_ConvertedValue*3.3/4096)*1000 / 4.3+ 25 ;
	//计算方法二
	//Current_Temp=(float)(V25-ADC_ConvertedValue)/AVG_SLOPE+25;	
	printf("\r\n temperature is：%f℃",Current_Temp);
}
////////////////////////////////
void rt_thread_uart1(void * rt_tick)
{
	while(1)
	{
		uart1();
		rt_thread_delay( *((rt_tick_t*)rt_tick) );
	}
}
void rt_thread_led_breath(void * rt_tick)
{
	while(1)
	{
		led_breath();
		LED1_ONOFF(Bit_SET);		//D1(PC13)灭
//		//如果挂起的是线程自身，在调用该函数后，必须调用 rt_schedule()进行系统调度
//		rt_thread_suspend( tid_led_breath );
//		rt_schedule();
		
//		rt_thread_delay( *((rt_tick_t*)rt_tick) );
	}
}

void rt_thread_adc(void * rt_tick)
{
	while(1)
	{
		adc1_dma();
		rt_thread_delay( *((rt_tick_t*)rt_tick) );
	}
}

void timerout1( void * parameter)
{
	adc1_dma();
}

void timerout2( void * parameter)
{
	static u8 count;
	printf("\r\n<<====POWER_LOW & STANDBY====%d>>",count++);
	//恢复剩余任务:激活呼吸灯，激活测温
	if (tid_led_breath != RT_NULL)
		rt_thread_resume(tid_led_breath);
	if (timer1 != RT_NULL)
		rt_timer_start( timer1 );
	
}



int main()
{
	extern void ADC_Configuration(void);
	
	rt_base_t level;
	rt_tick_t uart1_rt_tick = RT_TICK_PER_SECOND/10;
	rt_tick_t led_breath_rt_tick = RT_TICK_PER_SECOND;
	rt_tick_t ADC1_rt_tick = RT_TICK_PER_SECOND*2;

	ADC_Configuration();
	USART1_Config();
	NVIC_Configuration();//uart1中断向量配置初始化
	led1_gpio_init();
	
	level = rt_hw_interrupt_disable();//临界关闭中断，防止建立任务过程启动调度。
	//创建多个动态线程,自动返回生成的控制块tid
	tid_uart1 = rt_thread_create( "uart1",
								  rt_thread_uart1,
								  (void*)&uart1_rt_tick,
								  1024,2,5 );
	if (tid_uart1 != RT_NULL)
		rt_thread_startup( tid_uart1 );	//线程开启
	
	tid_led_breath = rt_thread_create( "led_breath",
									   rt_thread_led_breath,
									   (void*)&led_breath_rt_tick,
									   1024,4,5 );	
	if (tid_led_breath != RT_NULL)
		rt_thread_startup( tid_led_breath );	
	
//	tid_ADC1 = rt_thread_create( "ADC1",
//								 rt_thread_adc,
//								 (void*)&ADC1_rt_tick,
//								 1024,5,5 );	
//	if (tid_ADC1 != RT_NULL)
//		rt_thread_startup( tid_ADC1 );							

	/////////定时器任务/////////
	timer1 = rt_timer_create( "timer1",
							  timerout1,
							  NULL,
							  ADC1_rt_tick,
							  RT_TIMER_FLAG_PERIODIC);	//周期定时
	if (timer1 != RT_NULL)
		rt_timer_start( timer1 );	
	
	timer2 = rt_timer_create( "timer2",
							  timerout2,
							  NULL,
							  6*RT_TICK_PER_SECOND+10,
							  RT_TIMER_FLAG_PERIODIC);	
	if (timer2 != RT_NULL)
		rt_timer_start( timer2 );	

	
	rt_hw_interrupt_enable( level );//打开中断，中断程序启用，调度开始	
		
	while(1)
	{
		rt_thread_delay(RT_TICK_PER_SECOND);
//		printf("<<====POWER_LOW & STANDBY====%d>>\n",count++);
//		rt_thread_resume(tid_led_breath);
    }
	
}


