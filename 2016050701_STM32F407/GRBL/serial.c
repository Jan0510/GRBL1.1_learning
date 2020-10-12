#include "grbl.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];//串口接收缓存区大小64B
uint8_t serial_rx_buffer_head = 0;         //首指针
volatile uint8_t serial_rx_buffer_tail = 0;//尾指针

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];//串口发送缓存区大小128B
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

void serial_init() // USART1 初始化函数
{
	GPIO_InitTypeDef GPIO_USARTStructure;
	USART_InitTypeDef USART_InitStructure;               //串口类型定义初始化结构体
	NVIC_InitTypeDef NVIC_InitStructure;                 //中断优先级管理类型定义结构体
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
	
	
	//USART1端口配置
	GPIO_USARTStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_USARTStructure.GPIO_Mode = GPIO_Mode_AF;      //复用功能
	GPIO_USARTStructure.GPIO_Speed = GPIO_Speed_50MHz; //速度50MHz
	GPIO_USARTStructure.GPIO_OType = GPIO_OType_PP;    //推挽复用输出
	GPIO_USARTStructure.GPIO_PuPd = GPIO_PuPd_UP;      //上拉
	GPIO_Init(GPIOA,&GPIO_USARTStructure);             //初始化PA9，PA10
	
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1，函数的第三个参数在stm32f4xx_gpio.h中IS_DPIO_AF(AF)列表。
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;               //波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     //一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;        //无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //收发模式
	USART_Init(USART1, &USART_InitStructure);                  //初始化串口1
	
	USART_Cmd(USART1, ENABLE);                     //使能串口1 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //开启串口1中断
		
	//USART1 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;       //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	                        //根据指定的参数初始化NVIC寄存器
}

void serial_write(uint8_t data)                           //向串口缓存区写入一字节数据
{
	// Calculate next head
	uint8_t next_head = serial_tx_buffer_head + 1;
	if (next_head == TX_BUFFER_SIZE) 
	{ 
		next_head = 0; 
	}
	while (next_head == serial_tx_buffer_tail) //如果缓存区没有空间，则等待
	{ 	
		if (sys_rt_exec_state & EXEC_RESET) //复位则退出等待。
		{ 
			return; 
		} 

	}
	// Store data and advance head
	serial_tx_buffer[serial_tx_buffer_head] = data;
	serial_tx_buffer_head = next_head;
	USART_ITConfig(USART1, USART_IT_TXE , ENABLE); //开启USART1发送寄存器为空中断,当发送缓存区为空时，产生中断进行下以字节的写入。
}

uint8_t serial_read()                            //从输入缓存区读取一字节数据
{
	uint8_t tail = serial_rx_buffer_tail; 
	if (serial_rx_buffer_head == tail) //缓存区没有数据
	{
		return SERIAL_NO_DATA;
	} 
	else 
	{
		uint8_t data = serial_rx_buffer[tail]; 
		tail++;
		if (tail == RX_BUFFER_SIZE) { tail = 0; }
		serial_rx_buffer_tail = tail; 
		return data;
	}
}

void USART1_IRQHandler(void)                           // USART1相关中断服务函数
{
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) // USART1发送寄存器为空中断
	{
		uint8_t tail = serial_tx_buffer_tail;
		USART_SendData(USART1, serial_tx_buffer[tail]); // 发送串口发送寄存器尾指针指向的一个字节
		tail++;
		if (tail == TX_BUFFER_SIZE) 
		{ 
			tail = 0; 
		}
		serial_tx_buffer_tail = tail;
		if (tail == serial_tx_buffer_head) 
		{
			USART_ITConfig(USART1, USART_IT_TXE , DISABLE);// 如果串口发送缓冲器尾指针指向头指针,说明缓冲器数据发送完毕,关闭USART1发送寄存器为空中断
		}
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //USART1接收中断
	{
		uint8_t data = USART_ReceiveData(USART1);           //从USART1读取一个字节数据
		uint8_t next_head;
		// 获取实时指令字符
		switch (data)
		{
			case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT);
															bit_true_atomic(settings.status_report_mask,BITFLAG_RT_STATUS_SERIAL_RX);break; // Set as true
			case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
			case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
			case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
			case CMD_RESET:         mc_reset(); break;                                           // Call motion control reset routine.
			case CMD_BUFFER_COUNT:  printReceiveBufferCount(); break;                            //返回接收缓存区字节个数
			default:                                                                             //将数据写入接收缓存区 
				next_head = serial_rx_buffer_head + 1;
				if (next_head == RX_BUFFER_SIZE) 
				{ 
					next_head = 0; 
				}		
				// Write data to buffer unless it is full.
				if (next_head != serial_rx_buffer_tail) 
				{
					serial_rx_buffer[serial_rx_buffer_head] = data;
					serial_rx_buffer_head = next_head;    
				}
		}
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
	}
	
}

void serial_reset_read_buffer() 
{
	serial_rx_buffer_tail = serial_rx_buffer_head;
}

uint8_t serial_get_rx_buffer_count()          //获取串口接收缓存区字符个数
{
	uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_rx_buffer_head >= rtail) 
	{ 
		return(serial_rx_buffer_head-rtail); 
	}
	return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

uint8_t serial_get_tx_buffer_count()          //获取串口发送缓存区字符个数
{
	uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_tx_buffer_head >= ttail) 	
	{ 
		return(serial_tx_buffer_head-ttail); 
	}
	return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}

void printReceiveBufferCount()                //打印接收缓冲区占用个数，ADC采样值
{
	uint8_t count;
	u16 adc;
	count = serial_get_rx_buffer_count();
	print_uint8_base10(count);
	printString("\r\n");
	adc = Get_Adc(0);
	print_uint32_base10((uint32_t)adc);
	printString("\r\n");
}


