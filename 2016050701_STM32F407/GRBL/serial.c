#include "grbl.h"


uint8_t serial_rx_buffer[RX_BUFFER_SIZE];//���ڽ��ջ�������С64B
uint8_t serial_rx_buffer_head = 0;         //��ָ��
volatile uint8_t serial_rx_buffer_tail = 0;//βָ��

uint8_t serial_tx_buffer[TX_BUFFER_SIZE];//���ڷ��ͻ�������С128B
uint8_t serial_tx_buffer_head = 0;
volatile uint8_t serial_tx_buffer_tail = 0;

void serial_init() // USART1 ��ʼ������
{
	GPIO_InitTypeDef GPIO_USARTStructure;
	USART_InitTypeDef USART_InitStructure;               //�������Ͷ����ʼ���ṹ��
	NVIC_InitTypeDef NVIC_InitStructure;                 //�ж����ȼ��������Ͷ���ṹ��
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
	
	
	//USART1�˿�����
	GPIO_USARTStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_USARTStructure.GPIO_Mode = GPIO_Mode_AF;      //���ù���
	GPIO_USARTStructure.GPIO_Speed = GPIO_Speed_50MHz; //�ٶ�50MHz
	GPIO_USARTStructure.GPIO_OType = GPIO_OType_PP;    //���츴�����
	GPIO_USARTStructure.GPIO_PuPd = GPIO_PuPd_UP;      //����
	GPIO_Init(GPIOA,&GPIO_USARTStructure);             //��ʼ��PA9��PA10
	
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1�������ĵ�����������stm32f4xx_gpio.h��IS_DPIO_AF(AF)�б�
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;               //����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;     //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;        //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	                //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);                  //��ʼ������1
	
	USART_Cmd(USART1, ENABLE);                     //ʹ�ܴ���1 
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //��������1�ж�
		
	//USART1 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;       //����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3; //��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;       //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	        //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	                        //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
}

void serial_write(uint8_t data)                           //�򴮿ڻ�����д��һ�ֽ�����
{
	// Calculate next head
	uint8_t next_head = serial_tx_buffer_head + 1;
	if (next_head == TX_BUFFER_SIZE) 
	{ 
		next_head = 0; 
	}
	while (next_head == serial_tx_buffer_tail) //���������û�пռ䣬��ȴ�
	{ 	
		if (sys_rt_exec_state & EXEC_RESET) //��λ���˳��ȴ���
		{ 
			return; 
		} 

	}
	// Store data and advance head
	serial_tx_buffer[serial_tx_buffer_head] = data;
	serial_tx_buffer_head = next_head;
	USART_ITConfig(USART1, USART_IT_TXE , ENABLE); //����USART1���ͼĴ���Ϊ���ж�,�����ͻ�����Ϊ��ʱ�������жϽ��������ֽڵ�д�롣
}

uint8_t serial_read()                            //�����뻺������ȡһ�ֽ�����
{
	uint8_t tail = serial_rx_buffer_tail; 
	if (serial_rx_buffer_head == tail) //������û������
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

void USART1_IRQHandler(void)                           // USART1����жϷ�����
{
	if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET) // USART1���ͼĴ���Ϊ���ж�
	{
		uint8_t tail = serial_tx_buffer_tail;
		USART_SendData(USART1, serial_tx_buffer[tail]); // ���ʹ��ڷ��ͼĴ���βָ��ָ���һ���ֽ�
		tail++;
		if (tail == TX_BUFFER_SIZE) 
		{ 
			tail = 0; 
		}
		serial_tx_buffer_tail = tail;
		if (tail == serial_tx_buffer_head) 
		{
			USART_ITConfig(USART1, USART_IT_TXE , DISABLE);// ������ڷ��ͻ�����βָ��ָ��ͷָ��,˵�����������ݷ������,�ر�USART1���ͼĴ���Ϊ���ж�
		}
		USART_ClearITPendingBit(USART1,USART_IT_TXE);
	}
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //USART1�����ж�
	{
		uint8_t data = USART_ReceiveData(USART1);           //��USART1��ȡһ���ֽ�����
		uint8_t next_head;
		// ��ȡʵʱָ���ַ�
		switch (data)
		{
			case CMD_STATUS_REPORT: bit_true_atomic(sys_rt_exec_state, EXEC_STATUS_REPORT);
															bit_true_atomic(settings.status_report_mask,BITFLAG_RT_STATUS_SERIAL_RX);break; // Set as true
			case CMD_CYCLE_START:   bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START); break; // Set as true
			case CMD_FEED_HOLD:     bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD); break; // Set as true
			case CMD_SAFETY_DOOR:   bit_true_atomic(sys_rt_exec_state, EXEC_SAFETY_DOOR); break; // Set as true
			case CMD_RESET:         mc_reset(); break;                                           // Call motion control reset routine.
			case CMD_BUFFER_COUNT:  printReceiveBufferCount(); break;                            //���ؽ��ջ������ֽڸ���
			default:                                                                             //������д����ջ����� 
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

uint8_t serial_get_rx_buffer_count()          //��ȡ���ڽ��ջ������ַ�����
{
	uint8_t rtail = serial_rx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_rx_buffer_head >= rtail) 
	{ 
		return(serial_rx_buffer_head-rtail); 
	}
	return (RX_BUFFER_SIZE - (rtail-serial_rx_buffer_head));
}

uint8_t serial_get_tx_buffer_count()          //��ȡ���ڷ��ͻ������ַ�����
{
	uint8_t ttail = serial_tx_buffer_tail; // Copy to limit multiple calls to volatile
	if (serial_tx_buffer_head >= ttail) 	
	{ 
		return(serial_tx_buffer_head-ttail); 
	}
	return (TX_BUFFER_SIZE - (ttail-serial_tx_buffer_head));
}

void printReceiveBufferCount()                //��ӡ���ջ�����ռ�ø�����ADC����ֵ
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


