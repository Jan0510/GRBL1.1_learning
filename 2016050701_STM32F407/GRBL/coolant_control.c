#include "grbl.h"

//¿‰»¥∆˜≥ı ºªØ(checked)
void coolant_init()
{
	GPIO_InitTypeDef GPIO_CoolantStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE);
	#ifdef ENABLE_M7
		GPIO_CoolantStructure.GPIO_Pin= (1<<COOLANT_FLOOD_BIT)| (1 << COOLANT_MIST_BIT);
	#else
		GPIO_CoolantStructure.GPIO_Pin= (1<<COOLANT_FLOOD_BIT);
	#endif
	GPIO_CoolantStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_CoolantStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_CoolantStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_CoolantStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIO_COOLANT, &GPIO_CoolantStructure);
	coolant_stop();
	
}

//¿‰»¥∆˜Õ£÷π(checked)
void coolant_stop()
{
	GPIO_ResetBits(GPIO_COOLANT, (1<<COOLANT_FLOOD_BIT));
	#ifdef ENABLE_M7
		GPIO_ResetBits(GPIO_COOLANT, (1<<COOLANT_MIST_BIT));
	#endif
}
	\
// ¿‰»¥∆˜…Ë÷√◊¥Ã¨(checked)
void coolant_set_state(uint8_t mode)
{
	if (mode == COOLANT_FLOOD_ENABLE) 
	{
		GPIO_SetBits(GPIO_COOLANT, (1<<COOLANT_FLOOD_BIT));
		#ifdef ENABLE_M7  
		} 
		else if (mode == COOLANT_MIST_ENABLE) 
		{
			GPIO_SetBits(GPIO_COOLANT, (1<<COOLANT_MIST_BIT));
		#endif
	} 
	else
	{
		coolant_stop();
	}
}

// ¿‰»¥∆˜∆Ù∂Ø(unchecked!!!)
void coolant_run(uint8_t mode)
{
	if (sys.state == STATE_CHECK_MODE) { return; }
	protocol_buffer_synchronize(); // Ensure coolant turns on when specified in program.  
	coolant_set_state(mode);
}
	


