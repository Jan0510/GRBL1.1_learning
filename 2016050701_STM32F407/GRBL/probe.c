#include "grbl.h"

uint8_t probe_invert_mask;

// probe��ϵͳ��ʼ��(checked)
void probe_init()
{
	GPIO_InitTypeDef GPIO_ProbeStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,ENABLE); //ʹ��GPIOBʱ��
	
	GPIO_ProbeStructure.GPIO_Pin = PROBE_MASK;
	GPIO_ProbeStructure.GPIO_Mode = GPIO_Mode_IN; 
	GPIO_ProbeStructure.GPIO_Speed = GPIO_Medium_Speed;
	#ifdef DISABLE_PROBE_PIN_PULL_UP
		GPIO_ProbeStructure.GPIO_PuPd = GPIO_PuPd_DOWN; // �������� Ĭ�ϵ͵�ƽ
	#else
		GPIO_ProbeStructure.GPIO_PuPd = GPIO_PuPd_UP; // �������� Ĭ�ϸߵ�ƽ
	#endif
	GPIO_Init(PROBE_GPIO, &GPIO_ProbeStructure);

	probe_configure_invert_mask(false);
}
	
// ��probe��������ȡ��(checked)
void probe_configure_invert_mask(uint8_t is_probe_away)
{
	probe_invert_mask = 0; // Initialize as zero.
	if (bit_isfalse(settings.flags,BITFLAG_INVERT_PROBE_PIN)) { probe_invert_mask ^= PROBE_MASK; }
	if (is_probe_away) { probe_invert_mask ^= PROBE_MASK; }
}

// ��ȡprobe����״̬(checked)
uint8_t probe_get_state()
{
	return((GPIO_ReadInputData(PROBE_GPIO) & PROBE_MASK) ^ probe_invert_mask); 
}
	
	
void probe_state_monitor()
{
	if (sys_probe_state == PROBE_ACTIVE) 
	{
		if (probe_get_state()) 
		{
			sys_probe_state = PROBE_OFF;
			memcpy(sys.probe_position, sys.position, sizeof(sys.position));
			bit_true(sys_rt_exec_state, EXEC_MOTION_CANCEL);
		}
	}

}

