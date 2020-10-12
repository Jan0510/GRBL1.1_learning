#include "grbl.h"

void spindle_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE); 	//使能PORTF时钟	
	#ifdef VARIABLE_SPINDLE
		GPIO_PinAFConfig(GPIOF,GPIO_PinSource9,GPIO_AF_TIM14); //GPIOF9复用为定时器14
		GPIO_InitStructure.GPIO_Pin = SPINDLE_PWM_MASK;           //GPIOF9
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
		GPIO_Init(GPIO_SPINDLE,&GPIO_InitStructure);              //初始化PF9
		#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
			GPIO_InitStructure.GPIO_Pin = (1<< SPINDLE_ENABLE_BIT);           //GPIOF8
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
			GPIO_Init(GPIO_SPINDLE,&GPIO_InitStructure);
		#endif     
	// Configure no variable spindle and only enable pin.
	#else  
			GPIO_InitStructure.GPIO_Pin = (1<< SPINDLE_ENABLE_BIT);           //GPIOF8
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
			GPIO_Init(GPIO_SPINDLE,&GPIO_InitStructure);
	#endif
  
	#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
		GPIO_InitStructure.GPIO_Pin = (1<< SPINDLE_DIRECTION_BIT);           //GPIOF9
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;        //复用功能
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度100MHz
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //上拉
		GPIO_Init(GPIO_SPINDLE,&GPIO_InitStructure);
	#endif
	spindle_stop();	
}

void spindle_stop()
{
	#ifdef VARIABLE_SPINDLE
		TIM_Cmd(TIM14, DISABLE); 
		#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
			#ifdef INVERT_SPINDLE_ENABLE_PIN
				GPIO_SetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#else
				GPIO_ResetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#endif
		#endif
	#else
			#ifdef INVERT_SPINDLE_ENABLE_PIN
				GPIO_SetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#else
				GPIO_ResetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#endif
	#endif  
}

void spindle_set_state(uint8_t state, float rpm)
{
	// Halt or set spindle direction and rpm. 
	if (state == SPINDLE_DISABLE) 
	{
		spindle_stop();
	} 
	else 
	{
		uint16_t current_pwm;
		#ifndef USE_SPINDLE_DIR_AS_ENABLE_PIN
			if (state == SPINDLE_ENABLE_CW) 
			{
				GPIO_ResetBits(GPIO_SPINDLE, (1<<SPINDLE_DIRECTION_BIT));
			} 
			else 
			{
				GPIO_SetBits(GPIO_SPINDLE, (1<<SPINDLE_DIRECTION_BIT));
			}
		#endif
		#ifdef VARIABLE_SPINDLE
			// TODO: Install the optional capability for frequency-based output for servos.
			TIM14_PWM_Init(1000-1,64-1);
			if (rpm <= 0.0) { spindle_stop(); } // RPM should never be negative, but check anyway.
			else 
			{
				#define SPINDLE_RPM_RANGE (SPINDLE_MAX_RPM-SPINDLE_MIN_RPM)
				if ( rpm < SPINDLE_MIN_RPM ) { rpm = 0; } 
				else 
				{ 
					rpm -= SPINDLE_MIN_RPM; 
					if ( rpm > SPINDLE_RPM_RANGE ) { rpm = SPINDLE_RPM_RANGE; } // Prevent integer overflow
				}
				current_pwm = floor( rpm*(PWM_MAX_VALUE/SPINDLE_RPM_RANGE) + 0.5);
				#ifdef MINIMUM_SPINDLE_PWM
					if (current_pwm < MINIMUM_SPINDLE_PWM) { current_pwm = MINIMUM_SPINDLE_PWM; }
				#endif
				
				TIM_SetCompare1(TIM14,current_pwm);// Set PWM pin output
				// On the Uno, spindle enable and PWM are shared, unless otherwise specified.
				#ifdef USE_SPINDLE_DIR_AS_ENABLE_PIN
					#ifdef INVERT_SPINDLE_ENABLE_PIN
						GPIO_ResetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
					#else
						GPIO_SetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
					#endif
				#endif
			}
      
		#else
			// NOTE: Without variable spindle, the enable bit should just turn on or off, regardless
			// if the spindle speed value is zero, as its ignored anyhow.      
			#ifdef INVERT_SPINDLE_ENABLE_PIN
				GPIO_ResetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#else
				GPIO_SetBits(GPIO_SPINDLE, (1<<SPINDLE_ENABLE_BIT));
			#endif
		#endif

	}
}

void spindle_run(uint8_t state, float rpm)
{
  if (sys.state == STATE_CHECK_MODE) { return; }
  protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.  
  spindle_set_state(state, rpm);
}

