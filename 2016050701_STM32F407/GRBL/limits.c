#include "grbl.h"
extern long lround(double /*x*/);

#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR  1.5 // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR  5.0 // Must be > 1 to ensure limit switch is cleared.
#endif

//��λ��ϵͳ��ʼ��(checked)
void limits_init() 
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //ʹ��GPIOʱ��
	GPIO_InitStructure.GPIO_Pin = (1<<LIMIT_INT) | LIMIT_MASK;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//����
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
	#ifdef DISABLE_LIMIT_PIN_PULL_UP
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	#else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	#endif
	GPIO_Init(GPIOD, &GPIO_InitStructure);//��ʼ��
	if (bit_istrue(settings.flags,BITFLAG_HARD_LIMIT_ENABLE))
	{
		NVIC_InitTypeDef   NVIC_InitStructure;
		EXTI_InitTypeDef   EXTI_InitStructure;
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ��SYSCFGʱ��,�ⲿ�ж���	  
		SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOD, EXTI_PinSource0);//PD0 ���ӵ��ж���0
		
		EXTI_InitStructure.EXTI_Line = EXTI_Line0;//LINE0
		EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//�ж��¼�
		#ifdef DISABLE_LIMIT_PIN_PULL_UP
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�����ش��� 
		#else
			EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //�½��ش��� 
		#endif
		EXTI_InitStructure.EXTI_LineCmd = ENABLE;//ʹ��LINE0
		EXTI_Init(&EXTI_InitStructure);//����
		
		NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn;//�ⲿ�ж�0
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//��ռ���ȼ�0
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x01;//�����ȼ�1
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//ʹ���ⲿ�ж�ͨ��
		NVIC_Init(&NVIC_InitStructure);//���� 		
	}
	else
	{
		limits_disable(); 
	}	
}

// ��λ���ܽ���(unchecked)	
void limits_disable()
{
	EXTI->IMR &= ~EXTI_Line0;
}
	
// ��ȡ��λ����״̬(checked)
uint8_t limits_get_state()
{
	uint8_t limit_state = 0;
	uint8_t pin = ((uint8_t)GPIO_ReadInputData(GPIOD) & LIMIT_MASK);
	#ifdef INVERT_LIMIT_PIN_MASK
		pin ^= INVERT_LIMIT_PIN_MASK;
	#endif
	if (bit_isfalse(settings.flags,BITFLAG_INVERT_LIMIT_PINS)) { pin ^= LIMIT_MASK; }
	if (pin) 
	{  
		uint8_t idx;
		for (idx=0; idx<N_AXIS; idx++) 
		{
			if (pin & get_limit_pin_mask(idx)) { limit_state |= (1 << idx); }
		}
	}
	return(limit_state);
}

// ��λ�����жϺ���(checked)
#ifndef ENABLE_SOFTWARE_DEBOUNCE
	void  EXTI0_IRQHandler(void)
	{
		delay_ms(10);
		if(EXTI_GetITStatus(EXTI_Line0)!=RESET)
		{	
				
			if (sys.state != STATE_ALARM) 
			{ 
				if (!(sys_rt_exec_alarm)) 
				{
					#ifdef HARD_LIMIT_FORCE_STATE_CHECK
						// Check limit pin state. 
						if (limits_get_state()) 
						{
							mc_reset(); 
							// Initiate system kill.
							bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
						}
					#else
						mc_reset(); // Initiate system kill.
						bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate hard limit critical event
					#endif
				}
			}
			EXTI_ClearITPendingBit(EXTI_Line0); //���LINE0�ϵ��жϱ�־λ 
		}	
	}
#else
#endif

void limits_go_home(uint8_t cycle_mask) 
{
	uint8_t n_cycle;
	uint8_t step_pin[N_AXIS];
	float target[N_AXIS];
	float max_travel;
	uint8_t idx;
	bool approach;
	float homing_rate;
	uint8_t limit_state, axislock, n_active_axis;
	#ifdef COREXY
		int32_t off_axis_position;
	#endif
	int32_t set_axis_position;
	
	if (sys.abort) { return; } // Block if system reset has been issued.

	// Initialize
	/*=============================================
	uint8_t n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
	uint8_t step_pin[N_AXIS];
	float target[N_AXIS];
	float max_travel = 0.0;
	uint8_t idx;
	=============================================*/
	n_cycle = (2*N_HOMING_LOCATE_CYCLE+1);
	max_travel = 0.0;
	
	for (idx=0; idx<N_AXIS; idx++) 
	{  
		// Initialize step pin masks
		step_pin[idx] = get_step_pin_mask(idx);
		#ifdef COREXY    
			if ((idx==A_MOTOR)||(idx==B_MOTOR)) { step_pin[idx] = (get_step_pin_mask(X_AXIS)|get_step_pin_mask(Y_AXIS)); } 
		#endif

		if (bit_istrue(cycle_mask,bit(idx))) 
		{ 
			// Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
			// NOTE: settings.max_travel[] is stored as a negative value.
			max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR)*settings.max_travel[idx]);
		}
	}

	// Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
	/*===============================================
	bool approach = true;
	float homing_rate = settings.homing_seek_rate;

	uint8_t limit_state, axislock, n_active_axis;
	===============================================*/
	approach = true;
	homing_rate = settings.homing_seek_rate;
	do {

		system_convert_array_steps_to_mpos(target,sys.position);

		// Initialize and declare variables needed for homing routine.
		axislock = 0;
		n_active_axis = 0;
		for (idx=0; idx<N_AXIS; idx++) 
		{
			// Set target location for active axes and setup computation for homing rate.
			if (bit_istrue(cycle_mask,bit(idx))) 
			{
				n_active_axis++;
				sys.position[idx] = 0;
				// Set target direction based on cycle mask and homing cycle approach state.
				// NOTE: This happens to compile smaller than any other implementation tried.
				if (bit_istrue(settings.homing_dir_mask,bit(idx))) 
				{
				if (approach) { target[idx] = -max_travel; }
				else { target[idx] = max_travel; }
				} 
				else
				{ 
					if (approach) { target[idx] = max_travel; }
					else { target[idx] = -max_travel; }
				}        
				// Apply axislock to the step port pins active in this cycle.
				axislock |= step_pin[idx];
			}

		}
		homing_rate *= sqrt(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
		sys.homing_axis_lock = axislock;

		plan_sync_position(); // Sync planner position to current machine position.
    
		// Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
		#ifdef USE_LINE_NUMBERS
			plan_buffer_line(target, homing_rate, false, HOMING_CYCLE_LINE_NUMBER); // Bypass mc_line(). Directly plan homing motion.
		#else
			plan_buffer_line(target, homing_rate, false); // Bypass mc_line(). Directly plan homing motion.
		#endif
    
		st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
		st_wake_up(); // Initiate motion
		do {
			if (approach) 
			{
				// Check limit state. Lock out cycle axes when they change.
				limit_state = limits_get_state();
				for (idx=0; idx<N_AXIS; idx++) 
				{
					if (axislock & step_pin[idx])					
					{
						if (limit_state & (1 << idx)) { axislock &= ~(step_pin[idx]); }
					}
				}
				sys.homing_axis_lock = axislock;
			}

			st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.

			// Exit routines: No time to run protocol_execute_realtime() in this loop.
			if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_STOP)) 
			{
				// Homing failure: Limit switches are still engaged after pull-off motion
				if ( (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET)) ||  // Safety door or reset issued
					(!approach && (limits_get_state() & cycle_mask)) ||  // Limit switch still engaged after pull-off motion
					( approach && (sys_rt_exec_state & EXEC_CYCLE_STOP)) )  // Limit switch not found during approach.
				{
					mc_reset(); // Stop motors, if they are running.
					protocol_execute_realtime();
					return;
				} 
				else 
				{
					// Pull-off motion complete. Disable CYCLE_STOP from executing.
					bit_false_atomic(sys_rt_exec_state,EXEC_CYCLE_STOP);
					break;
				} 
			}

		} while (STEP_MASK & axislock);

		st_reset(); // Immediately force kill steppers and reset step segment buffer.
		plan_reset(); // Reset planner buffer to zero planner current position and to clear previous motions.

		delay_ms(settings.homing_debounce_delay); // Delay to allow transient dynamics to dissipate.

		// Reverse direction and reset homing rate for locate cycle(s).
		approach = !approach;

		// After first cycle, homing enters locating phase. Shorten search to pull-off distance.
		if (approach) 
		{ 
			/*==============================================================
			max_travel = settings.homing_pulloff*HOMING_AXIS_LOCATE_SCALAR; 
			==============================================================*/
			max_travel = settings.homing_pulloff*(float)HOMING_AXIS_LOCATE_SCALAR; 
			homing_rate = settings.homing_feed_rate;
		} 
		else 
		{
			max_travel = settings.homing_pulloff;    
			homing_rate = settings.homing_seek_rate;
		}
    
	} while (n_cycle-- > 0);
      
	// The active cycle axes should now be homed and machine limits have been located. By 
	// default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
	// can be on either side of an axes, check and set axes machine zero appropriately. Also,
	// set up pull-off maneuver from axes limit switches that have been homed. This provides
	// some initial clearance off the switches and should also help prevent them from falsely
	// triggering when hard limits are enabled or when more than one axes shares a limit pin.
	/*=====================================
	#ifdef COREXY
		int32_t off_axis_position = 0;
	#endif
	int32_t set_axis_position;
	======================================*/
	#ifdef COREXY
		off_axis_position = 0;
	#endif
		// Set machine positions for homed limit switches. Don't update non-homed axes.
	for (idx=0; idx<N_AXIS; idx++) 
	{
		// NOTE: settings.max_travel[] is stored as a negative value.
		if (cycle_mask & bit(idx)) {
		#ifdef HOMING_FORCE_SET_ORIGIN
			set_axis_position = 0;
		#else 
			if ( bit_istrue(settings.homing_dir_mask,bit(idx)) ) 
			{
				set_axis_position = lround((settings.max_travel[idx]+settings.homing_pulloff)*settings.steps_per_mm[idx]);
			} 
			else 
			{
				set_axis_position = lround(-settings.homing_pulloff*settings.steps_per_mm[idx]);
			}
		#endif
      
		#ifdef COREXY
			if (idx==X_AXIS)
			{ 
				off_axis_position = (sys.position[B_MOTOR] - sys.position[A_MOTOR])/2;
				sys.position[A_MOTOR] = set_axis_position - off_axis_position;
				sys.position[B_MOTOR] = set_axis_position + off_axis_position;          
			} 
			else if (idx==Y_AXIS) 
			{
				off_axis_position = (sys.position[A_MOTOR] + sys.position[B_MOTOR])/2;
				sys.position[A_MOTOR] = off_axis_position - set_axis_position;
				sys.position[B_MOTOR] = off_axis_position + set_axis_position;
			}
			else 
			{
				sys.position[idx] = set_axis_position;
			}        
		#else 
			sys.position[idx] = set_axis_position;
		#endif

		}
	}
	plan_sync_position(); // Sync planner position to homed machine position.
    
	// sys.state = STATE_HOMING; // Ensure system state set as homing before returning. 
}


// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
void limits_soft_check(float *target)
{
	uint8_t idx;
	for (idx=0; idx<N_AXIS; idx++) 
	{
   
		#ifdef HOMING_FORCE_SET_ORIGIN
			// When homing forced set origin is enabled, soft limits checks need to account for directionality.
			// NOTE: max_travel is stored as negative
			if (bit_istrue(settings.homing_dir_mask,bit(idx))) 
			{
				if (target[idx] < 0 || target[idx] > -settings.max_travel[idx]) { sys.soft_limit = true; }
			} 
			else 
			{
				if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { sys.soft_limit = true; }
			}
		#else  
			// NOTE: max_travel is stored as negative
			if (target[idx] > 0 || target[idx] < settings.max_travel[idx]) { sys.soft_limit = true; }
		#endif
    
		if (sys.soft_limit) 
		{
			// Force feed hold if cycle is active. All buffered blocks are guaranteed to be within 
			// workspace volume so just come to a controlled stop so position is not lost. When complete
			// enter alarm mode.
			if (sys.state == STATE_CYCLE) 
			{
				bit_true_atomic(sys_rt_exec_state, EXEC_FEED_HOLD);
			do {
				protocol_execute_realtime();
				if (sys.abort) { return; }
			} while ( sys.state != STATE_IDLE );
		}
    
		mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
		bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_SOFT_LIMIT|EXEC_CRITICAL_EVENT)); // Indicate soft limit critical event
		protocol_execute_realtime(); // Execute to enter critical event loop and system abort
		return;
		}
	}
}



