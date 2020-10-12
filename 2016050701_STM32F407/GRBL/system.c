#include "grbl.h"
volatile uint8_t sys_probe_state;
volatile uint8_t sys_rt_exec_state;  
volatile uint8_t sys_rt_exec_alarm;

//系统初始化
void system_init() 
{
	GPIO_InitTypeDef  	GPIO_InitStructure;	
	NVIC_InitTypeDef   	NVIC_InitStructure;
	EXTI_InitTypeDef   	EXTI_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //使能GPIOB时钟

	GPIO_InitStructure.GPIO_Pin = (1 << CONTROL_INT)|CONTROL_MASK; //中断脚，复位，进给保持，cycle_start引脚
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;//普通输入模式
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100M
	#ifdef DISABLE_CONTROL_PIN_PULL_UP
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	#else
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	#endif
    GPIO_Init(GPIO_CONTROL, &GPIO_InitStructure);
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能SYSCFG时钟,外部中断用
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOB, EXTI_PinSource1);//连接到中断线1
	
	EXTI_InitStructure.EXTI_Line = EXTI_Line1;//LINE1
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;//中断事件
	#ifdef DISABLE_CONTROL_PIN_PULL_UP
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //沿触发 
	#else
		EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling; //下降沿触发 
	#endif	
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;//使能LINE1
    EXTI_Init(&EXTI_InitStructure);//配置
	
	NVIC_InitStructure.NVIC_IRQChannel = EXTI1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x00;//抢占优先级0
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00;//子优先级0
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//使能外部中断通道
    NVIC_Init(&NVIC_InitStructure);//配置
}

//控制引脚中断函数
void EXTI1_IRQHandler(void)
{
	delay_ms(10);
	if(EXTI_GetITStatus(EXTI_Line1)!=RESET)
	{
		uint16_t  pin = (GPIO_ReadInputData(GPIOB) & CONTROL_MASK);
		#ifndef INVERT_ALL_CONTROL_PINS
			pin ^= CONTROL_INVERT_MASK;
		#endif
		
		if (pin) 
		{
			if (bit_istrue(pin,bit(RESET_BIT))) 
			{
				mc_reset();
				printString("reset\r\n");
			} 
			else if (bit_istrue(pin,bit(CYCLE_START_BIT))) 
			{
				bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
				printString("cycle\r\n");
				#ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
					} 
					else if (bit_istrue(pin,bit(FEED_HOLD_BIT))) 
					{	
						bit_true(sys_rt_exec_state, EXEC_FEED_HOLD); 
						printString("feed\r\n");
				#else
					} 
					else if (bit_istrue(pin,bit(SAFETY_DOOR_BIT))) 
					{
						bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
						printString("door\r\n");
				#endif
			}
		
		}
		EXTI_ClearITPendingBit(EXTI_Line1);
	}

}

// 检查安全门状态
uint8_t system_check_safety_door_ajar()
{
	#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
		#ifdef INVERT_CONTROL_PIN
			return(bit_istrue(CONTROL_PIN,bit(SAFETY_DOOR_BIT)));
		#else
			return(bit_isfalse(CONTROL_PIN,bit(SAFETY_DOOR_BIT)));
		#endif
	#else
		return(false); // Input pin not enabled, so just return that it's closed.
	#endif
}
	
void system_execute_startup(char *line) 
{
	uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++)
	{
    if (!(settings_read_startup_line(n, line))) 
		{
      report_status_message(STATUS_SETTING_READ_FAIL);
		}
		else 
		{
      if (line[0] != 0)
			{
        printString(line); // Echo startup line to indicate execution.
        report_status_message(gc_execute_line(line));
      }
    } 
  }  

}
	
uint8_t system_execute_line(char *line) 
{   
  uint8_t char_counter = 1; 
  uint8_t helper_var = 0; // Helper variable
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[(char_counter+1)] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[char_counter] ) {
        case '$' : // Prints Grbl settings
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
          else { report_grbl_settings(); }
          break;
        case 'G' : // Prints gcode parser state
          // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
          report_gcode_modes();
          break;   
        case 'C' : // Set check g-code mode [IDLE/CHECK]
          // Perform reset when toggling off. Check g-code mode should only work if Grbl
          // is idle and ready, regardless of alarm locks. This is mainly to keep things
          // simple and consistent.
          if ( sys.state == STATE_CHECK_MODE ) { 
            mc_reset(); 
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break; 
        case 'X' : // Disable alarm lock [ALARM]
          if (sys.state == STATE_ALARM) { 
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Don't run startup script. Prevents stored moves in startup from causing accidents.
            if (system_check_safety_door_ajar()) { // Check safety door switch before returning.
              bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              protocol_execute_realtime(); // Enter safety door mode.
            }
          } // Otherwise, no effect.
          break;                   
    //  case 'J' : break;  // Jogging methods
          // TODO: Here jogging can be placed for execution as a seperate subprogram. It does not need to be 
          // susceptible to other realtime commands except for e-stop. The jogging function is intended to
          // be a basic toggle on/off with controlled acceleration and deceleration to prevent skipped 
          // steps. The user would supply the desired feedrate, axis to move, and direction. Toggle on would
          // start motion and toggle off would initiate a deceleration to stop. One could 'feather' the
          // motion by repeatedly toggling to slow the motion to the desired location. Location data would 
          // need to be updated real-time and supplied to the user through status queries.
          //   More controlled exact motions can be taken care of by inputting G0 or G1 commands, which are 
          // handled by the planner. It would be possible for the jog subprogram to insert blocks into the
          // block buffer without having the planner plan them. It would need to manage de/ac-celerations 
          // on its own carefully. This approach could be effective and possibly size/memory efficient.  
//       }
//       break;
      }
      break;
    default : 
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[char_counter] ) {
        case '#' : // Print Grbl NGC parameters
          if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;          
        case 'H' : // Perform homing cycle [IDLE/ALARM]
          if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { 
            sys.state = STATE_HOMING; // Set system state variable
            // Only perform homing if Grbl is idle or lost.
            
            // TODO: Likely not required.
            if (system_check_safety_door_ajar()) { // Check safety door switch before homing.
              bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              protocol_execute_realtime(); // Enter safety door mode.
            }
            
            
            mc_homing_cycle(); 
            if (!sys.abort) {  // Execute startup scripts after successful homing.
              sys.state = STATE_IDLE; // Set to IDLE when complete.
              st_go_idle(); // Set steppers to the settings idle state before returning.
              system_execute_startup(line); 
            }
          } else { return(STATUS_SETTING_DISABLED); }
          break;
        case 'I' : // Print or store build info. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { 
            settings_read_build_info(line);
            report_build_info(line);
          } else { // Store startup line [IDLE/ALARM]
            if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
            helper_var = char_counter; // Set helper variable as counter to start of user info line.
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            settings_store_build_info(line);
          }
          break; 
        case 'R' : // Restore defaults [IDLE/ALARM]
          if (line[++char_counter] != 'S') { return(STATUS_INVALID_STATEMENT); }
          if (line[++char_counter] != 'T') { return(STATUS_INVALID_STATEMENT); }
          if (line[++char_counter] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (line[char_counter+2] != 0) { return(STATUS_INVALID_STATEMENT); }                        
          switch (line[++char_counter]) {
            case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Force reset to ensure settings are initialized correctly.
          break;
        case 'N' : // Startup lines. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // Print startup lines
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
            helper_var = true;  // Set helper_var to flag storing method. 
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line); // Set helper_var to returned status code.
            if (helper_var) { return(helper_var); }
            else { 
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }    
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}
	
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
	float pos;
    pos = steps[idx]/settings.steps_per_mm[idx];
	return(pos);
}
	
void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
	uint8_t idx;
	for (idx=0; idx<N_AXIS; idx++) 
	{
		position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
	}
	return;
}
	
