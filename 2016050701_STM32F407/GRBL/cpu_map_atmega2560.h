
#ifdef GRBL_PLATFORM
#error "cpu_map already defined: GRBL_PLATFORM=" GRBL_PLATFORM
#endif


#define GRBL_PLATFORM "Atmega2560"

// Serial port pins
#define SERIAL_RX USART0_RX_vect
#define SERIAL_UDRE USART0_UDRE_vect

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.

#define GPIO_STEP_PULSE	  GPIOA
#define STEP_DDR          DDRA 
#define STEP_PORT         PORTA
#define STEP_PIN          PINA
#define X_STEP_BIT        2 // stm32 Pin PA2
#define Y_STEP_BIT        3 // stm32 Pin PA3
#define Z_STEP_BIT        4 // stm32 Pin PA4
#define STEP_MASK ((1<<X_STEP_BIT)|(1<<Y_STEP_BIT)|(1<<Z_STEP_BIT)) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.

#define GPIO_STEP_DIR	    GPIOC
#define DIRECTION_DDR     DDRC 
#define DIRECTION_PORT    PORTC
#define DIRECTION_PIN     PINC
#define X_DIRECTION_BIT   1 
#define Y_DIRECTION_BIT   2 
#define Z_DIRECTION_BIT   3 
#define DIRECTION_MASK ((1<<X_DIRECTION_BIT)|(1<<Y_DIRECTION_BIT)|(1<<Z_DIRECTION_BIT)) // All direction bits

// Define stepper driver enable/disable output pin.
/*===================================================================================
#define STEPPERS_DISABLE_DDR   DDRB // 方向控制寄存器 stepper_Init() 函数调用
#define STEPPERS_DISABLE_PORT  PORTB // 数据寄存器 
#define STEPPERS_DISABLE_BIT   7 // MEGA2560 Digital Pin 13
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)
===================================================================================*/
#define GPIO_STEP_DISABLE	   GPIOC
#define STEPPERS_DISABLE_DDR   DDRC
#define STEPPERS_DISABLE_PORT  PORTC
#define STEPPERS_DISABLE_BIT   0 // MEGA2560 Digital Pin 13
#define STEPPERS_DISABLE_MASK (1<<STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins and limit interrupt vectors. 
// NOTE: All limit bit pins must be on the same port

#define GPIO_LIMIT		GPIOD
#define X_LIMIT_BIT     1 // MEGA2560 Digital Pin 10
#define Y_LIMIT_BIT     2 // MEGA2560 Digital Pin 11
#define Z_LIMIT_BIT     3 // MEGA2560 Digital Pin 12
#define LIMIT_INT       0  // Pin change interrupt enable pin
#define LIMIT_INT_vect  PCINT0_vect 
#define LIMIT_PCMSK     PCMSK0 // Pin change interrupt register
#define LIMIT_MASK ((1<<X_LIMIT_BIT)|(1<<Y_LIMIT_BIT)|(1<<Z_LIMIT_BIT)) // All limit bits

//Define LPOSwitch/HPOSwitch/GasSwitch/AutoSwitch ports

#define GPIO_Switch  GPIOE
#define LPOSwitch_BIT  0
#define HPOSwitch_BIT  1
#define GasSwitch_BIT  2
#define AutoSwitch_BIT  3
#define Switch_MASK ((1<<LPOSwitch_BIT)|(1<<HPOSwitch_BIT)|(1<<GasSwitch_BIT)|(1<<AutoSwitch_BIT))

// Define spindle enable and spindle direction output pins.

#define GPIO_SPINDLE 	GPIOF
#define SPINDLE_ENABLE_BIT      8 // MEGA2560 Digital Pin 6
#define SPINDLE_DIRECTION_BIT   10 // MEGA2560 Digital Pin 5

// Define flood and mist coolant enable output pins.
// NOTE: Uno analog pins 4 and 5 are reserved for an i2c interface, and may be installed at
// a later date if flash and memory space allows.

#define GPIO_COOLANT 		  GPIOF
#define COOLANT_FLOOD_BIT     6 // MEGA2560 Digital Pin 8
#ifdef ENABLE_M7 // Mist coolant disabled by default. See config.h to enable/disable.
#define COOLANT_MIST_BIT    7 // MEGA2560 Digital Pin 9
#endif  

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).

#define GPIO_CONTROL	  GPIOB
#define RESET_BIT         5  // MEGA2560 Analog Pin 8
#define FEED_HOLD_BIT     6  // MEGA2560 Analog Pin 9
#define CYCLE_START_BIT   7  // MEGA2560 Analog Pin 10
#define SAFETY_DOOR_BIT   2  // MEGA2560 Analog Pin 11
#define CONTROL_INT       1  // Pin change interrupt enable pin
#define CONTROL_INT_vect  PCINT2_vect
#define CONTROL_PCMSK     PCMSK2 // Pin change interrupt register
#define CONTROL_MASK ((1<<RESET_BIT)|(1<<FEED_HOLD_BIT)|(1<<CYCLE_START_BIT)|(1<<SAFETY_DOOR_BIT))
#define CONTROL_INVERT_MASK CONTROL_MASK // May be re-defined to only invert certain control pins.

// Define probe switch input pin.

#define PROBE_GPIO		GPIOF
#define PROBE_BIT       5  // MEGA2560 Analog Pin 15
#define PROBE_MASK      (1<<PROBE_BIT)

// Start of PWM & Stepper Enabled Spindle
#ifdef VARIABLE_SPINDLE
	// Advanced Configuration Below You should not need to touch these variables
	// Set Timer up to use TIMER4B which is attached to Digital Pin 7
	#define PWM_MAX_VALUE       	1000.0
	#define TCCRA_REGISTER		TCCR4A
	#define TCCRB_REGISTER		TCCR4B
	#define OCR_REGISTER		  OCR4B

	#define COMB_BIT			    COM4B1
	#define WAVE0_REGISTER		WGM40
	#define WAVE1_REGISTER		WGM41
	#define WAVE2_REGISTER		WGM42
	#define WAVE3_REGISTER		WGM43

	#define SPINDLE_PWM_BIT		9 // MEGA2560 Digital Pin 97
	#define SPINDLE_PWM_MASK 		(1<<SPINDLE_PWM_BIT)
#endif // End of VARIABLE_SPINDLE
