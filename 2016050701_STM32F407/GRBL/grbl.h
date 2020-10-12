#ifndef grbl_h
#define grbl_h
// ≤‚ ‘Ω· ¯∫Û…æ≥˝
//=====================
#define LINE_BUFFER_SIZE 80
//=====================

//≤‚ ‘∫Û…æ≥˝
//==========================
#define COOLANT_FLOOD_ENABLE 2 // M8
//==========================

// Grbl versioning system
#define GRBL_VERSION "0.9j"
#define GRBL_VERSION_BUILD "20160317"

#include "stm32f4xx.h"
#include "delay.h"

#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "ctype.h"
#include "math.h"
#include "stdbool.h"

#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "system.h"
#include "defaults.h"
#include "cpu_map.h"
#include "coolant_control.h"
#include "protocol.h"
#include "serial.h"
#include "print.h"
#include "probe.h"
#include "iic.h"
#include "24cxx.h"
#include "eeprom.h"
#include "report.h"
#include "limits.h"
#include "timer.h"
#include "spindle_control.h"
#include "gcode.h"
#include "motion_control.h"
#include "planner.h"
#include "stepper.h"
#include "adc_distance.h"
#include "switch_io.h"




#endif
