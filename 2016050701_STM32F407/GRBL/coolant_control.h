#ifndef coolant_control_h
#define coolant_control_h 
#include "grbl.h"

void coolant_init(void);
void coolant_stop(void);
void coolant_set_state(uint8_t mode);
void coolant_run(uint8_t mode);

#endif

