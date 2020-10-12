#ifndef limits_h
#define limits_h 

#include "grbl.h"

// Initialize the limits module
void limits_init(void);

// Enable hard limits
void limits_enable(void);

// Disables hard limits.
void limits_disable(void);

// Returns limit state as a bit-wise uint8 variable.
uint8_t limits_get_state(void);

// Perform one portion of the homing cycle based on the input settings.
void limits_go_home(uint8_t cycle_mask);

// Check for soft limit violations
void limits_soft_check(float *target);

#endif


