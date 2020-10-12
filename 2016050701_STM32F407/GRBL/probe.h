#ifndef probe_h
#define probe_h 
#include "grbl.h"

// Values that define the probing state machine.  
#define PROBE_OFF     0 // Probing disabled or not in use. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

// Probe pin initialization routine.
void probe_init(void);

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to 
// appropriately set the pin logic according to setting for normal-high/normal-low operation 
// and the probing cycle modes for toward-workpiece/away-from-workpiece. 
void probe_configure_invert_mask(uint8_t is_probe_away);

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state(void);

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void probe_state_monitor(void);

#endif


