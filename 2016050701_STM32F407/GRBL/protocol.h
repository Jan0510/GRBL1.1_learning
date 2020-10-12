#ifndef protocol_h
#define protocol_h

// Line buffer size from the serial input stream to be executed.
// NOTE: Not a problem except for extreme cases, but the line buffer size can be too small
// and g-code blocks can get truncated. Officially, the g-code standards support up to 256
// characters. In future versions, this will be increased, when we know how much extra
// memory space we can invest into here or we re-write the g-code parser not to have this 
// buffer.
#ifndef LINE_BUFFER_SIZE
  #define LINE_BUFFER_SIZE 80
#endif

// Starts Grbl main loop. It handles all incoming characters from the serial port and executes
// them as they complete. It is also responsible for finishing the initialization procedures.
void protocol_main_loop(void);

// Checks and executes a realtime command at various stop points in main program
void protocol_execute_realtime(void);

// Notify the stepper subsystem to start executing the g-code program in buffer.
// void protocol_cycle_start();

// Reinitializes the buffer after a feed hold for a resume.
// void protocol_cycle_reinitialize(); 

// Initiates a feed hold of the running program
// void protocol_feed_hold();

// Executes the auto cycle feature, if enabled.
void protocol_auto_cycle_start(void);

// Block until all buffered steps are executed
void protocol_buffer_synchronize(void);

#endif
