#ifndef _ROMI_H_
#define _ROMI_H_

#include "nonBlockingMillis.h"

#define GO_HANDLE(x) { current_state = x; }
#define WAIT_THEN_GO_STATE(n, t)  \
		    current_state = NON_BLOCKING_DELAY_STATE; \
	      next_state = n;  \
	    	blocking_time = t  \

#define WAIT_NONBLOCKING_MS(ms) if (!nonBlockingDelay(ms)) { \
                                GO_HANDLE(IDLE_STATE);  \
                                break; \
       	                        } 

#define HANDLE_THEN_GO(firstState, SecondState) current_state = firstState; \
                                                next_state = SecondState;   \
                                                
       				
#define WAIT_NONBLOCKING_SAME_MS(ms, state) if (!nonBlockingDelay(ms)) { \
                                            GO_HANDLE(state); \
                                            break; \
                                            }  

#define WAIT_AND_GO(ms)   static uint16_t tm {0}; \
                          if(tm < ms) {tm++; break;} \
                          else {tm=0;}
						
#define BREAK_AND_GO(st) GO_HANDLE(st); \
						             break;					      				 

enum ROMI_STATES {
  IDLE_STATE,
  NON_BLOCKING_DELAY_STATE,
  WAIT_BEFORE_TURNING,
  TURN_ROMI_STATE,
  PATH_TRACING,
  MOTOR_STOP,
  STOP_SYSTEM,
  SLOW_DOWN,
  ACCELERATED_START,
  
};

extern uint8_t current_state;
extern uint8_t next_state;
extern uint32_t blocking_time;

#endif _ROMI_H_
