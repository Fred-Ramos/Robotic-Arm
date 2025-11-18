#ifndef FSM_H
#define FSM_H

#include <Arduino.h>

//STATE MACHINE DEFINITION
typedef struct {                           //define finite state machine structure (store state machine data)
  int state, new_state;                    //current state ; state to change to
  unsigned long tes, tis;                  //time entering the new state ; time in the same state
} fsm;

//SET STATE MACHINE'S NEW STATE
void set_state(fsm& fsm_this, int new_state)  //function that changes the state (fsm_t object)
{
  if (fsm_this.state != new_state){           //if state changes:
    fsm_this.state = new_state;
    fsm_this.tes = millis();                  //tes updates to current time
    fsm_this.tis = 0;                           //tis resets
  }
}

#endif