#ifndef LOCK_H
#define LOCK_H

#include "HomeSpan.h" 

//LOCK set
#define CURRENT_LOCK_STATE_OPEN    0
#define CURRENT_LOCK_STATE_CLOSED  1

#define TARGET_LOCK_STATE_OPEN    0
#define TARGET_LOCK_STATE_CLOSED  1

void IRAM_ATTR onDoorTimer();

struct DoorLock;

struct DoorLock: Service::LockMechanism{
  SpanCharacteristic *LockCurrentState;
  SpanCharacteristic *LockTargetState;
  int LockPin = 16;

  DoorLock();
  boolean update();
  void loop();
  void initDoorTimer();
};
#endif /LOCK_H/
