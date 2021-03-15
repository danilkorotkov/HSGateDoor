#include "Lock.h" 

hw_timer_t * LockTimer = NULL;
portMUX_TYPE LocktimerMux = portMUX_INITIALIZER_UNLOCKED;
bool isDoorTimered = false;

void IRAM_ATTR onDoorTimer(){
  portENTER_CRITICAL_ISR(&LocktimerMux);
  isDoorTimered = true;
  portEXIT_CRITICAL_ISR(&LocktimerMux);
};

void DoorLock :: initDoorTimer(){
  LockTimer = timerBegin(1, 80, true);
  timerAttachInterrupt(LockTimer, &onDoorTimer, true);
  timerAlarmWrite(LockTimer, 1000000, false);
  timerAlarmEnable(LockTimer);  
};
 
DoorLock :: DoorLock() : Service::LockMechanism(){
    LockCurrentState  = new Characteristic::LockCurrentState(CURRENT_LOCK_STATE_CLOSED);
    LockTargetState   = new Characteristic::LockTargetState (TARGET_LOCK_STATE_CLOSED);
    pinMode(LockPin,OUTPUT); 
    digitalWrite(LockPin,LOW);

}

boolean DoorLock :: update(){            
  if(LockTargetState->getNewVal()==TARGET_LOCK_STATE_OPEN &&
    LockCurrentState->getVal() != CURRENT_LOCK_STATE_OPEN){ 
    LOG1("-----------Opening Door----------\n");
    LockCurrentState->setVal(CURRENT_LOCK_STATE_OPEN);   
    
    digitalWrite(LockPin,HIGH);
    initDoorTimer();
  }
  return(true);                                   // return true to indicate the update was successful (otherwise create code to return false if some reason you could not turn on the LED)
  
} // update
  
void DoorLock ::loop(){
  if (isDoorTimered){
    portENTER_CRITICAL(&LocktimerMux);
    timerEnd(LockTimer);
    LockTimer = NULL;
    isDoorTimered = false;
    portEXIT_CRITICAL(&LocktimerMux);        
    LOG1("----------Timer door deinited----------\n");
    digitalWrite(LockPin,LOW);
    LockCurrentState->setVal(CURRENT_LOCK_STATE_CLOSED);
    LockTargetState->setVal(TARGET_LOCK_STATE_CLOSED);
  }
}
