// Possible values for characteristic CURRENT_DOOR_STATE:
#define CURRENT_DOOR_STATE_OPEN    0
#define CURRENT_DOOR_STATE_CLOSED  1
#define CURRENT_DOOR_STATE_OPENING 2
#define CURRENT_DOOR_STATE_CLOSING 3
#define CURRENT_DOOR_STATE_STOPPED 4
#define CURRENT_DOOR_STATE_UNKNOWN 255

#define TARGET_DOOR_STATE_OPEN    0
#define TARGET_DOOR_STATE_CLOSED  1
#define TARGET_DOOR_STATE_UNKNOWN 255
#define SENSOR_CLOSED   0
#define SENSOR_RELEASED 1

#include "SlGate.h"
#include "Lock.h"

////////////////////////////
int ButtonArray[3];
bool isTimered = false;
hw_timer_t * ButtonTimer = NULL;
portMUX_TYPE DoortimerMux = portMUX_INITIALIZER_UNLOCKED;
  
void IRAM_ATTR isr(void* arg) {
    Sensor* s = static_cast<Sensor*>(arg);
    s->changed = true;
};

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&DoortimerMux);
  isTimered = true;
  portEXIT_CRITICAL_ISR(&DoortimerMux);
};
///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////
void SL_GATE::initTimer(){
  ButtonTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(ButtonTimer, &onTimer, true);
  timerAlarmWrite(ButtonTimer, 1000000, false);
  timerAlarmEnable(ButtonTimer);  
};

SL_GATE::SL_GATE() : Service::GarageDoorOpener(){

  LOG1("Constructing Gate…\n");
  CurrentDoorState  = new Characteristic::CurrentDoorState();
  TargetDoorState =   new Characteristic::TargetDoorState();
  ObstructionDetected=new Characteristic::ObstructionDetected();
  Name=new Characteristic::Name("Gate");
  GateDoor *GatePosition;  
                         
  pinMode(OpenPin,OUTPUT); 
  digitalWrite(OpenPin,LOW);
  ButtonArray[0] = OpenPin;
                                              
  pinMode(ClosePin,OUTPUT);
  digitalWrite(ClosePin,LOW);
  ButtonArray[1] = ClosePin;  
                      
  pinMode(StopPin,OUTPUT);
  digitalWrite(StopPin,LOW);
  ButtonArray[2] = StopPin;
    
  pinMode(ClSensorPin.PIN, INPUT_PULLUP);
  pinMode(OpSensorPin.PIN, INPUT_PULLUP);
  pinMode(ObSensorPin.PIN, INPUT_PULLUP);
  attachInterruptArg(ClSensorPin.PIN, isr, &ClSensorPin, CHANGE);
  attachInterruptArg(OpSensorPin.PIN, isr, &OpSensorPin, CHANGE);
  attachInterruptArg(ObSensorPin.PIN, isr, &ObSensorPin, CHANGE);
  
  GatePosition = new GateDoor(this);
  
  PollCurrentState();

  LOG1("Constructing Gate successful!\n");
  //LOG1(WiFi.localIP());  
} // end constructor

void SL_GATE::PollCurrentState(){
    if (digitalRead(ClSensorPin.PIN)      == SENSOR_CLOSED &&  CurrentDoorState-> getVal() !=CURRENT_DOOR_STATE_CLOSED)        
                                                              {
                                                              ClSensorPin.stableState = SENSOR_CLOSED;
                                                              FullyClosed();
                                                              }
    
    else if (digitalRead(OpSensorPin.PIN) == SENSOR_CLOSED && CurrentDoorState->  getVal() != CURRENT_DOOR_STATE_OPEN)   
                                                              {
                                                              OpSensorPin.stableState = SENSOR_CLOSED;
                                                              FullyOpened();
                                                              }
    
    else if                                                   (CurrentDoorState-> getVal() != CURRENT_DOOR_STATE_OPEN)   
                                                              {CurrentDoorState-> setVal(CURRENT_DOOR_STATE_OPEN);   
                                                              TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);
                                                              OpSensorPin.stableState = SENSOR_RELEASED; 
                                                              ClSensorPin.stableState = SENSOR_RELEASED;
                                                              }
    
    if (digitalRead(ObSensorPin.PIN)      == SENSOR_CLOSED && !ObstructionDetected->getVal())        
                                                              {ObstructionDetected->setVal(true);
                                                              ObSensorPin.stableState = SENSOR_CLOSED;

                                                              GatePosition->ObstructionDetected->setVal(true);
                                                              }
    
}

boolean SL_GATE::update(){            

    if(TargetDoorState->getNewVal()==TARGET_DOOR_STATE_OPEN &&
       CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPENING){ 
                                                              
                                                              
      LOG1("-----------Opening Gate----------\n");
      CurrentDoorState->setVal(CURRENT_DOOR_STATE_OPENING);   // set the current-state value to 2, which means "opening"
          
      digitalWrite(ClosePin,LOW);
      digitalWrite(OpenPin,HIGH);
      CycleTimeBegin = millis();    
  
      initTimer();
      
    
    } else if(TargetDoorState->getNewVal()==TARGET_DOOR_STATE_CLOSED &&
              CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSING){

        // анализ препятствия
        if (!ObstructionDetected->getVal()){
          LOG1("----------Closing Gate----------\n");                                 // else the target-state value is set to 1, and HomeKit is requesting the door to be in the closed position
          CurrentDoorState->setVal(CURRENT_DOOR_STATE_CLOSING);   // set the current-state value to 3, which means "closing"         
        
          digitalWrite(OpenPin,LOW);
          digitalWrite(ClosePin,HIGH);
          CycleTimeBegin = millis();
  
          initTimer();  

        } else if (CurrentDoorState->getVal() == CURRENT_DOOR_STATE_OPENING || CurrentDoorState->getVal() == CURRENT_DOOR_STATE_OPEN) 
                  {TargetDoorState->setVal(TARGET_DOOR_STATE_OPEN);}

    } 
    
    LOG1("----------UpdateOver----------\n");
    CycleTimeBegin = millis();
    return(true);                                   // return true to indicate the update was successful (otherwise create code to return false if some reason you could not turn on the LED)
  
} // update

void SL_GATE::loop(){                                     
      // если истек таймер удержания кнопки, убиваем таймер, отжимаем кнопку
      if (isTimered){
        portENTER_CRITICAL(&DoortimerMux);
        timerEnd(ButtonTimer);
        ButtonTimer = NULL;
        isTimered = false;
        portEXIT_CRITICAL(&DoortimerMux);        
        LOG1("----------Timer deinited----------\n");
        for (int i=0; i<3; i++) {
          digitalWrite(ButtonArray[i],LOW);
        }        
      }
      
      // если сработал концевик, фиксируем время срабатывания, и игнорируем его изменения-дребезг после обработки события
      if (ClSensorPin.changed && (millis() - ClPortPollBegin)>PortPollTimeout) {
        ClSensorPin.changed = false;
        ClPortPollBegin = millis();
        CycleTimeBegin = millis();
        // если новоее состояние отличается от предыдущего стабильного
        if (digitalRead(ClSensorPin.PIN) == SENSOR_CLOSED && ClSensorPin.stableState == SENSOR_RELEASED)   {
                                                              LOG1("----------ClSensorPin.SENSOR_CLOSED----------\n");
                                                              // обновляем стабильное
                                                              ClSensorPin.stableState = SENSOR_CLOSED;       
                                                              // устанавливаем состояние Закрыто
                                                              FullyClosed();}
        
        if (digitalRead(ClSensorPin.PIN) == SENSOR_RELEASED && ClSensorPin.stableState == SENSOR_CLOSED) {
                                                              LOG1("----------ClSensorPin.SENSOR_RELEASED----------\n");  
                                                              ClSensorPin.stableState = SENSOR_RELEASED;
                                                              // если состояние итак открывается, то ничего менять не будем
                                                              // по сути игнорим открывание через HAP и реагируем на брелок
                                                              if ( CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPENING ){ 
                                                              CurrentDoorState->setVal(CURRENT_DOOR_STATE_OPENING);
                                                              TargetDoorState->setVal(TARGET_DOOR_STATE_OPEN);}}
      
      } else if ( ((millis() - ClPortPollBegin)>PortPollTimeout) && ClSensorPin.stableState == SENSOR_CLOSED && CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSED ){
                                                              CurrentDoorState->setVal(CURRENT_DOOR_STATE_CLOSED);}
      
      if (OpSensorPin.changed && (millis() - OpPortPollBegin)>PortPollTimeout) {
        OpSensorPin.changed = false;
        OpPortPollBegin = millis();
        CycleTimeBegin = millis();
        if (digitalRead(OpSensorPin.PIN) == SENSOR_CLOSED && OpSensorPin.stableState == SENSOR_RELEASED)   {
                                                              LOG1("----------OpSensorPin.SENSOR_CLOSED----------\n");
                                                              OpSensorPin.stableState = SENSOR_CLOSED;
                                                              FullyOpened();}
        
        if (digitalRead(OpSensorPin.PIN) == SENSOR_RELEASED && OpSensorPin.stableState == SENSOR_CLOSED) {
                                                              LOG1("----------OpSensorPin.SENSOR_RELEASED----------\n");
                                                              OpSensorPin.stableState = SENSOR_RELEASED;
                                                              // если состояние итак закрывается, то ничего менять не будем
                                                              // по сути игнорим закрывание через HAP и реагируем на брелок
                                                              if (CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSING){ 
                                                              CurrentDoorState->setVal(CURRENT_DOOR_STATE_CLOSING);
                                                              TargetDoorState->setVal(TARGET_DOOR_STATE_CLOSED);}}     
      
      } else if ( ((millis() - OpPortPollBegin)>PortPollTimeout) && OpSensorPin.stableState == SENSOR_CLOSED && CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPEN ){
                                                              CurrentDoorState->setVal(CURRENT_DOOR_STATE_OPEN);}

      if (ObSensorPin.changed && (millis() - ObPortPollBegin)>PortPollTimeout) {
        ObSensorPin.changed = false;
        ObPortPollBegin = millis();
        CycleTimeBegin = millis();
        if (digitalRead(ObSensorPin.PIN) == SENSOR_CLOSED && ObSensorPin.stableState == SENSOR_RELEASED)    
                                                              {LOG1("----------Optocoupler.SENSOR_CLOSED----------\n");
                                                               ObstructionDetected->setVal(true);
                                                               ObSensorPin.stableState = SENSOR_CLOSED;
                                                               
                                                               GatePosition->ObstructionDetected->setVal(true);
                                                               }
        
        if (digitalRead(ObSensorPin.PIN) == SENSOR_RELEASED && ObSensorPin.stableState == SENSOR_CLOSED)                                                        
                                                              {LOG1("----------Optocoupler.SENSOR_RELEASED----------\n");
                                                               ObstructionDetected->setVal(false);
                                                               ObSensorPin.stableState = SENSOR_RELEASED;

                                                               GatePosition->ObstructionDetected->setVal(false);
                                                               }
      
      } else if ( ((millis() - ObPortPollBegin)>PortPollTimeout) && ObSensorPin.stableState == SENSOR_CLOSED && !ObstructionDetected->getVal() )
                                                              {ObstructionDetected->setVal(true);
                                                              GatePosition->ObstructionDetected->setVal(true);
                                                              }
        
        else if ( ((millis() - ObPortPollBegin)>PortPollTimeout) && ObSensorPin.stableState == SENSOR_RELEASED && ObstructionDetected->getVal() )
                                                              {ObstructionDetected->setVal(false);
                                                              GatePosition->ObstructionDetected->setVal(false);
                                                              }
                                                              
      
      if ( (millis() - CycleTimeBegin) > CycleTimeout ) { 
        LOG1("----------CycleTimeBegin.updated----------\n");
        CycleTimeBegin = millis();
        PollCurrentState();
      }
}// loop 

void SL_GATE::FullyOpened(){
  CurrentDoorState-> setVal(CURRENT_DOOR_STATE_OPEN);   
  TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);
  
  GatePosition->CurrentPosition->setVal(FULLY_OPENED);
  GatePosition->TargetPosition->setVal(FULLY_OPENED);
  GatePosition->PositionState->setVal(2);  
}

void SL_GATE::FullyClosed(){
  CurrentDoorState-> setVal(CURRENT_DOOR_STATE_CLOSED); 
  TargetDoorState->   setVal(TARGET_DOOR_STATE_CLOSED);
  
  GatePosition->CurrentPosition->setVal(FULLY_CLOSED);
  GatePosition->TargetPosition->setVal(FULLY_CLOSED);
  GatePosition->PositionState->setVal(2);  
}

//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
GateDoor::GateDoor(SL_GATE* gate) : Service::Door(){
      CurrentPosition      = new Characteristic::CurrentPosition(GateDoorState.CurrentPosition);
      TargetPosition       = new Characteristic::TargetPosition(GateDoorState.CurrentPosition);
      PositionState        = new Characteristic::PositionState(2);
      ObstructionDetected  = new Characteristic::ObstructionDetected(false);
      HoldPosition         = new Characteristic::HoldPosition(false);
      this->gate=gate;
      NVS_init();
      //return(this);
   
}

void GateDoor::NVS_init(){
  LOG1("----------Reading storage----------\n");
  nvs_open("GATE",NVS_READWRITE,&gateNVS);
      
  if(!nvs_get_blob(gateNVS,"GATEDATA",NULL,&nvslen)) {                       // if found GATE data in NVS
  nvs_get_blob(gateNVS,"GATEDATA",&GateDoorState,&nvslen); }              // retrieve data  
}

boolean GateDoor :: update(){            
  return(true);                                   
} // update
    
void GateDoor::loop(){
  return;
  if ( (millis() - CycleTimeBegin) > PollTimeout ) {
        
    gate->TargetDoorState-> setVal( gate->CurrentDoorState->getVal() );
    gate->CurrentDoorState-> setVal( !gate->CurrentDoorState->getVal() );
        
    CycleTimeBegin = millis();
  }
} //loop   

nvs_handle GateDoor::gateNVS;