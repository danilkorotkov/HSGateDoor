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

int OpenPin     = 18;                                       
int ClosePin    = 19;
int StopPin     = 21;
  
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
  //GateDoor *GatePosition;  
                         
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
    LOG1("polling...\n");
  
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
    
    else if                                                   (CurrentDoorState-> getVal() != CURRENT_DOOR_STATE_OPEN || GatePosition->PositionState->getVal() != DOOR_STOPPED)   
                                                              {CurrentDoorState-> setVal(CURRENT_DOOR_STATE_OPEN);   
                                                              TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);
                                                              OpSensorPin.stableState = SENSOR_RELEASED; 
                                                              ClSensorPin.stableState = SENSOR_RELEASED;

                                                              GatePosition->TargetPosition->setVal(50);
                                                              GatePosition->CurrentPosition->setVal(50);
                                                              GatePosition->PositionState->setVal(DOOR_STOPPED);
                                                              GatePosition->valid = false;
                                                              }
    
    if (digitalRead(ObSensorPin.PIN)      == SENSOR_CLOSED && !ObstructionDetected->getVal())        
                                                              {ObstructionDetected->setVal(true);
                                                              ObSensorPin.stableState = SENSOR_CLOSED;

                                                              GatePosition->ObstructionDetected->setVal(true);
                                                              }
    
  LOG1("polling over\n");
}

boolean SL_GATE::update(){            

    if(TargetDoorState->getNewVal()==TARGET_DOOR_STATE_OPEN &&
       CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPENING){ 
                                                                                                                     
      LOG1("-----------Opening Gate----------\n");
      FullyOpenExtern();
      Open();
    
    } else if(TargetDoorState->getNewVal()==TARGET_DOOR_STATE_CLOSED &&
              CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSING){

        // анализ препятствия
        if (!ObstructionDetected->getVal()){
          LOG1("----------Closing Gate----------\n");                                
          FullyCloseExtern();
          Close();
          
        } else if (CurrentDoorState->getVal() == CURRENT_DOOR_STATE_OPENING || CurrentDoorState->getVal() == CURRENT_DOOR_STATE_OPEN) 
                  {TargetDoorState->setVal(TARGET_DOOR_STATE_OPEN);}

    } 
    
    LOG1("----------UpdateOver----------\n");
    CycleTimeBegin = millis();
    return(true);                                  
  
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
                                                              FullyClosed();
                                                              }
        
        if (digitalRead(ClSensorPin.PIN) == SENSOR_RELEASED && ClSensorPin.stableState == SENSOR_CLOSED) {
                                                              LOG1("----------ClSensorPin.SENSOR_RELEASED----------\n");  
                                                              
                                                              // если состояние итак открывается, то ничего менять не будем
                                                              // по сути игнорим открывание через HAP и реагируем только на брелок
                                                              if ( CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPENING ){ 
                                                                FullyOpenExtern();
                                                                TargetDoorState->setVal(TARGET_DOOR_STATE_OPEN);
                                                              }
                                                              ClSensorPin.stableState = SENSOR_RELEASED;
                                                              }
      
      } else if ( ((millis() - ClPortPollBegin)>PortPollTimeout) && ClSensorPin.stableState == SENSOR_CLOSED && CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSED ){
                                                              FullyClosed();}
      
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
                                                              
                                                              // если состояние итак закрывается, то ничего менять не будем
                                                              // по сути игнорим закрывание через HAP и реагируем на брелок
                                                              if (CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSING){ 
                                                                FullyCloseExtern();
                                                                TargetDoorState->setVal(TARGET_DOOR_STATE_CLOSED);
                                                              }
                                                              OpSensorPin.stableState = SENSOR_RELEASED;
                                                              }     
      
      } else if ( ((millis() - OpPortPollBegin)>PortPollTimeout) && OpSensorPin.stableState == SENSOR_CLOSED && CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPEN ){
                                                              FullyOpened();}

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
  GatePosition->PositionState->setVal(DOOR_STOPPED);
  GatePosition->valid = true;
}

void SL_GATE::FullyOpenExtern(){
  CurrentDoorState->setVal(CURRENT_DOOR_STATE_OPENING);
          
  GatePosition->PositionState ->setVal(DOOR_OPENING);
  GatePosition->TargetPosition->setVal(FULLY_OPENED);
  GatePosition->GateDoorState.Direction = 1;
  GatePosition->updateTime = millis();
      
  if (ClSensorPin.stableState == SENSOR_CLOSED) {GatePosition->GateDoorState.fromZeroPos = true;}
}

void SL_GATE::FullyCloseExtern(){
  CurrentDoorState->setVal(CURRENT_DOOR_STATE_CLOSING);

  GatePosition->PositionState ->setVal(DOOR_CLOSING);
  GatePosition->TargetPosition->setVal(FULLY_CLOSED);    
  GatePosition->GateDoorState.Direction = 0;
  GatePosition->updateTime = millis();
          
  if (OpSensorPin.stableState == SENSOR_CLOSED) {GatePosition->GateDoorState.fromZeroPos = true;} 
}

void SL_GATE::FullyClosed(){
  CurrentDoorState-> setVal(CURRENT_DOOR_STATE_CLOSED); 
  TargetDoorState->   setVal(TARGET_DOOR_STATE_CLOSED);
  
  GatePosition->CurrentPosition->setVal(FULLY_CLOSED);
  GatePosition->TargetPosition->setVal(FULLY_CLOSED);
  GatePosition->PositionState->setVal(DOOR_STOPPED);
  GatePosition->valid = true;    
}

void SL_GATE::Open(){
  digitalWrite(ClosePin,LOW);
  digitalWrite(StopPin,LOW);
  digitalWrite(OpenPin,HIGH);
  
  initTimer();  
}

void SL_GATE::Close(){
  digitalWrite(OpenPin,LOW);
  digitalWrite(StopPin,LOW);
  digitalWrite(ClosePin,HIGH);
  
  initTimer();  
}

void SL_GATE::Stop(){
  digitalWrite(OpenPin,LOW);
  digitalWrite(ClosePin,LOW);
  digitalWrite(StopPin,HIGH);
  
  initTimer();    
}
//////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////
GateDoor::GateDoor(SL_GATE* gate) : Service::Door(){
      CurrentPosition      = new Characteristic::CurrentPosition(50);
      TargetPosition       = new Characteristic::TargetPosition(50);
      PositionState        = new Characteristic::PositionState(2);
      ObstructionDetected  = new Characteristic::ObstructionDetected(false);

      this->gate=gate;
      NVS_init();
      LOG1("Constructing Door…\n");
   
}

void GateDoor::NVS_init(){
  LOG1("----------Reading storage----------\n");
  nvs_open("GATE",NVS_READWRITE,&gateNVS);
      
  if(!nvs_get_blob(gateNVS,"GATEDATA",NULL,&nvslen)) {                       // if found GATE data in NVS
    LOG1("----------Found GATE storage----------\n");
    nvs_get_blob(gateNVS,"GATEDATA",&GateDoorState,&nvslen); }              // retrieve data  
}

boolean GateDoor :: Calibrate(){
  uint32_t StartTime, cycleTime;
  uint32_t TimeOut = 150*1000;
  

  //phase 1
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;
  StartTime = millis();
  if (digitalRead(gate->ClSensorPin.PIN) == SENSOR_RELEASED) {
    gate->Close();
    while ( gate->ClSensorPin.changed == false || (millis() - StartTime) < TimeOut ){}
    if ( (millis() - StartTime) > TimeOut ){return(false);}
  }

  //phase 2
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;
  
  if ( digitalRead(gate->ClSensorPin.PIN) == SENSOR_CLOSED ) {
    gate->Open();
    cycleTime = millis();
    
    while ( gate->OpSensorPin.changed == false || (millis() - StartTime) < TimeOut ){}
    
    if ( (millis() - StartTime) > TimeOut ){return(false);}
    
    if ( digitalRead(gate->OpSensorPin.PIN) == SENSOR_CLOSED ){
      GateDoorState.openTime = millis() - cycleTime;
    }else {return(false);}
  } else {return(false);}

  //phase 3
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;

  if ( digitalRead(gate->OpSensorPin.PIN) == SENSOR_CLOSED ){
    gate->Close();
    cycleTime = millis();

    while ( gate->ClSensorPin.changed == false || (millis() - StartTime) < TimeOut ){}
    if ( (millis() - StartTime) > TimeOut ){return(false);}

      if ( digitalRead(gate->ClSensorPin.PIN) == SENSOR_CLOSED ){
        GateDoorState.closeTime = millis() - cycleTime;
        gate->ClSensorPin.changed = false;
        gate->OpSensorPin.changed = false;

        nvs_set_blob(gateNVS,"GATEDATA",&GateDoorState,sizeof(GateDoorState));
        nvs_commit(gateNVS);
        
        return(true);
        
      }else {return(false);}
    
  }else {return(false);}
  
  
  return(false);
}

void GateDoor::NothingTODO(){
  PositionState->setVal(DOOR_STOPPED);
  TargetPosition->setVal(CurrentPosition->getVal());  
}

boolean GateDoor :: update(){
  gate->CycleTimeBegin = millis();
  
  if (GateDoorState.openTime == 0 || GateDoorState.closeTime == 0) {
    if (!Calibrate()) {
      NothingTODO();
      return (true);
    }
  } 

  //open
  if ( TargetPosition->getNewVal() > CurrentPosition->getVal() ){
    //openning
    
    PositionState->setVal(DOOR_OPENING);
    
    gate->CurrentDoorState->  setVal(CURRENT_DOOR_STATE_OPENING);
    gate->TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);

    if (valid){
      cycleTime = GateDoorState.openTime * TargetPosition->getNewVal() / CurrentPosition->getVal();
    } else {
      cycleTime = 0;
      TargetPosition->setVal(FULLY_OPENED);
    }

    gate->Open();
    GateDoorState.Direction = 1; 
  }

  //close
  if ( TargetPosition->getNewVal() < CurrentPosition->getVal() && !ObstructionDetected->getVal()){
    //closing 
    PositionState->setVal(DOOR_CLOSING);
    gate->CurrentDoorState->  setVal(CURRENT_DOOR_STATE_CLOSING);
    
    if (TargetPosition->getNewVal() == FULLY_CLOSED) {
      gate->TargetDoorState->   setVal(TARGET_DOOR_STATE_CLOSED);
    } else {
      gate->TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);
    }

    if (valid){
      cycleTime = GateDoorState.closeTime * TargetPosition->getNewVal() / CurrentPosition->getVal();
    } else {
      cycleTime = 0;
      TargetPosition->setVal(FULLY_CLOSED);
      gate->TargetDoorState->   setVal(TARGET_DOOR_STATE_CLOSED);
    }
    
    gate->Close();
    GateDoorState.Direction = 0;
    
  } else {
    NothingTODO();
  }
  
  //
  if ( TargetPosition->getNewVal() == CurrentPosition->getVal() ){
    NothingTODO();
  }
  
  updateTime = millis();          
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
