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
//#define SENSOR_CLOSED   0
//#define SENSOR_RELEASED 1

#define P_SENSOR_CLOSED   0
#define P_SENSOR_RELEASED 1

#include "SlGate.h"

////////////////////////////
int ButtonArray[3];
bool isTimered = false;
hw_timer_t * ButtonTimer = NULL;
portMUX_TYPE DoortimerMux = portMUX_INITIALIZER_UNLOCKED;

int OpenPin     = 18;                                       
int ClosePin    = 19;
int StopPin     = 21;
int JmpPin      = 13; // high на в промежуточном положении все High и  low - в крайних 

int SENSOR_CLOSED   = 0;
int SENSOR_RELEASED = 1;

void IRAM_ATTR isr(void* arg) {
    Sensor* s = static_cast<Sensor*>(arg);
    s->changed = true;
};

void IRAM_ATTR onTimer(){
  portENTER_CRITICAL_ISR(&DoortimerMux);
  isTimered = true;
  for (int i=0; i<3; i++) {
    digitalWrite(ButtonArray[i],LOW);
   } 
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
  CurrentDoorState  = new Characteristic::CurrentDoorState(CURRENT_DOOR_STATE_OPEN);
  TargetDoorState =   new Characteristic::TargetDoorState(TARGET_DOOR_STATE_OPEN);
  ObstructionDetected=new Characteristic::ObstructionDetected();
  Name=new Characteristic::Name("Gate");
 
                         
  pinMode(OpenPin,OUTPUT); 
  digitalWrite(OpenPin,LOW);
  ButtonArray[0] = OpenPin;
                                              
  pinMode(ClosePin,OUTPUT);
  digitalWrite(ClosePin,LOW);
  ButtonArray[1] = ClosePin;  
                      
  pinMode(StopPin,OUTPUT);
  digitalWrite(StopPin,LOW);
  ButtonArray[2] = StopPin;

  pinMode(JmpPin, INPUT_PULLUP);
  SENSOR_CLOSED    ^= digitalRead(JmpPin);
  SENSOR_RELEASED  ^= digitalRead(JmpPin);// инвертирование концевиков
  if (digitalRead(JmpPin)) {LOG1("reverse sensors \n");}
    
  pinMode(ClSensorPin.PIN, INPUT_PULLUP);
  pinMode(OpSensorPin.PIN, INPUT_PULLUP);
  pinMode(ObSensorPin.PIN, INPUT_PULLUP);
  attachInterruptArg(ClSensorPin.PIN, isr, &ClSensorPin, CHANGE);
  attachInterruptArg(OpSensorPin.PIN, isr, &OpSensorPin, CHANGE);
  attachInterruptArg(ObSensorPin.PIN, isr, &ObSensorPin, CHANGE);
  
  GatePosition = new GateDoor(this);

  ClSensorPin.stableState = digitalRead(ClSensorPin.PIN);
  OpSensorPin.stableState = digitalRead(OpSensorPin.PIN);
  
  PollCurrentState();

  LOG1("Constructing Gate successful!\n");
  //LOG1(WiFi.localIP());  
} // end constructor

void SL_GATE::PollCurrentState(){
    LOG1("polling...\n");
  
    if (digitalRead(ClSensorPin.PIN)      == SENSOR_CLOSED)   {if (CurrentDoorState-> getVal() !=CURRENT_DOOR_STATE_CLOSED)        
                                                              {LOG1("polling CLOSED\n");
                                                              ClSensorPin.stableState = SENSOR_CLOSED;
                                                              FullyClosed();
                                                              }}
    
    else if (digitalRead(OpSensorPin.PIN) == SENSOR_CLOSED)   {if (CurrentDoorState->  getVal() != CURRENT_DOOR_STATE_OPEN)   
                                                              {LOG1("polling OPEN\n");
                                                              OpSensorPin.stableState = SENSOR_CLOSED;
                                                              FullyOpened();
                                                              }}
    
    else if                                                   (CurrentDoorState-> getVal() != CURRENT_DOOR_STATE_OPEN || GatePosition->PositionState->getVal() != DOOR_STOPPED)   
                                                              {LOG1("polling all released\n");
                                                              CurrentDoorState-> setVal(CURRENT_DOOR_STATE_OPEN);   
                                                              TargetDoorState->   setVal(TARGET_DOOR_STATE_OPEN);
                                                              OpSensorPin.stableState = SENSOR_RELEASED; 
                                                              ClSensorPin.stableState = SENSOR_RELEASED;

                                                              GatePosition->TargetPosition->setVal(50);
                                                              GatePosition->CurrentPosition->setVal(50);
                                                              GatePosition->PositionState->setVal(DOOR_STOPPED);
                                                              GatePosition->valid = false;
                                                              }
    
    if (digitalRead(ObSensorPin.PIN)    == P_SENSOR_CLOSED && !ObstructionDetected->getVal())        
                                                              {ObstructionDetected->setVal(true);
                                                              ObSensorPin.stableState = P_SENSOR_CLOSED;

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
      }
      
      // если сработал концевик, фиксируем время срабатывания, и игнорируем его изменения-дребезг после обработки события
      if (ClSensorPin.changed && (millis() - PortPollBegin)>PortPollTimeout) {
        ClSensorPin.changed = false;
        PortPollBegin = millis();
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
      
      } else if ( ((millis() - PortPollBegin)>PortPollTimeout) && (ClSensorPin.stableState == SENSOR_CLOSED) && (CurrentDoorState->getVal() != CURRENT_DOOR_STATE_CLOSED) ){
                                                              LOG1("closed by timeout\n");
                                                              FullyClosed();}
      
      if (OpSensorPin.changed && (millis() - PortPollBegin)>PortPollTimeout) {
        OpSensorPin.changed = false;
        PortPollBegin = millis();
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
      
      } else if ( ((millis() - PortPollBegin)>PortPollTimeout) && OpSensorPin.stableState == SENSOR_CLOSED && CurrentDoorState->getVal() != CURRENT_DOOR_STATE_OPEN ){
                                                              LOG1("opened by timeout\n");
                                                              FullyOpened();}

      if (ObSensorPin.changed && (millis() - ObPortPollBegin)>PortPollTimeout) {
        ObSensorPin.changed = false;
        ObPortPollBegin = millis();
        CycleTimeBegin = millis();
        if (digitalRead(ObSensorPin.PIN) == P_SENSOR_CLOSED && ObSensorPin.stableState == P_SENSOR_RELEASED)    
                                                              {LOG1("----------Optocoupler.SENSOR_CLOSED----------\n");
                                                               ObstructionDetected->setVal(true);
                                                               ObSensorPin.stableState = P_SENSOR_CLOSED;
                                                               
                                                               GatePosition->ObstructionDetected->setVal(true);
                                                               }
        
        if (digitalRead(ObSensorPin.PIN) == P_SENSOR_RELEASED && ObSensorPin.stableState == P_SENSOR_CLOSED)                                                        
                                                              {LOG1("----------Optocoupler.SENSOR_RELEASED----------\n");
                                                               ObstructionDetected->setVal(false);
                                                               ObSensorPin.stableState = P_SENSOR_RELEASED;

                                                               GatePosition->ObstructionDetected->setVal(false);
                                                               }
      
      } else if ( ((millis() - ObPortPollBegin)>PortPollTimeout) && ObSensorPin.stableState == P_SENSOR_CLOSED && !ObstructionDetected->getVal() )
                                                              {ObstructionDetected->setVal(true);
                                                              GatePosition->ObstructionDetected->setVal(true);
                                                              }
        
        else if ( ((millis() - ObPortPollBegin)>PortPollTimeout) && ObSensorPin.stableState == P_SENSOR_RELEASED && ObstructionDetected->getVal() )
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
  //GatePosition->GateDoorState.Direction = 1;
  GatePosition->updateTime = millis();
  GatePosition->cycleTime = 0;
      
  /*if (ClSensorPin.stableState == SENSOR_CLOSED) {GatePosition->GateDoorState.fromZeroPos = true;
  }else {GatePosition->GateDoorState.fromZeroPos = false;}*/
}

void SL_GATE::FullyCloseExtern(){
  CurrentDoorState->setVal(CURRENT_DOOR_STATE_CLOSING);

  GatePosition->PositionState ->setVal(DOOR_CLOSING);
  GatePosition->TargetPosition->setVal(FULLY_CLOSED);    
  //GatePosition->GateDoorState.Direction = 0;
  GatePosition->updateTime = millis();
  GatePosition->cycleTime = 0;
          
  /*if (OpSensorPin.stableState == SENSOR_CLOSED) {GatePosition->GateDoorState.fromZeroPos = true;
  }else {GatePosition->GateDoorState.fromZeroPos = false;} */
}

void SL_GATE::FullyClosed(){
  CurrentDoorState-> setVal(CURRENT_DOOR_STATE_CLOSED); 
  TargetDoorState->  setVal(TARGET_DOOR_STATE_CLOSED);
  
  GatePosition->CurrentPosition->setVal(FULLY_CLOSED);
  GatePosition->TargetPosition->setVal(FULLY_CLOSED);
  GatePosition->PositionState->setVal(DOOR_STOPPED);
  GatePosition->valid = true;    
}

void SL_GATE::Open(){
  LOG1("call open routine\n");
  digitalWrite(ClosePin,LOW);
  digitalWrite(StopPin,LOW);
  digitalWrite(OpenPin,HIGH);
  
  initTimer();  
}

void SL_GATE::Close(){
  LOG1("call close routine\n");
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
      PositionState        = new Characteristic::PositionState(DOOR_STOPPED);
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
  uint32_t TimeOut = 150000;
  
  LOG1("calibration started\n");
  LOG1("phase 1\n");
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;
  StartTime = millis();
  if (digitalRead(gate->ClSensorPin.PIN) == SENSOR_RELEASED) {
    gate->Close();

    while ( (gate->ClSensorPin.changed == false) && ((millis() - StartTime) < TimeOut) ){
      LOG1("waiting for close\n");
      LOG1("millis() - StartTime ");LOG1(millis() - StartTime); LOG1("ms\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    if ( (millis() - StartTime) >= TimeOut ){LOG1("calibration failed\n");return(false);}
  }

  LOG1("phase 2\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;
  
  if ( digitalRead(gate->ClSensorPin.PIN) == SENSOR_CLOSED ) {
    gate->Open();
    cycleTime = millis();
    
    while ( gate->OpSensorPin.changed == false && (millis() - StartTime) < TimeOut ){
      LOG1("waiting for open\n");
      LOG1("millis() - StartTime ");LOG1(millis() - StartTime); LOG1("ms\n");
      vTaskDelay(1000 / portTICK_PERIOD_MS);
      }
    
    if ( (millis() - StartTime) > TimeOut ){return(false);}
    
    if ( digitalRead(gate->OpSensorPin.PIN) == SENSOR_CLOSED ){
      GateDoorState.openTime = millis() - cycleTime;
      LOG1("GateDoorState.openTime = ");LOG1(GateDoorState.openTime); LOG1("ms\n");
    }else {return(false);}
  } else {return(false);}

  LOG1("phase 3\n");
  vTaskDelay(1000 / portTICK_PERIOD_MS);
  gate->ClSensorPin.changed = false;
  gate->OpSensorPin.changed = false;

  if ( digitalRead(gate->OpSensorPin.PIN) == SENSOR_CLOSED ){
    gate->Close();
    cycleTime = millis();

    while ( gate->ClSensorPin.changed == false || (millis() - StartTime) < TimeOut ){}
    if ( (millis() - StartTime) > TimeOut ){return(false);}

      if ( digitalRead(gate->ClSensorPin.PIN) == SENSOR_CLOSED ){
        GateDoorState.closeTime = millis() - cycleTime;
        LOG1("GateDoorState.closeTime = ");LOG1(GateDoorState.closeTime); LOG1("ms\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
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
  cycleTime = 0;  
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
    //GateDoorState.Direction = 1; 
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
    //GateDoorState.Direction = 0;
    
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
  uint32_t tempTime;
  
  tempTime = millis() - updateTime;
  
  if (cycleTime != 0){
    if (gate->ClSensorPin.changed || gate->OpSensorPin.changed){
      cycleTime = 0;
      gate->ClSensorPin.changed = false;
      gate->OpSensorPin.changed = false;
      gate->PollCurrentState();
    }
  }
  
  if ( cycleTime != 0 && tempTime >= cycleTime  ){
    gate->Stop();
    PositionState->setVal(DOOR_STOPPED);
    CurrentPosition->setVal(TargetPosition->getVal());
    gate->CycleTimeBegin = millis();
    cycleTime = 0; 
    if (gate->CurrentDoorState->getVal()>0){
      gate->CurrentDoorState->  setVal(CURRENT_DOOR_STATE_CLOSED);        
    } else {gate->CurrentDoorState->  setVal(CURRENT_DOOR_STATE_OPEN);}
  }
} //loop   

nvs_handle GateDoor::gateNVS;
