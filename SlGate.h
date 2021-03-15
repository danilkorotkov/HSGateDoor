#ifndef SLGATE_H
#define SLGATE_H
#include "HomeSpan.h" 

#include <nvs.h>
#include <nvs_flash.h>

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

#define FULLY_OPENED 100
#define FULLY_CLOSED 0

struct SL_GATE;
struct GateDoor;
struct Sensor;
struct CustomOpen;

void IRAM_ATTR isr(void*);
void IRAM_ATTR onTimer();

struct Sensor {
    uint8_t PIN;
    bool changed;
    int stableState;
};

struct CustomOpen {
    uint32_t updateTime = 0;
    bool fromZeroPos = false;
    bool Direction = 0; // 0 - opening 1 - closing
    uint32_t openTime = 0;
    uint32_t closeTime = 0;
    uint8_t CurrentPosition = 50; 
};

struct SL_GATE : Service::GarageDoorOpener {         // First we create a derived class from the HomeSpan 

  int OpenPin     = 18;                                       // this variable stores the pin number defined 
  int ClosePin    = 19;
  int StopPin     = 21;
  struct Sensor OpSensorPin = {22, false, SENSOR_RELEASED};
  struct Sensor ClSensorPin = {23, false, SENSOR_RELEASED};
  struct Sensor ObSensorPin = {17, false, SENSOR_RELEASED};
  
  uint32_t CycleTimeout = 60000; //60s
  uint32_t CycleTimeBegin;
  uint32_t PortPollTimeout = 200;
  uint32_t ClPortPollBegin, OpPortPollBegin, ObPortPollBegin = 0;
  
  SpanCharacteristic *CurrentDoorState;              
  SpanCharacteristic *TargetDoorState;
  SpanCharacteristic *ObstructionDetected;
  SpanCharacteristic *Name;
  GateDoor *GatePosition;  
   
  SL_GATE();
  void PollCurrentState();
  void initTimer();
  boolean update();                                   
  void loop();
  void FullyOpened();
  void FullyClosed();
};

struct GateDoor : Service::Door{
  size_t nvslen;             // not used but required to read blobs from NVS
  static nvs_handle gateNVS;
    
  struct CustomOpen GateDoorState;
  
  SpanCharacteristic *CurrentPosition;
  SpanCharacteristic *TargetPosition;
  SpanCharacteristic *PositionState;
  SpanCharacteristic *ObstructionDetected;
  SpanCharacteristic *HoldPosition;
  SL_GATE *gate;
   
    /*  PositionState
        0 ”Going to the minimum value specified in metadata”
        1 ”Going to the maximum value specified in metadata”
        2 ”Stopped”
        3-255 ”Reserved” */
    
  uint32_t CycleTimeBegin = millis();
  uint32_t PollTimeout = 10000;
  GateDoor(SL_GATE* gate);
  void NVS_init();    
  boolean update();
  void loop();   
};

////////////////

#endif //SLGATE_H
