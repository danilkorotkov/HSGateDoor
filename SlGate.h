#ifndef SLGATE_H
#define SLGATE_H
#include "HomeSpan.h" 

#include <nvs.h>
#include <nvs_flash.h>

// Possible values for characteristic garage
#define CURRENT_DOOR_STATE_OPEN    0
#define CURRENT_DOOR_STATE_CLOSED  1
#define CURRENT_DOOR_STATE_OPENING 2
#define CURRENT_DOOR_STATE_CLOSING 3
#define CURRENT_DOOR_STATE_STOPPED 4
#define CURRENT_DOOR_STATE_UNKNOWN 255

#define TARGET_DOOR_STATE_OPEN    0
#define TARGET_DOOR_STATE_CLOSED  1
#define TARGET_DOOR_STATE_UNKNOWN 255
//position sensor
//#define SENSOR_CLOSED   0
//#define SENSOR_RELEASED 1
//door
#define FULLY_OPENED 100
#define FULLY_CLOSED 0

#define DOOR_CLOSING 0
#define DOOR_OPENING 1
#define DOOR_STOPPED 2

struct SL_GATE;
struct GateDoor;
struct Sensor;
struct CustomOpen;

void IRAM_ATTR isr(void*);
void IRAM_ATTR onTimer();

//int SENSOR_CLOSED   = 0;
//int SENSOR_RELEASED = 1;

struct Sensor {
    uint8_t PIN;
    bool changed;
    int stableState;
};

struct CustomOpen {
    //uint32_t  updateTime  = 0;
    //bool      fromZeroPos = false;
    //bool      Direction   = 0; // 1 - opening 0 - closing
    uint32_t  openTime    = 0;
    uint32_t  closeTime   = 0;
};

struct SL_GATE : Service::GarageDoorOpener {         // First we create a derived class from the HomeSpan 

  struct Sensor OpSensorPin = {22, false, 1};
  struct Sensor ClSensorPin = {23, false, 1};
  struct Sensor ObSensorPin = {17, false, 1};
  
  uint32_t CycleTimeout = 60000; //60s
  uint32_t CycleTimeBegin;
  uint32_t PortPollTimeout = 500;
  uint32_t PortPollBegin, ObPortPollBegin = 0;
  
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
  void FullyOpenExtern();
  void FullyCloseExtern();
  void Open();
  void Close();
  void Stop();
};

struct GateDoor : Service::Door{
  size_t nvslen;             // not used but required to read blobs from NVS
  static nvs_handle gateNVS;
  bool      valid       = false; // положение неизвестно   
  struct CustomOpen GateDoorState;
  uint32_t  cycleTime   = 0;
  uint32_t  updateTime  = 0;
  uint32_t  delta       = 0;
  
  SpanCharacteristic *CurrentPosition;
  SpanCharacteristic *TargetPosition;
  SpanCharacteristic *PositionState;
  SpanCharacteristic *ObstructionDetected;

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
  boolean Calibrate();
  void NothingTODO();   
};

////////////////

#endif //SLGATE_H
