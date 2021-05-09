////////////////////////////////////////////////////////////
//                                                        //
//    HomeSpan: A HomeKit implementation for the ESP32    //
//    ------------------------------------------------    //
//                                                        //
//        Sliding gate                                    //
//                                                        //
//                                                        //
////////////////////////////////////////////////////////////
#include "HomeSpan.h" 
#include "Lock.h" 
#include "SlGate.h"

void setup() {
  Serial.begin(115200);

  //used pins JmpPin13 LockPin16 ObS17 OpenPin18 ClosePin19 StopPin21 OpS22 ClS23 ControlPin0 StatusPin2  
  homeSpan.setApSSID("Sl-Gate-AP");
  homeSpan.setApPassword("");
  homeSpan.setControlPin(0);
  homeSpan.setStatusPin(2);
  homeSpan.setLogLevel(1);

  homeSpan.setSketchVersion("2.1.1");
  homeSpan.enableOTA();
  
  homeSpan.begin(Category::GarageDoorOpeners,"Sl Gate");
  
  new SpanAccessory(); 
  
    new Service::AccessoryInformation(); 
      new Characteristic::Name("Ворота"); 
      new Characteristic::Manufacturer("Danil"); 
      new Characteristic::SerialNumber("0000002"); 
      new Characteristic::Model("3 key model"); 
      new Characteristic::FirmwareRevision("0.0.3"); 
      new Characteristic::Identify();            
      
    new Service::HAPProtocolInformation();      
      new Characteristic::Version("1.1.0"); 
  
    new SL_GATE();

  new SpanAccessory(); 
  
    new Service::AccessoryInformation(); 
      new Characteristic::Name("Калитка"); 
      new Characteristic::Manufacturer("Danil"); 
      new Characteristic::SerialNumber("0000002"); 
      new Characteristic::Model("3 key model"); 
      new Characteristic::FirmwareRevision("0.0.3"); 
      new Characteristic::Identify();            
      
    new Service::HAPProtocolInformation();      
      new Characteristic::Version("1.1.0"); 

    new DoorLock();
}

void loop() {
  homeSpan.poll();
}
