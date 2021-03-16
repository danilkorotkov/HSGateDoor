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

  
  homeSpan.setApSSID("Sl-Gate-AP");
  homeSpan.setApPassword("");
  homeSpan.setControlPin(0);
  homeSpan.setStatusPin(2);
  homeSpan.setLogLevel(1);

  homeSpan.setSketchVersion("2.0.1");
  homeSpan.enableOTA();
  
  homeSpan.begin(Category::GarageDoorOpeners,"Sl Gate");
  
  new SpanAccessory(); 
  
    new Service::AccessoryInformation(); 
      new Characteristic::Name("Ворота"); 
      new Characteristic::Manufacturer("Danil"); 
      new Characteristic::SerialNumber("0000002"); 
      new Characteristic::Model("3 key model"); 
      new Characteristic::FirmwareRevision("0.0.2"); 
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
      new Characteristic::FirmwareRevision("0.0.2"); 
      new Characteristic::Identify();            
      
    new Service::HAPProtocolInformation();      
      new Characteristic::Version("1.1.0"); 

    new DoorLock();
}

void loop() {
  homeSpan.poll();
}
