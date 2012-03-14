#ifndef HAWKTransfer_h
#define HAWKTransfer_h

#include "HAWKSensor.h"

#define MAX_SENSORS  10

struct transfer_packet{
  uint8_t num_sensors; //number of sensors
  HAWKSensor sensors; //pointer to array of sensor data objects
};

//initialize a serial transfer object
//add their HAWKSensors to the list of things that should be sent
//from then on, they call update on their sensors
//and then send using serial transfer object

class HAWKTransfer {
  public:  
    void add(HAWKSensor);
    void send();
  private:
    transfer_packet _tp; //user needs to add their sensors to the tp queue  
    EasyTransfer _ET;
    uint8_t _counter; //keeps track of how many sensors have been added
}

#endif
