#ifndef HAWKSensor_h
#define HAWKSensor_h

#include "HAWKTransfer.h"
#include "QueueList.h"

class HAWKSensor { //holds data for a sensor
  public:
    void update(uint8_t*);
    uint8_t size; //size of sensor data blob
    String id; //name for sensor
  private:
    unint8_t* _data;
}



#endif
