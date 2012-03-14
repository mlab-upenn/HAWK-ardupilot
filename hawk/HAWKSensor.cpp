#include "EasyTransfer.h"
#include "HAWKSensor.h"

//
// Constructor
//

HAWKSensor::HAWKSensor(uint8_t s, String n) : 
  _data = (uint8_t*) malloc(s * sizeof(uint8_t));
{
  id = n;
  size = s;
}

//
// Destructor
//

HAWKSensor::~HAWKSensor() : 
{
  free(data);
}


void HAWKSensor::update(uint8_t* d) : 
{
  if(d)
    _data = d;
}

