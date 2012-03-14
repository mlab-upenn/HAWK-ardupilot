#include "HAWKTransfer.h"
#include "HAWKSensor.h"

//
// Constructor
//

HAWKTransfer::HAWKTransfer(uint8_t n, HardwareSerial *theSerial) : 
  _tp.num_sensors = n;
  _tp.sensors = (HAWKSensor*) malloc(num_sensors * sizeof(HAWKSensor));
  _ET.begin(details(_tp), theSerial);
  _counter = 0;
{
}

//
// Destructor
//

HAWKTransfer::~HAWKTransfer() : 
{
  free(_tp.sensors);
}


void HAWKTransfer::add(HAWKSensor hs) : 
{
  if(hs) {
      _tp.sensors[counter] = hs;
  }
}

class HAWKTransfer {
  public:  
    void add(HAWKSensor);
    void send();
    
  private:
    transfer_packet _tp; //user needs to add their sensors to the tp queue  
}
