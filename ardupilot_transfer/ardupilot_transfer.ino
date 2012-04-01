#include <SoftwareSerial.h>

//FTDI cable
//black - GND
//orange (TX) - 2 (RX)
//yellow (RX) - 3 (TX)

#define MAX_TRIES 3
#define ACK 0xFE
#define NACK 0xFD
#define LAND 0xAA

#define RX 2
#define TX 3

SoftwareSerial mySerial(RX, TX);

uint8_t rxdata;
uint8_t txdata[25] = "Hello, World";

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("I'm ready");
  
  //int datasize = sizeof(txdata);
  
  //sendData(txdata, datasize);
}

void sendData(uint8_t *txdata, int datasize) {
  uint8_t buf;
  uint8_t CS = datasize;
  
  int i;
  for (i=0; i<MAX_TRIES; i++) {
    //preamble
    mySerial.write(0x06);
    mySerial.write(0x85);
    Serial.println("Sent preamble.");
    
    //size
    mySerial.write(datasize);
    Serial.print("Sent size: ");
    Serial.println(datasize,HEX);
    
    //data
    int j;
    for (j=0; j<datasize; j++) {
      CS ^= txdata[j]; //CS
      mySerial.write(txdata[j]);
      Serial.print("Sent: ");
      Serial.println(txdata[j],HEX);
    }
 
    mySerial.write(CS);
    Serial.print("Sent CS: ");
    Serial.println(CS,HEX);
    CS = datasize;
    
    //handshake
    uint8_t response;
    delay(100);
    response = mySerial.read();

    if (response == ACK) {
      Serial.print("Received ACK: ");
      Serial.println(response,HEX);
      Serial.println("Sent data to Kevin successfully");
      break;
    }
    else if (response == NACK) {
      //don't increment, and try again
      i--;
      Serial.print("Received NACK: ");
      Serial.println(response,HEX);
    }
    else if (response == LAND) {
      Serial.println("Telling ArduPilot to land quadrotor");
    }
    else {
      //proceed with the loop
    }
  }
}

uint8_t receiveData() {
  while(mySerial.available() == 0);
  Serial.println("Passed blocking");
  
  uint8_t buf;
  //TODO: malloc
  uint8_t CS = 1;
  rxdata = 0;
  
  //preamble
  buf = mySerial.read();
  Serial.println(buf,HEX);
  if (buf != 0x06) {
    buf = NACK;
    Serial.print("About to send: ");
    Serial.println(buf);
    mySerial.write(buf);
    Serial.println("Sent NACK");
    return 0;
  }
  buf = mySerial.read();
  Serial.println(buf,HEX);
  if (buf != 0x85) {
    buf = NACK;
    Serial.print("About to send: ");
    Serial.println(buf);
    mySerial.write(buf);
    Serial.println("Sent NACK");
    return 0;
  }
  Serial.println("Received preamble");
  
  //size
  uint8_t data_size = mySerial.read();
  Serial.println(data_size);
  
  //data
  //received data = (uint8_t)malloc(data_size*sizeof(uint8_t))
  rxdata = mySerial.read();
  Serial.println(rxdata,HEX);
  CS ^= rxdata;
  Serial.println(CS,HEX);

  //CS
  buf = mySerial.read();
  Serial.print("CS received: ");
  Serial.println(buf,HEX);
  if (buf != CS) {
    buf = NACK;
    Serial.print("About to send: ");
    Serial.println(buf);
    mySerial.write(buf);
    Serial.println("Checksum did not match");
    return 0;
  }
  else {
    buf = ACK;
    Serial.print("About to send: ");
    Serial.println(buf);
    mySerial.write(buf);
    Serial.println("Received data from Kevin");
    return 1;
  }
  
}

void loop() {
  //txdata = (uint8_t)malloc(sizeof(uint8_t));
  int datasize = sizeof(txdata);
  
  sendData(txdata, datasize);
  
  //receiveData();
  
  //delay(10); 
  
}
