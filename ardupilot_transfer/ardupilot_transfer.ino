#include <SoftwareSerial.h>

//FTDI cable
//black - GND
//orange (TX) - 2 (RX)
//yellow (RX) - 3 (TX)

#define LAND 0xAA

#define RX 2
#define TX 3

SoftwareSerial mySerial(RX, TX);

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
  Serial.println("Program started.");
  
}

void sendData(uint8_t *txdata, int datasize) {
  uint8_t buf;
  uint8_t CS = datasize;
  
  //preamble
  mySerial.write(0x06);
  mySerial.write(0x85);
  mySerial.flush();
  Serial.println("Sent preamble.");
   
  //size
  mySerial.write(datasize);
  mySerial.flush();
  Serial.print("Sent size: ");
  Serial.println(datasize,HEX);
    
  //data
  int j;
  for (j=0; j<datasize; j++) {
    CS ^= txdata[j]; //CS
    mySerial.write(txdata[j]);
    mySerial.flush();
    Serial.print("Sent data: ");
    Serial.println(txdata[j],HEX);
  }
  
  //checksum
  mySerial.write(CS);
  mySerial.flush();
  Serial.print("Sent CS: ");
  Serial.println(CS,HEX);
}

//try to receive data and put it in rxdata (pass in address of a pointer)
//returns num bytes of received packet, or 0 if unsuccessful
//automatically frees pointer if it was being used before
uint8_t receiveData(uint8_t **rxdata) {
  uint8_t buf;
  uint8_t CS;
  
  if (mySerial.available() == 0)
    return 0;
  
  //preamble
  buf = mySerial.read();
  Serial.println(buf,HEX);
  if (buf != 0x06) {
    return 0;
  }
  buf = mySerial.read();
  Serial.println(buf,HEX);
  if (buf != 0x85) {
    return 0;
  }
  Serial.println("Received preamble");
  
  //size
  uint8_t data_size = mySerial.read();
  Serial.print("Expecting to receive size: ");
  Serial.println(data_size);
  
  if(rxdata != 0)
    free(rxdata);
  *rxdata = 0;
  *rxdata = (uint8_t *) malloc(data_size);
  
  CS = data_size;
  
  //data
  int i;
  for (i = 0; i < data_size; i++) {
    *((*rxdata) + i) = mySerial.read();
    Serial.print("Data received: ");
    Serial.println(*((*rxdata) + i), HEX);
    CS ^= *((*rxdata) + i);
  }
  Serial.print("CS calculated: ");
  Serial.println(CS,HEX);

  //CS
  buf = mySerial.read();
  Serial.print("CS received: ");
  Serial.println(buf,HEX);
  if (buf != CS) {
    Serial.println("Checksum did not match.");
    return 0;
  }
  else {
    Serial.println("Received data successfully.");
    return data_size;
  }
  
}

void loop() {
  while(1);
}
