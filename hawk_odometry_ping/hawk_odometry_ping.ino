#include <EEPROM.h>
#include "APM_RC.h"
#include <Wire.h>
#include "Arduino.h"

#define CalibrationMaskAddress  0x00
#define IMAGESIZE 144
#define MOTOR_STOP 1000
#define RXpin A12
#define of_scale 100
#define MAX_D 10

short cal[IMAGESIZE];
// reference "from" which optical flow is measured
char refimage[IMAGESIZE];
// the image "to" which optical flow displacements are measured.
char inpimage[IMAGESIZE];
char charbuf[50];
int framecounter=0;
int  accx,accy;  // accumulated optical flows in X and Y directions
int  maxdelta;  // "delta" amount that triggers updating the reference frame
char  firstframe;  
char outputmode;
//top, arm1, arm2, arm3, arm4, bottom
int dist[6] = {0, 0, 0, 0, 0, 0};
int RCin[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int RCold[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int RCout[8]= {0, 0, 0, 0, 0, 0, 0, 0};
int RCtrans[8] = {0, 0, 0, 0, 0, 0, 0, 0};
int pings[6]= {A1, A3, A5, A7, A9, A11};
int i;

int ofx,ofy; // current measured displacement- integer versions
float fofx,fofy; // current measured displacement- float versions
int odox,odoy; // odometry x and y values
int delta; // square of total displacement- integer version
float fdelta; // square of total displacement- float version

void setup() {
  int i;
  APM_RC.Init(); 
  /*  
  APM_RC.OutputCh(0,MOTOR_STOP);
  APM_RC.OutputCh(1,MOTOR_STOP);
  APM_RC.OutputCh(2,MOTOR_STOP);
  APM_RC.OutputCh(4,MOTOR_STOP);
  */
  pinMode(RXpin, OUTPUT);
  digitalWrite(RXpin, HIGH);
  delay(20);
  pinMode(RXpin, INPUT); //make it HI Z
  Serial.begin(115200);
  Tam_InitArduino(1);
  for (i=0; i<IMAGESIZE; ++i) {
    refimage[i] = inpimage[i] = 0;
    cal[i] = EEPROM.read(CalibrationMaskAddress+2*i);
    cal[i] += short(EEPROM.read(CalibrationMaskAddress+2*i+1))<<8;
  }
  //dumpCommandList();
  firstframe=1; 
  accx=accy=0; // reset accumulations
  maxdelta=500; // square of displacement that triggers updating frame
  outputmode = 0; // set initial output mode to quiet
}

void loop() {
  short i;
  /*
  APM_RC.OutputCh(0,MOTOR_STOP);
  APM_RC.OutputCh(1,MOTOR_STOP);
  APM_RC.OutputCh(2,MOTOR_STOP);
  APM_RC.OutputCh(4,MOTOR_STOP);
  */
  framecounter++;
  read_pings();
  Tam2LoadSubimageCharCalibrated(12,2,12,2,inpimage,cal); //subtract noise from last image
  // If odometer is being reset (or at startup) then perform the following sequence
  if (firstframe) {
    for (i=0; i<IMAGESIZE; ++i)
      refimage[i] = inpimage[i];
    firstframe=0;
    accx=0; accy=0;
  }  
  // Compute optical flow displacement between reference and input images
  IIA2(refimage, inpimage, &fofx, &fofy);
  ofx=fofx; ofy=fofy;
  fdelta = fofx*fofx + fofy*fofy;
  if (fdelta>maxdelta) { // Threshold exceeded: update reference image
    for (i=0; i<IMAGESIZE; ++i)
      refimage[i] = inpimage[i];
    odox = accx + ofx;
    odoy = accy + ofy;
    accx += ofx;
    accy += ofy;
    ofx=ofy=0;
  } else { // Threshold not exceeded- just compute odometry
    odox = accx + ofx;
    odoy = accy + ofy;
  }    
  //Serial.print("x:");
  //Serial.print(odox);
  //Serial.print(" y:");
  //Serial.print(odoy);
  //Serial.print(" ");
  if (outputmode==1) {
    Serial.print(odox);
    Serial.print(" ");
    Serial.println(odoy);
  }
  serial_commands();  
  for(i = 0; i < 8; i++) {
    RCin[i] = channel_filter(i, APM_RC.InputCh(i), RCold[i]);
    RCtrans[i] = RCin[i]; 
    /*
    if(i == 4) {
      int temp = RCtrans[4];
      RCtrans[4] = RCtrans[3];
      RCtrans[3] = temp;  
    } 
    */
    RCold[i] = RCtrans[i];
  }
  for(i = 0; i < 8; i++) {
    Serial.print(RCtrans[i]);
    Serial.print(", "); 
  }
  
  Serial.println();
  
  for(i = 0; i < 8; i++) {
    APM_RC.OutputCh(i, RCtrans[i]);
  }

  
} 

void read_pings() {
  int i;
  for(i = 0; i < 6; i++) {
     dist[i] = analogRead(pings[i]); 
     //Serial.print(dist[i]);
     //Serial.print(", ");
  }
  //Serial.println("");
}

void dumpCommandList() {
  Serial.println("a - ASCII image");
  Serial.println("c - Calibrate");
  Serial.println("f - print frame counter");
  Serial.println("m - MATLAB image");
  Serial.println("r - read calibration from EEPROM");
  Serial.println("s - store calibration to EEPROM");  
  Serial.println("0 (zero) - quiet output");
  Serial.println("1 (one) - output every frame");
  Serial.println("o (letter o) - print odometry info");
}



void IIA2(char *X1, char *X2, float *ofx, float *ofy) {
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;        
  char *f0 = X1 + 12 + 1; // "12"
  char *f1 = X1 + 12 + 2; // "12"
  char *f2 = X1 + 12; // "12"
  char *f3 = X1 + 2*12 + 1; // "12"
  char *f4 = X1 + 1;
  char *fz = X2 + 12 + 1; // "12"
  // loop through
  for (char r=1; r!=12-1; ++r) { // "12"
    for (char c=1; c!=12-1; ++c) { // "12"
      // compute differentials, then increment pointers (post increment)
      F2F1 = (*(f2++) - *(f1++));
      F4F3 = (*(f4++) - *(f3++));
      FCF0 = (*(fz++) - *(f0++));
      // update summations
      A11 += (F2F1 * F2F1);
      A12 += (F4F3 * F2F1);
      A22 += (F4F3 * F4F3);
      b1  += (FCF0 * F2F1);                   
      b2  += (FCF0 * F4F3);                                   
    }
    f0+=2;
    fz+=2;
    f1+=2;
    f2+=2;
    f3+=2;
    f4+=2;
  }       
  int64_t detA = ( (int64_t)(A11)*A22 - (int64_t)(A12)*A12 );
  int64_t XS = ( (int64_t)(b1)*A22 - (int64_t)(b2)*A12 ) * of_scale / detA;
  int64_t YS = ( (int64_t)(b2)*A11 - (int64_t)(b1)*A12 ) * of_scale / detA;
  *ofx = XS;
  *ofy = YS;
}



int channel_filter(int ch, int val, int old_val) {  
  //return val;  
  switch(ch) {
    case 0: 
      val = ((val - 1473)*275./418. + 1500); 
      if(val - RCold[0] > MAX_D) {
        val = RCold[0] + MAX_D;    
      }
      else if(RCold[0] - val > MAX_D) {
        val = RCold[0] - MAX_D;  
      }
      break;
    case 1: 
      val = ((val - 1470)*200./414. + 1500); 
      if(val - RCold[1] > MAX_D) {
        val = RCold[1] + MAX_D;    
      }
      else if(RCold[1] - val > MAX_D) {
        val = RCold[1] - MAX_D;  
      }
      break;
    case 2:
      /*  
      val = ((val - 1470)*1./2. + 1500);     
      if(val - RCold[2] > MAX_D) {
        val = RCold[2] + MAX_D;    
      }
      else if(RCold[2] - val > MAX_D) {
        val = RCold[2] - MAX_D;  
      }*/
      val = val;
      break;
    case 3: 
      val = ((val - 1468)*1./2. + 1500);   
      if(val - RCold[3] > MAX_D) {
        val = RCold[3] + MAX_D;    
      }
      else if(RCold[3] - val > MAX_D) {
        val = RCold[3] - MAX_D;  
      }
      break;
    case 4: case 5: case 6: case 7:
      val = val;                             
      break;
   default: Serial.println("Unhandled channel");
  }
  return val;
}

void serial_commands() {
  if (Serial.available()>0) { // if a command was sent, enter this block
    char command = Serial.read(); // pull command from input buffer
    switch (command) {
      // Command c: grab calibration mask- best performed when Tam chip is
      // exposed to uniform background e.g. with piece of paper resting
      // right on top of lens or similar.
      case 'c':
        Tam2LoadSubimageShort(12,2,12,2,cal); // Change 12 for different image sizes
        Serial.println("Calibration pattern grabbed from image sensor");
        dumpShortImageAsciiSlow(cal,12,12,0,0); // Change 12 for different image sizes
        Serial.println("Done.");
        break;
       
      // Command a: Dump ASCII rendering of 12x12 image to monitor
      case 'a':
        Tam2LoadSubimageCharCalibrated(12,2,12,2,inpimage,cal); // Change 12 for different image sizes
        //Serial.println("MATLAB dump of image, after calibration");
        //dumpCharImageMatlabSlow(inpimage,12,12);
        Serial.println("ASCII dump of image, after calibration");
        dumpCharImageAsciiSlow(inpimage,12,12,0,0); // Change 12 for different image sizes
        Serial.println("Done.");
        break;

      // Command f: Dump frame counter to the serial monitor
      case 'f':
        Serial.print("frame counter = ");
        Serial.println(framecounter);
        break;

      // Command m; Dump 12x12 image to serial monitor in MATLAB format
      case 'm':
        Serial.println("%-----------------------------");
        dumpCharImageMatlabSlow(inpimage,12,12); // Change 12 for different image sizes
        Serial.println("%-----------------------------");
        break;

      // Command r: Read fixed patt noise mask from EEPROM
      case 'r':
        Serial.println("Reading calibration mask from EEPROM...");
        for (i=0; i<IMAGESIZE; ++i) {
          cal[i] = EEPROM.read(CalibrationMaskAddress+2*i);
          cal[i] += short(EEPROM.read(CalibrationMaskAddress+2*i+1))<<8;
        }
        Serial.println("Done");
        break;
        
      // Command s: Save fixed patt noise mask to EEPROM. Note that
      // this may not be the fastest way, and may put excessive
      // strain on the EEPROM, so don't do this more than necessary
      case 's':
        Serial.println("Writing calibration mask to EEPROM...");
        for (i=0; i<IMAGESIZE; ++i) {
          EEPROM.write(CalibrationMaskAddress+2*i,(cal[i])&0xFF);
          EEPROM.write(CalibrationMaskAddress+2*i+1,(cal[i]>>8)&0xFF);
        }
        Serial.println("Done");
        break;

      // Command 1 (one): Set output mode to 1 for odometry dumped every frame
      case '1': // 1 = one
        outputmode=1;
        Serial.println("Output mode is 1");
        break;
        
      // Command 0 (zero): Set output mode to 0 to stop dumping odometry info
      case '0': // 0 = zero
        outputmode=0;
        Serial.println("Output mode is 0");
        break;

      // Command o (lower case): Send detailed odometry info to serial monitor
      case 'o': // o = letter o lowercase
        sprintf(charbuf,"X: accx + ofx = odox  %d + %d = %d",accx,ofx,odox);
        Serial.println(charbuf);
        sprintf(charbuf,"Y: accy + ofy = odoy  %d + %d = %d",accy,ofy,odoy);
        Serial.println(charbuf);
        break;
        
      // Command z: Reset the accumulator- essentially by setting firstframe=1
      // we reset the accumulators to zero and the reference image to the 
      // current next frame, just like at power up.
      case 'z': // reset the accumulator
        Serial.println("Resetting odometry value");
        firstframe=1;
        break;
        
      default:
        Serial.println("Command not recognized. Choose from list below.");
        dumpCommandList();
        
    }
  }
}


