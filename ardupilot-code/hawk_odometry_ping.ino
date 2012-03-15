/*
Tam2_OdometryExample -- Arduino sketch demonstrating acquisition of a
12x12 subimage from a 16x16 Tam2 sensor, and performing odometry using
Srinivasan's image interpolation algorithm.

===============================================================================
Copyright (c) 2011, Centeye, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of Centeye, Inc. nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL CENTEYE, INC. BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
===============================================================================
*/
/*
This Arduino sketch demonstrates simple odometry using a Tam2 16x16 vision chip
using an Arduino board and a Centeye Rox1_v1 ArduEye interface board.
Currently this sketch has been implemented with just an Arduino Duemilanove
board, but it should be easy to adapt to other Arduino boards.

The odometry algorithm can be summarized as follows:

Step 1: Set accumulation=0
Step 2: Grab input_image
Step 3: Set ref_image = input_image
Step 4: Grab input_image
Step 5: Compute OF between ref_image and input_image
Step 6: if OF > threshold then
            set accumulation = accumulation + OF
            set odometry = accumulation
            set ref_image = input_image
        else
            set odometry = accumulation + OF
        endif
Step 7: output odometry
Step 8: Goto Step 4

The Centeye Tam2 chip is a simple 16x16 image sensor with logarithmic pixels.
===============================================================================
Change Log: If you make changes to the code and redistribute it, please enter
the nature of your change and optionally your name below. 
2011 March 8: Birth of code (release 1), by Geoffrey L. Barrows

This version includes the following files:
DumpImage_r2
TamLib_r2
*/

// EEPROM library is used to store the calibration mask
#include <EEPROM.h>
// Location within EEPROM to store calibration mask.
#define CalibrationMaskAddress  0x00

// Other sketch libraries needed- In the Arduino environment, go to
// Sketch -> Add File... and make sure the following library sketches
// are included:
// TamLib_r2: Basic interface between Arduino and Tam chip on Rox1_v1 board
// DumpImage_r2: Routines to dump an image to the serial monitor

//=========================================================================
// GLOBAL VARIABLES
//-------------------------------------------------------------------------
// Global Image Variables
// Note that 2D images are stored as 1D arrays, with the elements filled
// in row-wise. Thus, if we store a 4x32 image into array img[], elements
// img[0] through img[31] store the first row of the image, elements img[32]
// through img[63] the second row of the image, and so on.

// For this script we are going to use just a 12x12 subimage from the 
// 16x16 array of the Tam2 chip. Generally we store 2D images as a 1D
// array, row wise, to simplify use of pointers associated with C arrays.
// The first row the image is stored in the first twelve elements, the 
// second row in the second twelve elements, and so on. Thus the
// 12x12 array requires 144 elements. Note that we tried writing this
// code using the raw 16x16 array (of 256 elements) but this seems to
// have filled up the Arduino's memory.

// Image size assumes a 12x12 array. Change this for different image sizes
#define IMAGESIZE 144
// Calibration- this is the fixed pattern noise mask, stored as shorts due
// to the 10-bit ADC
short cal[IMAGESIZE];
// This is the reference image, the image "from" which optical flow
// displacements are measured. This reference image is updated every 
// so often, as will be apparent below.
char refimage[IMAGESIZE];
// This is the image input every frame, and the image "to" which 
// optical flow displacements are measured.
char inpimage[IMAGESIZE];

//-------------------------------------------------------------------------
// Other Global Variables
// charbuf is a character buffer used for generating output strings to the
// serial monitor.
char charbuf[50];

// framecounter: gets incremented every frame- useful for frame rate measurement
int framecounter=0;

// Optical flow related values
int  accx,accy;  // accumulated optical flows in X and Y directions
int  maxdelta;  // "delta" amount that triggers updating the reference frame

// flag: 1 when the sensor accumulation is reset, such as at startup
char  firstframe;  

// outputmode: 1 when odometry is sent out to Serial every frame
// 0 when nothing is sent to Serial (e.g. quiet number)
char outputmode;

//For ping sensors
int startpin = A8;
int ping1 = A0;
int ping2 = A2;
int ping3 = A4;
int ping4 = A6;
int ping5 = A7;
int ping6 = A5;
int ping7 = A3;
int ping8 = A1;

int dist1 = 0;
int dist2 = 0;
int dist3 = 0;
int dist4 = 0;
int dist5 = 0;
int dist6 = 0;
int dist7 = 0;
int dist8 = 0;


//==========================================================================
// PRELIMINARY FUNCTIONS
//-------------------------------------------------------------------------
// dumpCommandList -- This dumps a list of available commands to the 
// serial monitor.
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

//-------------------------------------------------------------------------
// intro -- This outputs a welcome message to the serial monitor on
// program start
void intro(){
  Serial.println("Sample odometry code using Tam2 16x16 chip\n");
}

//-------------------------------------------------------------------------
// IIA2 -- This function computes optical flow between two images X1 and
// X2 using a simplified version of Mandyam Srinivasan's image interpolation
// algorithm. This algorithm assumes that displacements are generally on 
// the order of one pixel or less. The x and y displacements are sent
// out as floats. (See note at bottom of this function.) The #def value
// of_scale represents one pixel worth of displacement- if the displacement
// between X1 and X2 is one pixel in one direction, the corresponding of
// value (ofx or ofy) should be on the order of 100.
// This function is hard coded for a 12x12 image. It can be adapted to
// work with different array sizes by changing all instances of "12" to 
// the appropriate value below where noted.
// Credit- Thanks to "A.J." on Embedded Eye for optimizing this function.
// Apologies for ruining it with the "float" assignments at the end.
#define of_scale 100
void IIA2(char *X1, char *X2, float *ofx, float *ofy)
{
  int32_t  A11=0, A12=0, A22=0, b1=0, b2=0;
  int16_t  F2F1, F4F3, FCF0;
        
  // set up pointers
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
        
  // Note from Geof- The code below assigning OFX and OFY are from the
  // original implementations of the code. For some reason I cannot get
  // this algorithm to work unless I use the "float" assignment below.
  // My knowledge of casteing in C is limited, so it is probably a very
  // simple error. Anyway- apologizes for sticking with the float
  // assignement for now- it is wasteful of CPU cycles. I'll send a
  // free ArduEye to the first person who corrects this issue.
  //OFX=(signed char)(XS);
  //OFY=(signed char)(YS);
  *ofx = XS;
  *ofy = YS;
}

//=========================================================================
// ARDUINO CORE FUNCTIONS

//-------------------------------------------------------------------------
// setup -- Arduino initialization routine
void setup() {
  //initiates the ping sensors for read
  digitalWrite(startpin, HIGH);
  delay(2);
  digitalWrite(startpin, LOW);
  
  short i;
  attachInterrupt(0, read_pings, RISING);
  // Set up serial monitor- default value 9600 is used; higher values will
  // speed up operation of sensor since the serial monitor is the bottleneck
  // of this code.  
  Serial.begin(9600);
  
  // Set up Arduino to talk to Tam series chips  
  Tam_InitArduino(1);
  
  // Clear the images
  for (i=0; i<IMAGESIZE; ++i) {
    refimage[i] = inpimage[i] = 0;
    cal[i] = 150; // generic startup value
  }
    
  // Send introductory message to serial monitor    
  intro();
  dumpCommandList();
  
  // Other initializations
  firstframe=1; 
  accx=accy=0; // reset accumulations
  maxdelta=500; // square of displacement that triggers updating frame
  //outputmode = 0; // set initial output mode to quiet
  outputmode = 1; //set to constantly stream odometry data
  
  Serial.println("Reading calibration mask from EEPROM...");
  for (i=0; i<IMAGESIZE; ++i) {
    cal[i] = EEPROM.read(CalibrationMaskAddress+2*i);
    cal[i] += short(EEPROM.read(CalibrationMaskAddress+2*i+1))<<8;
  } 
}

//-------------------------------------------------------------------------
// loop -- main program loop. One iteration of this loop is one frame.
void loop() {
  short i;
  int ofx,ofy; // current measured displacement- integer versions
  float fofx,fofy; // current measured displacement- float versions
  int odox,odoy; // odometry x and y values
  int delta; // square of total displacement- integer version
  float fdelta; // square of total displacement- float version
  
  framecounter++;
  read_pings();
  delay(100);
  // Load 12x12 image into inpimage, using cal for fixed pattern noise mask.
  // Note that on program startup the calibration mask is effectively blank
  // and thus has no effect.
  Tam2LoadSubimageCharCalibrated(12,2,12,2,inpimage,cal); // Change 12 for different image sizes
  
  // If odometer is being reset (or at startup) then perform the following sequence
  if (firstframe) {
    // Set reference image = input image
    for (i=0; i<IMAGESIZE; ++i)
      refimage[i] = inpimage[i];
    // set flag to zero
    firstframe=0;
    // clear x and y accumulations
    accx=0; accy=0;
  }
  
  // Compute optical flow displacement between reference and input images
  IIA2(refimage, inpimage, &fofx, &fofy);
  
  // hack - Not sure why I have to pass variables as floats and convert
  // them to ints here.
  ofx=fofx; ofy=fofy;
  
  // fdelta and delta are basically the square of the sum of the x and y
  // displacements. remember h^2 = a^2 + b^2- we are just computing h^2
  fdelta = fofx*fofx + fofy*fofy;
  
  // Depending on whether the current displacement between the reference 
  // image and the current image is greater than the threshold set by maxdelta, 
  // perform one of the two sequences
  if (fdelta>maxdelta) { // Threshold exceeded: update reference image
    // Set reference image equal to input image
    for (i=0; i<IMAGESIZE; ++i)
      refimage[i] = inpimage[i];
    // compute current odometry values (accumulation plus current displacement)
    odox = accx + ofx;
    odoy = accy + ofy;
    // increment accumulations and set ofx and ofy to zero
    accx += ofx;
    accy += ofy;
    ofx=ofy=0;
    // The above sequence could be done a little bit more efficiently, but it
    // was done this way to make it more clear what is going on.
  } else { // Threshold not exceeded- just compute odometry
    // Compute odometry values
    odox = accx + ofx;
    odoy = accy + ofy;
  }    
  
  // At this point, values odox and odoy contain the odometry measurements
  // and can be used in any way desired.
  
  // If the output mode is 1, then we dump the odometry values to the
  // Serial monitor. More extensive outputs are possible, but of course
  // increase the amount of time sending data to the serial monitor.
  if (outputmode==1) {
    //Serial.print(odox);
    //Serial.print(" ");
    //Serial.println(odoy);
    Serial.print("odometry x,y: ");
    Serial.print(odox);
    Serial.print(",");
    Serial.print(odoy);
    Serial.print(";  ");
  }
   
  // Process commands from the serial monitor- the user can send the Arduino
  // commands to calibrate the sensor, dump screen shots, save/load 
  // fixed pattern noise calibration mask to/from EEPROM
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

void read_pings() {
  dist1 = analogRead(ping1);
  dist2 = analogRead(ping2);
  dist3 = analogRead(ping3); 
  dist4 = analogRead(ping4);
  dist5 = analogRead(ping5);
  dist6 = analogRead(ping6); 
  dist7 = analogRead(ping7);
  dist8 = analogRead(ping8); 
  Serial.print("ping: ");
  Serial.print(dist1);
  Serial.print(",");
  Serial.print(dist2);
  Serial.print(",");
  Serial.print(dist3);
  Serial.print(",");
  Serial.print(dist4);
  Serial.print(",");
  Serial.print(dist5);
  Serial.print(",");
  Serial.print(dist6);
  Serial.print(",");
  Serial.print(dist7);
  Serial.print(",");
  Serial.println(dist8);

//  int i, count;
//  //distance bar for ping1
//  count=0;
//  Serial.print("Ping 1: [");
//  Serial.print(dist1);
//  Serial.print("] ");
//  for (i=0; i<dist1; ++i){
//    count++;
//    if (count == 5){
//      Serial.print(")");
//      count = 0;
//    }
//  }
//  Serial.println();
  
}
