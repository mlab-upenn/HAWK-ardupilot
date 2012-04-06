/*
TamLib -- Library of functions to interface with Tam2 and Tam4 chips
This library is for the Rox1_v1 Arduino interface board.

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
Change Log: If you make changes to the code and redistribute it, please enter
the nature of your change and optionally your name below. 
2011 February 21: Birth of code, by Geoffrey L. Barrows
2011 February 28: Revision 1, removed dependency on Arduino board type
2011 March 8: Revision 2, added Tam2LoadSubimageShort and Tam2LoadSubimageCharCalibrated
*/

//-----------------------------------------------------------------------------
// Define Interface with Tam series chips
#define  TAM_BD_CLOCKPIN    13  // The chip clock pin CLK is connected to Arduino digital 7
#define  TAM_BD_CLEARBAR    15  // The chip reset pin CLB is connected to Arduino digital 6
#define  TAM_BD_ANALOG      14  // The chip analog out pin ANALOG is connected to Arduino digital 0

#define  TAM2_IMAGESIZE  256  // Number of pixels in 16x16 Tam2 chip
#define  TAM4_IMAGESIZE  128  // Number of pixels in 4x32 Tam2 chip

//-----------------------------------------------------------------------------
// Tam_InitArduino
// Initializes the Arduino
void Tam_InitArduino(char board) {
  pinMode(TAM_BD_CLOCKPIN,OUTPUT);
  pinMode(TAM_BD_CLEARBAR,OUTPUT);
}

//-----------------------------------------------------------------------------
// Tam2_LoadImage -- Loads a 16x16 Tam2 image (256 pixels) into an 
// array of shorts. Note that array img must be preallocated.
void  Tam2_LoadImage(short *img) {
  short i;
  
  // reset Tam2 pixel counter by pulsing low the CLB line
  digitalWrite(TAM_BD_CLEARBAR,LOW);
  digitalWrite(TAM_BD_CLEARBAR,HIGH);
  // read through pixels
  for (i=0; i<TAM2_IMAGESIZE; ++i) {
    // delay slightly
    delayMicroseconds(10); // small delay to let analog pathway settle
    // read in pixel
    img[i] = analogRead(TAM_BD_ANALOG);
    // pulse clock line to advanced to next pixel
    digitalWrite(TAM_BD_CLOCKPIN,HIGH);
    digitalWrite(TAM_BD_CLOCKPIN,LOW);
  }
}

//-----------------------------------------------------------------------------
// Tam4_LoadImage -- Loads a 4x32 Tam2 image (128 pixels) into an 
// array of shorts. Note that array img must be preallocated.
void  Tam4_LoadImage(short *img) {
  short i;
  
  // reset Tam4 pixel counter by pulsing low the CLB line
  digitalWrite(TAM_BD_CLEARBAR,LOW);
  digitalWrite(TAM_BD_CLEARBAR,HIGH);
  // read through pixels
  for (i=0; i<TAM4_IMAGESIZE; ++i) {
    // delay slightly
    delayMicroseconds(10); // small delay to let analog pathway settle
    // read in pixel
    img[i] = analogRead(TAM_BD_ANALOG);
    // pulse clock line to advanced to next pixel
    digitalWrite(TAM_BD_CLOCKPIN,HIGH);
    digitalWrite(TAM_BD_CLOCKPIN,LOW);
  }
}

//-----------------------------------------------------------------------------
// Tam_LoadSubImage -- Generic function that loads a subimage from one
// of the Tam chips. This is useful for loading one or more full rows.
// The user must specify how many times the clock line is pulsed to skip
// over unwanted pixels, and how many pixels are then read. Array img is
// the destination array and must be preallocated. skip is how many
// pixels to skip over. load is how many pixels to acquire and store
// in array img.
void  Tam_LoadSubImage(short *img, short skip, short load) {
  short i;

  // reset Tam pixel counter by pulsing low the CLB line
  digitalWrite(TAM_BD_CLEARBAR,LOW);
  digitalWrite(TAM_BD_CLEARBAR,HIGH);
  // Pulse clock to skip over unwanted pixels
  for (i=0; i<skip; ++i) {
    digitalWrite(TAM_BD_CLOCKPIN,HIGH);
    digitalWrite(TAM_BD_CLOCKPIN,LOW);
  }    
  // Acquire and store wanted pixels
  for (i=0; i<load; ++i) {
    // delay slightly
    delayMicroseconds(10); // small delay to let analog pathway settle
    // read in pixel
    img[i] = analogRead(TAM_BD_ANALOG);
    // pulse clock line to advanced to next pixel
    digitalWrite(TAM_BD_CLOCKPIN,HIGH);
    digitalWrite(TAM_BD_CLOCKPIN,LOW);
  }
}

//----------------------------------------------------------------------------
// Tam2LoadSubimageShort -- This function reads a subimage from the 16x16 raw Tam2 array.
// A subimage is a rectangular image of size numrows x numcols. The upper left end
// of the subwindow is determined by pixel (firstrow,firstcol). This program pulses
// CLKIN through every pixel of the window, and digitizes only the pixels necessary.
// The resulting image is stored in array img. This function loads the image
// into an array of shorts.
void Tam2LoadSubimageShort(char numrows, char firstrow, char numcols, char firstcol, short *img) {
  char r,c;
  short *pi;
  
  pi = img; // set pointer pi to first element of img
  // Reset the Tam's pixel counter
  digitalWrite(TAM_BD_CLEARBAR,LOW);  // resets on low
  digitalWrite(TAM_BD_CLEARBAR,HIGH);  // "resting" state is high
  
  for (r=0; r<16; ++r) { // loop through rows of Tam2 chip
    for (c=0; c<16; ++c) { // loop through columns of Tam2 chip
      // Check to see if we digitize this pixel
      if ( r>=firstrow && r<firstrow+numrows && c>=firstcol && c<firstcol+numcols ) {
        // Yes. We digitize this pixel.
        delayMicroseconds(20); // Perfunctory delay to let ADC settle
        *pi = analogRead(TAM_BD_ANALOG); // digitize and store in img[pi]
        ++pi; // advance pointer
      }      
      // Pulse clock line
      digitalWrite(TAM_BD_CLOCKPIN,HIGH);
      digitalWrite(TAM_BD_CLOCKPIN,LOW);       
    }
  }
}

//----------------------------------------------------------------------------
// Tam2LoadSubimageCharCalibrated -- Same as Tam2LoadSubimageShort but
// 1) loads subimage into an array of chars, and 
// 2) uses array cal for the fixed pattern noise calbiration to subtract
void Tam2LoadSubimageCharCalibrated(char numrows, char firstrow, char numcols, char firstcol, char *img, short *cal) {
  char r,c;
  char *pi;
  short adcin;
  short *pc;
  
  pi = img; // point pi to first pixel of image
  pc = cal; // point pc to first pixel of calibration image
  // Reset the Tam's pixel counter
  digitalWrite(TAM_BD_CLEARBAR,LOW);
  digitalWrite(TAM_BD_CLEARBAR,HIGH);  
  
  for (r=0; r<16; ++r) { // loop through rows of Tam2 chip
    for (c=0; c<16; ++c) { // loop through columns of Tam2 chip
      // Check to see if we digitize this pixel
      if ( r>=firstrow && r<firstrow+numrows && c>=firstcol && c<firstcol+numcols ) {
        // Yes. We digitize this pixel.
        delayMicroseconds(20); // Perfunctory delay to let ADC settle
        adcin = analogRead(TAM_BD_ANALOG); // digitize
        adcin += 0 - *pc; // calibrate: "0" could be replaced with another value
        if (adcin>127)
          adcin=127;
        if (adcin<-127)
          adcin=-127;
        *pi = adcin; // record calibrated pixel
        ++pi; ++pc; // advance pointers
      }      
      // Pulse clock line
      digitalWrite(TAM_BD_CLOCKPIN,HIGH);
      digitalWrite(TAM_BD_CLOCKPIN,LOW);       
    }
  }
}
