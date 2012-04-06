/*
DumpImage -- Library of functions to dump 1D and 2D images to the
serial monitor.

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
2011 February 21: birth, by Geoffrey L. Barrows
2011 February 28: Revision 1, misc edits
2011 March 8: Revision 2. Added dumpCharImageAsciiSlow
*/

//-----------------------------------------------------------------------------
// Global Variables

// AsciiDispChar[11] contains 10 characters for rendering intensities. Note that
// these are loosely from "most stuff" to "least stuff"
char AsciiDispChar[11] = "#@%*=+-,. "; // List of characters
char NumAsciiDispChar = 10; // Number of characters in list

//-----------------------------------------------------------------------------
// dumpShortImageMatlabSlow -- Dumps a 2D image to the serial monitor in a form
// that may be copied into MATLAB as a matrix. This is useful for showing
// the full integer type values rather than a rendering. Array img is the 
// 2D image, stored row-wise as a linear array. numrows and numcolumns contain
// the number of rows and columns in the array.
void dumpShortImageMatlabSlow(short *img, short numrows, short numcolumns) {
  short m,n,*pix;

  // The MATLAB variable will be "Data"  
  Serial.println("Data = [");
  
  // Initialize pointer
  pix=img;
  
  // Loop through rows and columns and print out values
  for (m=0; m<numrows; ++m) {
    for (n=0; n<numcolumns; ++n) {
      short pixout = *pix;
      Serial.print(pixout); // print out value to serial monitor
      Serial.print(" "); // print out space
      ++pix;
    }
    Serial.println(""); // end of line- print carriage return
  }
  
  Serial.println("];"); // finish defining MATLAB variable
}

//-----------------------------------------------------------------------------
// dumpCharImageMatlabSlow -- This function is exactly the same as the 
// function dumpShortImageMatlabSlow except the image is an image of chars.
void dumpCharImageMatlabSlow(char *img, short numrows, short numcolumns) {
  short m,n;
  char *pix;

  // The MATLAB variable will be "Data"  
  Serial.println("Data = [");
  
  // Initialize pointer
  pix=img;
  
  // Loop through rows and columns and print out values
  for (m=0; m<numrows; ++m) {
    for (n=0; n<numcolumns; ++n) {
      short pixout = *pix;
      Serial.print(pixout); // print out value to serial monitor
      Serial.print(" "); // print out space
      ++pix;
    }
    Serial.println(""); // end of line- carrriage return
  }
  
  Serial.println("];"); // finish defining MATLAB variable
}

//-----------------------------------------------------------------------------
// dumpShortImageAsciiSlow -- This function dumps the image to the serial monitor
// but as an ASCII rendered image rather than as numbers. img, numrows, and 
// numcolumns are as above. mini and maxi are respective minimum and maximum values
// used to define the range of pixel intensities. If these are "0" then we compute
// them. The character string AsciiDispChar defined above describes how the
// intensities are rendered.
void dumpShortImageAsciiSlow(short *img, short numrows, short numcolumns, short mini, short maxi) {
  short i,m,n,*pix,delta;
  
  // if mini==0 then we need to compute mini = minimum pixel value
  if (mini==0) {
    mini = *img;
    for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
      if (*pix<mini)
        mini=*pix;
    }
  }
  
  // if maxi==0 then we need to compute maxi = maximum pixel value
  if (maxi==0) {
    maxi = *img;
    for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
      if (*pix>maxi)
        maxi=*pix;
    }
  }
  
  // Now we compute "delta", which is how many intensity steps are defined
  // by one character of AsciiDispChar
  delta = maxi-mini;
  delta = delta / NumAsciiDispChar;
  
  // Now the fun part- loop through and display the image
  pix=img; // first initialize pointer
  for (m=0; m<numrows; ++m) {
    for (n=0; n<numcolumns; ++n) {
      // for the given pixel, rescale intensity to i so that i defines
      // one character of the string AsciiDispChar
      i = *pix - mini;
      i = i / delta;
      // make sure i is not out of range...
      if (i<0)
        i=0;
      if (i>9)
        i=9;
      // send character to the serial monitor
      Serial.print(AsciiDispChar[i]);
      // advance pointer to next pixel
      pix++;
    }
    Serial.println(" "); // Carriage return at end of line
  }
}

//-----------------------------------------------------------------------------
// dumpCharImageAsciiSlow -- This is the same as dumpShortImageAsciiSlow except
// that the image is stored as an array of chars rather than shorts
void dumpCharImageAsciiSlow(char *img, short numrows, short numcolumns, short mini, short maxi) {
  short i,m,n,delta;
  char *pix;
  
  // if mini==0 then we need to compute mini = minimum pixel value
  if (mini==0) {
    mini = *img;
    for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
      if (*pix<mini)
        mini=*pix;
    }
  }
  
  // if maxi==0 then we need to compute maxi = maximum pixel value
  if (maxi==0) {
    maxi = *img;
    for (i=0,pix=img; i<numrows*numcolumns; ++i,++pix) {
      if (*pix>maxi)
        maxi=*pix;
    }
  }
  
  // Now we compute "delta", which is how many intensity steps are defined
  // by one character of AsciiDispChar
  delta = maxi-mini;
  delta = delta / NumAsciiDispChar;
  
  // Now the fun part- loop through and display the image
  pix=img; // first initialize pointer
  for (m=0; m<numrows; ++m) {
    for (n=0; n<numcolumns; ++n) {
      // for the given pixel, rescale intensity to i so that i defines
      // one character of the string AsciiDispChar
      i = *pix - mini;
      i = i / delta;
      // make sure i is not out of range...
      if (i<0)
        i=0;
      if (i>9)
        i=9;
      // send character to the serial monitor
      Serial.print(AsciiDispChar[i]);
      // advance pointer to next pixel
      pix++;
    }
    Serial.println(" "); // Carriage return at end of line
  }
}

      
