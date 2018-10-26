/*
 Name:		FirstDraft.ino
 Created:	10/26/2018 4:15:40 PM
 Author:	Jonathan
*/

#include "libraries\MPU9250\MPU9250.h"
#include "libraries\quaternionFileters\quaternionFilters.h"
#include <math.h>

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;  // Set up pin 13 led for toggling
int vibPin = 9;  // Vibration motor pin. 

// the setup function runs once when you press reset or power the board
void setup() {

}

// the loop function runs over and over again until power down or reset
void loop() {
  
}
