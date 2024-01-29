/*
 Name:		Sketch3.ino
 Created:	2023/11/29 11:31:56
 Author:	llh13
*/
#include<Arduino.h>
// the setup function runs once when you press reset or power the board
void setup() {
	pinMode(13, HIGH);
}

// the loop function runs over and over again until power down or reset
void loop() {
	digitalWrite(13, HIGH);
}
