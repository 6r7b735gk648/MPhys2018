#include <math.h>
#include <Wire.h>
// Include the required Wire library for I2C<br>#include 

int16_t SetPoint = 0;  //-32, 768 .. 32, 767
long LastTime = 0;

byte myArray[2];

// the setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();
	Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (millis() - LastTime > 2000) {
		SetPoint = random(-32000, 32000);
		Serial.println(SetPoint);
		LastTime = millis();

		myArray[0] = (SetPoint >> 8) & 0xFF;
		myArray[1] = SetPoint & 0xFF;

		Wire.beginTransmission(9); // transmit to device #9
		Wire.write(myArray,2);              // sends 
		Wire.endTransmission();    // stop transmitting

	}


}
