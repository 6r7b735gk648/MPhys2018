#include <math.h>
#include <Wire.h>

byte switchByte = 's';

#define int16_tMin -32768
#define int16_tMax 32767


// The setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();

	//Initialise serial For debug use
	Serial.begin(115200);
	Serial.setTimeout(2);
	while (!Serial) {
		Serial.println("Waiting for serial, how can this be?");
	}

	Serial.println("Set up successful");
}

// The loop function runs repeatedly until power down or reset
void loop() {
	//long StartTime = micros();
	if (Serial.available() > 0) {
		switchByte = Serial.read();
	}

	switch (switchByte)
	{
	case 'i':
		SendSpeed(5000);
		break;
	case 's':
		SendSpeed(0);
		break;
	default:
		SendSpeed(0);
		break;
	}
}

void SendSpeed(float Speed) {
	byte myArray[2];
	int16_t SetPoint = constrain(Speed, int16_tMin, int16_tMax); //-32, 768 .. 32, 767
	myArray[0] = (SetPoint >> 8) & 0xFF;
	myArray[1] = SetPoint & 0xFF;

	Wire.beginTransmission(9); // transmit to device #9
	Wire.write(myArray, 2);              // sends 
	Wire.endTransmission();    // stop transmitting
}
