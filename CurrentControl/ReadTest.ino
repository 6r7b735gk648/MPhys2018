#include <math.h>

float SetPoint = 0;
long LastTime = 0;

// the setup function runs once when you press reset or power the board
void setup() {
	Serial.begin(115200);
}

// the loop function runs over and over again until power down or reset
void loop() {
	if (millis() - LastTime > 2000) {
		SetPoint = random(0, 255);
		LastTime = millis();
	}
	analogWrite(3, SetPoint);

	Serial.println(SetPoint / 255 * 1023);
}
