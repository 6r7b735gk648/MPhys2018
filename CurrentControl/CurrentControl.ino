#include <math.h>
#include <Wire.h>

//Defining IO pins
#define CS A0
#define BPin 8
#define APin 7
#define pwmPin 3 // pin 3 = 490HZ, pin 5 = 980Hz

float SignalProportional = 0.0;
float SignalIntegral = 0.0;

float PWMOutput = 0;
float Speed = 0;

float Current = 0;
float Error = 0;

float SetPoint = 0; 

#define SetPointSF 0.015

#define IntegralGain 0.0001
#define ProportionalGain 0.001 

// The setup function runs once when you press reset or power the board
void setup() {
	// Start serial connection for debugging
	Serial.begin(115200);
	// Start the I2C Bus as Slave on address 9
	Wire.begin(9);
	// Link function to I2C 'recieve data' event
	Wire.onReceive(receiveEvent);
}

// The recieveEvent function is called when the I2C data bus recieves data (Start is when SDA (serial data) is pulled low while SCL (serial clock) stays high.)
void receiveEvent(int bytes) {
	byte myArray[2];
	myArray[0] = Wire.read();
	myArray[1] = Wire.read();

	int16_t SetPointint16 = myArray[0]; //-32, 768 .. 32, 767
	SetPointint16 = (SetPointint16 << 8) | myArray[1];
	SetPoint = (float)SetPointint16 * SetPointSF;
}

// The loop function runs over and over again until power down or reset
void loop() {

	// Calculate the error in the feedback loop
	Error = abs(SetPoint) - Current;

	// Condition such that low RPMs don't bind the motor, this is specific to the reaction wheel pendulums stability.
//	if (SetPoint< 1 && SetPoint>-1) {
//		//Condition such that when a true zero signal is recieve, the motor actually stops being powered i.e. a stop signal
//		if (SetPoint == 0.0) {
//			PWMOutput = 0.0;
//		}
//	}
	// Else run the PI feedback loop.
//	else {
	if (PWMOutput < 1.0 && PWMOutput >= 0.0) {
		SignalIntegral += IntegralGain * Error;
	}
	//else if(PWMOutput > 1.0){
	//	Serial.print("Error: PWMOutput exceeded upper bound = ");
	//	Serial.println(PWMOutput, 8);
	//}
	//else if (PWMOutput < -0.0) {
	//	Serial.print("Error: PWMOutput exceeded lower bound = ");
	//	Serial.println(PWMOutput,8);
	//}

	SignalProportional = ProportionalGain * Error;
	PWMOutput = (SignalIntegral + SignalProportional);
//	}

	Speed = constrain(PWMOutput, 0, 1);


	//Sign of Speed value is converted to direction of motor 
	if (SetPoint < 0) {
		digitalWrite(APin, LOW);
		digitalWrite(BPin, HIGH);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
	else {
		digitalWrite(APin, HIGH);
		digitalWrite(BPin, LOW);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
	Current = max(analogRead(CS)-3.0,0.0);

	//Serial.print(SetPoint, 4);
	//Serial.print(',');
	//Serial.print(PWMOutput, 4);
	//Serial.print(',');
	//Serial.println(Current);
}