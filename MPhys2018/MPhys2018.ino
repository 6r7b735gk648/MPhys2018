/*
 Name:		MPhys2018.ino
 Created:	12/13/2018 11:38:35 AM
 Author:	George
*/
#include <math.h>
#include <MPU9250.h>

//Defining OUTPUT pins
#define BPin 8
#define APin 7
#define pwmPin 3

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

long TimePrev = 0;
float AccelX;
float AccelZ;
float AccelAngle;
float GyroY;
float GyroYCal;
float GyroAngle;
float Angle;

int speed = 0;
float FuncSpeed = 1;

// the setup function runs once when you press reset or power the board
void setup() {
	//INitialise serial to display angle data
	Serial.begin(115200);
	while (!Serial) {}

	//Initalise OUTPUT pins
	pinMode(BPin, OUTPUT);
	pinMode(APin, OUTPUT);
	pinMode(pwmPin, OUTPUT);


	// start communication with IMU 
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
	IMU.readSensor();
	//Establish initial angle reading from accelerometer only (Sensor must be stationary for accuracy)
	TimePrev = micros();
	AccelX = IMU.getAccelX_mss() / 9.81;
	AccelZ = IMU.getAccelZ_mss() / 9.81;
	Angle = atan2f((float)AccelZ, (float)AccelX);
	//Calibrate the gyro (Sensor must be stationary for accuracy)
	GyroYCal = IMU.getGyroY_rads();
}

// the loop function runs over and over again until power down or reset
void loop() {
	//Increment speed
	speed += 1;
	float FuncSpeed = sin((float)speed / 1000);
	//if((millis()/1000)%5 == 0){
	//  FuncSpeed = 0.5;
	//  }
	//else if((millis()/1000)%1 == 0){
	//  FuncSpeed = 0.5;
	//  }





	if (FuncSpeed < 0) {
		digitalWrite(BPin, LOW);
		digitalWrite(APin, HIGH);
		analogWrite(pwmPin, int(abs(FuncSpeed) * 255));
	}
	else {
		digitalWrite(BPin, HIGH);
		digitalWrite(APin, LOW);
		analogWrite(pwmPin, int(abs(FuncSpeed) * 255));
	}

	// read the sensor
	IMU.readSensor();
	// display the data

	AccelX = IMU.getAccelX_mss() / 9.81;
	AccelZ = IMU.getAccelZ_mss() / 9.81;

	GyroY = IMU.getGyroY_rads() - GyroYCal;

	GyroAngle = Angle + GyroY * (micros() - TimePrev) / 1000000;

	Angle = GyroAngle;

	AccelAngle = atan2f((float)AccelZ, (float)AccelX);

	Angle = 0.98*Angle + 0.02 * AccelAngle;


	//Serial.print(GyroAngle*180/PI,6);
	//Serial.print("\t");

	Serial.print(AccelAngle * 180 / PI, 6);
	Serial.print("\t");

	Serial.print(Angle * 180 / PI, 6);
	Serial.print("\t");

	Serial.print(FuncSpeed);
	Serial.print("\t");

	Serial.println("");

	TimePrev = micros();

	delay(10);

}
