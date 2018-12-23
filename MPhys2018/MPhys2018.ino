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

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

// Used by MPU9250 for error checking
int status; 

// Controls loop switch
int switchByte;

// Time at which get_angle was last ran in micro seconds (overflow after approximately 70 minutes)
long TimePrev = 0;

// Calibration value for Y axis gyro data, established at setup
float GyroYCal; 

// 'Current' angle value in X-Z plane
float Angle;

// Current motor speed [-1,1]
int speed = 0;

// The setup function runs once when you press reset or power the board
void setup() {
	//Initialise serial to display angle data
	Serial.begin(115200);
	while (!Serial) {}

	//Initalise OUTPUT pins
	pinMode(BPin, OUTPUT);
	pinMode(APin, OUTPUT);
	pinMode(pwmPin, OUTPUT);


	// Start communication with IMU 
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
	
	// Calibrate the gyro (Sensor must be stationary for accuracy)
	IMU.readSensor();
	GyroYCal = IMU.getGyroY_rads();
}

// The loop function runs repeatedly until power down or reset
void loop() {
	if (Serial.available() > 0) {
		switchByte = Serial.read();
	}
	//get_angle();
	switch (switchByte) 
	{
	case 'r':
		get_angle();
		break;
	case 'g':
		set_speed(0.3);
		break;
	case 's':
		set_speed(0);
		break;
	default:
		break;
	}
}

void set_speed(float Speed) {
	//if speed is less than zero, set direction 
	if (Speed < 0) {
		digitalWrite(BPin, LOW);
		digitalWrite(APin, HIGH);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
	else {
		digitalWrite(BPin, HIGH);
		digitalWrite(APin, LOW);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
}

void get_angle() {
	// Read the MPU9250 sensor
	IMU.readSensor();

	// Normalise accelerometer values in accordance with gravity, important as these values will be combined using 'atan2f' to increase the angles accuracy
	float AccelX = constrain(IMU.getAccelX_mss(), -9.81,9.81) / 9.81;
	float AccelZ = constrain(IMU.getAccelZ_mss(), -9.81, 9.81) / 9.81;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	float AccelAngle = atan2f((float)AccelZ, (float)AccelX);

	// Correct gyro data using calibration value from setup
	float GyroY = IMU.getGyroY_rads() - GyroYCal;

	//Initalise time values for linear approx of gyro differentiation
	long TimeNow = micros();
	long TimeDelta = TimeNow - TimePrev;	
	float GyroAngle = NAN;

	// If time since get_angle was last ran is greater than 0.2 seconds, do not us gyro data as time interval too great for linear approxmation of gyro differatiation
	if (TimeDelta > 200000) {
		// Set angle to Accelerometer angle
		Angle = AccelAngle;
	}
	else {
		// Calculate (gyro) angle using Y axis gyro values
		GyroAngle = Angle + GyroY * TimeDelta / 1000000;
		// Calculate angle using complamentary filter
		Angle = 0.98 * GyroAngle + 0.02 * AccelAngle;
	}

	// Set previous time to be current time, used if/ when this function is run again
	TimePrev = TimeNow;

}

