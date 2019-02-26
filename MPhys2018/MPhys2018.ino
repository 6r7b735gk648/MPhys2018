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

//PID controller gains
float Gain_Proportional = 7.5;
float Gain_Integral = 0.0;
float Gain_Derivative = 0.;
float Gain_Rotor_Speed = -0.;
float MotorSpeed = 0;
float PIDOutput = 0;

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

// Used by MPU9250 for error checking
int status; 

// Controls loop switch
int switchByte = 'c';

// Time at which get_angle was last ran in micro seconds (overflow after approximately 70 minutes)
long TimePrevGetAngle = 0;

// Calibration value for Y axis gyro data, established at setup
float GyroAngle = 0;
float GyroYCal = 0; 

// 'Current' angle value in X-Z plane
float Angle = 0;
// Time at 'current' angle value
float AngleTime = 0;
// Previous angle value in X-Z plane (used for differential part of PID controller)
float AnglePrev = 0;
// Time at previous angle value
float AngleTimePrev = 0;

// Angle the PID controller is aiming to achieve 
float SetPointAngle = -0.06058467;
float AngleProportional = 0;
float AngleIntegral = 0;
float AngleDerivative = 0;

// Current motor speed [-1,1]
int speed = 0;
// Max motor speed
float MaxSpeed = 1;

// The setup function runs once when you press reset or power the board
void setup() {
	Serial.setTimeout(25);
	//Initialise serial to display angle data
	//Serial.begin(115200);
	Serial.begin(56000);
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
	//long StartTime = micros();
	if (Serial.available() > 0) {
		switchByte = Serial.read();
		Serial.write(switchByte);
	}

	
	//get_angle();
	switch (switchByte) 
	{
	case 'r':
		get_angle();
		break;
	case 'c':
		get_angle();
		SetPointAngle = Angle;
		break;
	case 'g':
			set_speed(pid_controller());
		break;
	case 's':
		MotorSpeed = 0;
		set_speed(MotorSpeed);	
		break;
	default:
		break;
	}
	//long RunTime = micros() - StartTime;
	//Serial.println(RunTime);
}

void set_speed(float Speed) {
	//if speed is less than zero, set direction 
	if (Speed < 0) {
		digitalWrite(APin, LOW);
		digitalWrite(BPin, HIGH);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
	else {
		digitalWrite(APin, HIGH);
		digitalWrite(BPin, LOW);
		analogWrite(pwmPin, int(abs(Speed) * 255));
	}
}

float pid_controller() {
	while (micros() < AngleTimePrev + 2500)
	{
	}
	get_angle();

	float Error = Angle - SetPointAngle;
	float TimeDelta = (AngleTime - AngleTimePrev) / 1000000;

	if (abs(Angle - SetPointAngle) >= 0.174533) {
		
		switchByte = 's';
		return 0;
	}

	AngleProportional = Gain_Proportional * Error;
	AngleIntegral = AngleIntegral + Gain_Integral * Error * TimeDelta;
	AngleDerivative = Gain_Derivative * (Error - (AnglePrev - SetPointAngle)) / TimeDelta;
	PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput;

	// Convert PIDOutput which is 'acceleration' and adding it to 90% of the motor speed to help slow the wheel down if stable, then normalised [-1,1] as a motor speed for PWM control
	///MotorSpeed = constrain(PIDOutput + MotorSpeed, -MaxSpeed, MaxSpeed);
	MotorSpeed = constrain(PIDOutput, -MaxSpeed, MaxSpeed);

	return MotorSpeed;
}

void get_angle() {
	AnglePrev = Angle;
	AngleTimePrev = TimePrevGetAngle;
	// Read the MPU9250 sensor
	IMU.readSensor();

	// Normalise accelerometer values in accordance with gravity, important as these values will be combined using 'atan2f' to increase the angles accuracy
	float AccelX = constrain(IMU.getAccelX_mss(), -9.81,9.81) / 9.81;
	float AccelZ = constrain(IMU.getAccelZ_mss(), -9.81, 9.81) / 9.81;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	float AccelAngle = atan2f(AccelZ, AccelX);

	// Correct gyro data using calibration value from setup
	float GyroY = IMU.getGyroY_rads() - GyroYCal;

	//Initalise time values for linear approx of gyro differentiation
	AngleTime = micros();
	float TimeDelta = (AngleTime - TimePrevGetAngle) / 1000000;

	// If time since get_angle was last ran is greater than 0.2 seconds, do not us gyro data as time interval too great for approxmation of gyro integration
	if (TimeDelta > 0.2) {
		// Set angle to Accelerometer angle
		Angle = AccelAngle;
	}
	else {
		// Calculate (gyro) angle using Y axis gyro values
		//GyroAngle = Angle + GyroY * TimeDelta / 1000000;
		GyroAngle = GyroAngle + GyroY * TimeDelta;
		Angle = Angle + GyroY*TimeDelta;
		// Calculate angle using complamentary filter
		Angle = 0.98 * Angle + 0.02 * AccelAngle;

	}

	// Set previous time to be current time, used if/ when this function is run again
	TimePrevGetAngle = AngleTime;
	
}

