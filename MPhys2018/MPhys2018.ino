#include <math.h>
#include <MPU9250.h>

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

//Defining OUTPUT pins
#define BPin 7
#define APin 8
#define pwmPin 3

//PID controller gains
float Gain_Proportional = 6.0;
float Gain_Integral = 0.0;
float Gain_Derivative = 0.4;
float Gain_Rotor_Speed = -0.;
float MotorSpeed = 0;
float PIDOutput = 0;

// Controls loop switch
int switchByte = 'c';


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

// Max motor speed
# define MaxSpeed 0.8
// Scale factor for accelerometer z reading
#define AccelxBias -0.0208
#define AccelxSF 0.999759249
#define AccelzBias -0.635659927
#define AccelzSF 0.991977013
#define gravity 9.8071495

// The setup function runs once when you press reset or power the board
void setup() {
	
	//Initialise serial For debug use
	Serial.begin(115200);
	Serial.setTimeout(25);
	while (!Serial) {}

	// Start communication with MPU9255
	int status;
	status = IMU.begin();
	if (status < 0) {
		Serial.println("IMU initialization unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
	
	// Calibrate the gyro (Sensor must be stationary for accuracy)
	status = IMU.calibrateGyro();
	if (status < 0) {
		Serial.println("Gyro calibration unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}

	// Set full scale range of accelerometer to [-2,2] g 
	status = IMU.setAccelRange(MPU9250::ACCEL_RANGE_2G);
	if (status < 0) {
		Serial.println("Gyro calibration unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}

	// Set full scale range of gyroscope to [-500,500] deg/s
	status = IMU.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
	if (status < 0) {
		Serial.println("Set gyroscope range unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}

	// Set Digital Low Pass Filter (DLPF) bandwidth to 184 Hz
	status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_MaxHZ);
	if (status < 0) {
		Serial.println("Set digital low pass filter unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
}

// The loop function runs repeatedly until power down or reset
void loop() {
	//long StartTime = micros();
	if (Serial.available() > 0) {
		switchByte = Serial.read();
		switch (switchByte)
		{
		case 'r':
			Serial.println("Reading angle data (running get_angle)");
			break;
		case 'c':
			Serial.println("Calibrating setpoint...");
			break;
		case 'g':
			Serial.println("Running PID stabilisation...");
			break;
		case 's':
			Serial.println("STOP!");
			break;
		default:
			break;
		}
	}

	switch (switchByte) 
	{
	case 'r':
		get_angle();

		break;
	case 'c':

		get_angle();
		if (micros() > AngleTimePrev + 1000) // Date sample rate of 1000Hz (over 2x DLPF for both accel and gyro) gives 1000 microseconds between each loop.
		{
			SetPointAngle = Angle;
		}
		
		break;
	case 'g':
		if (micros() > AngleTimePrev + 1000) // Date sample rate of 1000Hz (over 2x DLPF for both accel and gyro) gives 1000 microseconds between each loop.
		{
			set_speed(pid_controller());
		}
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
	//Sign of Speed value is converted to direction of motor 
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
	get_angle();
	float TimeDelta = (AngleTime - AngleTimePrev) / 1000000;
	float Error = Angle - SetPointAngle;
	

	
	if (abs(Error) >= 0.174533) {
		Serial.println("Error: Angle out of bounds");
		switchByte = 's';
		return 0;
	}

	AngleProportional = Gain_Proportional * Error;
	AngleIntegral = AngleIntegral + Gain_Integral * Error * TimeDelta;
	AngleDerivative = Gain_Derivative * (Error - (AnglePrev - SetPointAngle)) / TimeDelta;
	PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput;

	MotorSpeed = constrain(PIDOutput, -MaxSpeed, MaxSpeed);

	return MotorSpeed;
}

void get_angle() {
	// Set previous angle and time to be current angle time, before calculating new values
		AnglePrev = Angle;
	AngleTimePrev = AngleTime;

	// Read the MPU9250 sensor
	if (IMU.readSensor() < 0) {
		Serial.println("Error: No data in MPU9255s buffer");
	}

	// Normalise accelerometer values in accordance with gravity, important as these values will be combined using 'atan2f' to increase the angles accuracy
	//float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, -gravity, gravity) / gravity;
	//float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -gravity, gravity) / gravity;

	// Decreased interval of constraint to only its working range (+/- 15 degrees from verticle) to help prevent effect of accelerometer spikes.
	float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, 8.0, 10) / gravity;
	float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -5, 5) / gravity;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	float AccelAngle = atan2f(AccelZ, AccelX);

	//Initalise time values for linear approx of gyro differentiation
	AngleTime = micros();
	float TimeDelta = (AngleTime - AngleTimePrev) / 1000000;

	// If time since get_angle was last ran is greater than 0.2 seconds, do not us gyro data as time interval too great for approxmation of gyro integration
	if (TimeDelta > 0.2) {
		// Set angle to Accelerometer angle
		Angle = AccelAngle;
		Serial.println("Error: Gyroscopic data timeout");
	}
	else {
		// Calculate gyro angle by integrating (multiplying by timesince last measurment) and adding it to the previous angle measurement
		Angle = Angle + IMU.getGyroY_rads()*TimeDelta;
		// Calculate angle using complamentary filter
		Angle = 0.998 * Angle + (1-0.998) * AccelAngle;
	}

}

