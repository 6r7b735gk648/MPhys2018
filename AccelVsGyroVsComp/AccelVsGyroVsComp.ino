#include <math.h>
#include <MPU9250.h>
#include <Wire.h>

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

// Controls loop switch
char switchByte;
int Speed;

// 'Current' angle value in X-Z plane
float Angle_0 = 0, Angle_0_Time = 0;
float Angle_minus1 = 0, Angle_minus1_Time = 0;

float InitialAngle = 0;

float GyroAngle = 0;
float AccelAngle = 0;

#define int16_tMin -32768
#define int16_tMax 32767

int Count = 0;

#define BPin 8
#define APin 7
#define pwmPin 3 // pin 3 = 490HZ, pin 5 = 980Hz

// Scale factor for accelerometer z reading
#define AccelxBias -0.0208
#define AccelxSF 0.999759249
#define AccelzBias -0.635659927
#define AccelzSF 0.991977013
#define gravity 9.8071495

// The setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();

	//Initialise serial For debug use
	Serial.begin(115200);
	Serial.setTimeout(2);
	while (!Serial) {
		Serial.println("Waiting for serial, how can this be?");
	}

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
	//status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_MaxHZ);
	status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_41HZ);
	if (status < 0) {
		Serial.println("Set digital low pass filter unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
	Serial.println("Set up successful");

	// Read the MPU9250 sensor
	if (IMU.readSensor() < 0) {
		Serial.println("Error: No data in MPU9255s buffer");
	}

	// Decreased interval of constraint to only its working range (+/- 15 degrees from verticle) to help prevent effect of accelerometer spikes.
	float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, 8.0, 10) / gravity;
	float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -5, 5) / gravity;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	InitialAngle = atan2f(AccelZ, AccelX);
	GyroAngle = InitialAngle;
}

// The loop function runs repeatedly until power down or reset
void loop() {
	if (Serial.available() > 0) {
		switchByte = Serial.read();
	}
	switch (switchByte)
	{
	case 'r':
		if (Count < 50) {
			
			get_angle();
			Serial.print(Speed);
			Serial.print(',');
			Serial.print(AccelAngle, 4);
			Serial.print(',');
			Serial.print(GyroAngle, 4);
			Serial.print(',');
			Serial.println(Angle_0, 4);
			Count += 1;
		}
		else {
			Speed += 1;
			analogWrite(pwmPin, Speed);
			Count = 0;
		}

		delay(10);

		break;
	case 's':
		analogWrite(pwmPin, 0);
		break;

	default:
		analogWrite(pwmPin, 0);
		break;
	}
	digitalWrite(APin, LOW);
	digitalWrite(BPin, HIGH);
}


void get_angle() {
	// Set previous angle and time to be current angle time, before calculating new values
	Angle_minus1 = Angle_0;
	Angle_minus1_Time = Angle_0_Time;


	// Read the MPU9250 sensor
	if (IMU.readSensor() < 0) {
		Serial.println("Error: No data in MPU9255s buffer");
	}

	// Decreased interval of constraint to only its working range (+/- 15 degrees from verticle) to help prevent effect of accelerometer spikes.
	float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, 8.0, 10) / gravity;
	float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -5, 5) / gravity;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	AccelAngle = atan2f(AccelZ, AccelX);

	//Initalise time values for linear approx of gyro differentiation
	Angle_0_Time = micros();
	float TimeDelta = (Angle_0_Time - Angle_minus1_Time) / 1000000;

	// If time since get_angle was last ran is greater than 0.2 seconds, do not us gyro data as time interval too great for approxmation of gyro integration
	if (TimeDelta > 0.2) {
		// Set angle to Accelerometer angle
		if (Angle_0 != 0) {
			Serial.println("Error: Gyroscopic data timeout");
		}
		Angle_0 = AccelAngle;
	}
	else if (AccelAngle > Angle_0 - 0.1 && AccelAngle < Angle_0 + 0.1) {
		// Calculate gyro angle by integrating (multiplying by timesince last measurment) and adding it to the previous angle measurement
		GyroAngle += IMU.getGyroY_rads()*TimeDelta;
		Angle_0 = Angle_0 + IMU.getGyroY_rads()*TimeDelta;
		// Calculate angle using complamentary filter
		Angle_0 = 0.999 * Angle_0 + (1 - 0.999) * AccelAngle;
	}
	else {
		// Calculate gyro angle by integrating (multiplying by timesince last measurment) and adding it to the previous angle measurement
		GyroAngle += IMU.getGyroY_rads()*TimeDelta;
		Angle_0 = Angle_0 + IMU.getGyroY_rads()*TimeDelta;
	}
	float w = 1;
	Angle_0 = w * Angle_0 + (1 - w) * Angle_minus1;
}

