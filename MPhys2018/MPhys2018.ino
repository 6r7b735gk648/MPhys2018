#include <math.h>
#include <MPU9250.h>

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);


//PID controller gains					// Last stable value set
#define Gain_Proportional	700000		// 600000.0
#define  Gain_Integral		74000		// 64000.0 
#define  Gain_Derivative	50000		//40000.0 
#define  Gain_Rotor_Speed	-.001		//-.001

float PIDOutput = 0;

// Controls loop switch
char switchByte;


// 'Current' angle value in X-Z plane
float Angle_0 = 0,			Angle_0_Time = 0;
float Angle_minus1 = 0,		Angle_minus1_Time = 0;
float Angle_minus2 = 0,		Angle_minus2_Time = 0;
float Angle_minus3 = 0,		Angle_minus3_Time = 0;
float Angle_minus4 = 0,		Angle_minus4_Time = 0;
float Angle_minus5 = 0,		Angle_minus5_Time = 0;
float Angle_minus6 = 0,		Angle_minus6_Time = 0;
float Angle_minus7 = 0,		Angle_minus7_Time = 0;
float Angle_minus8 = 0,		Angle_minus8_Time = 0;
float Angle_minus9 = 0,		Angle_minus9_Time = 0;
float Angle_minus10 = 0,	Angle_minus10_Time = 0;
float Angle_minus11 = 0,	Angle_minus11_Time = 0;
float Angle_minus12 = 0,	Angle_minus12_Time = 0;
float Angle_minus13 = 0,	Angle_minus13_Time = 0;


#define int16_tMin -32768
#define int16_tMax 32767

// Angle the PID controller is aiming to achieve 
float SetPointAngle = 0;
float AngleProportional = 0;
float AngleIntegral = 0;
float AngleDerivative = 0;


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
	status = IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
	if (status < 0) {
		Serial.println("Set digital low pass filter unsuccessful");
		Serial.print("Error Status: ");
		Serial.println(status);
		while (1) {}
	}
	Serial.println("Set up successful");
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
		if (micros() > Angle_minus1_Time + 1000) // Date sample rate of 1000Hz (over 2x DLPF for both accel and gyro) gives 1000 microseconds between each loop.
		{
			SetPointAngle = Angle_0;
		}
		break;

	case 'g':
		AngleProportional = 0;
		AngleIntegral = 0;
		AngleDerivative = 0;
		switchByte = 'q';
		Serial.println("PID values cleared");
		Serial.println("Running PID stabilisation...");
		break;

	case 'q':
		if (micros() > Angle_minus1_Time + 1000) // Date sample rate of 1000Hz (over 2x DLPF for both accel and gyro) gives 1000 microseconds between each loop.
		{
			pid_controller();
		}
		break;

	case 's':
		SendSpeed(0);	
		get_angle();
		break;

	default:

		SendSpeed(0);
		get_angle();
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

void pid_controller() {
	get_angle();
	
	float Error = Angle_0 - SetPointAngle;

	//Stops motor if device becomes unstable and falls over: (+-) 0.174533 rad = 10 degrees
	if (abs(Error) >= 0.174533) {
		Serial.println("Error: Angle out of bounds");
		switchByte = 's';
	}

	AngleProportional = Gain_Proportional * Error;
	
	AngleIntegral =	AngleIntegral + Gain_Integral * Error * (Angle_0_Time - Angle_minus1_Time) / 1000000;

	AngleDerivative =	Gain_Derivative *	(((Angle_0 - SetPointAngle) - (Angle_minus1 - SetPointAngle))	/ ((Angle_0_Time - Angle_minus1_Time)/1000000)* (.27473) +
											 ((Angle_minus1 - SetPointAngle) - (Angle_minus2 - SetPointAngle)) / ((Angle_minus1_Time - Angle_minus2_Time) / 1000000) * (.24176) +
											 ((Angle_minus2 - SetPointAngle) - (Angle_minus3 - SetPointAngle)) / ((Angle_minus2_Time - Angle_minus3_Time) / 1000000) * (.20879) +
											 ((Angle_minus3 - SetPointAngle) - (Angle_minus4 - SetPointAngle)) / ((Angle_minus3_Time - Angle_minus4_Time) / 1000000) * (.17582) +
											 ((Angle_minus4 - SetPointAngle) - (Angle_minus5 - SetPointAngle)) / ((Angle_minus4_Time - Angle_minus5_Time) / 1000000) * (.14286) +
											 ((Angle_minus5 - SetPointAngle) - (Angle_minus6 - SetPointAngle)) / ((Angle_minus5_Time - Angle_minus6_Time) / 1000000) * (.10989) +
											 ((Angle_minus6 - SetPointAngle) - (Angle_minus7 - SetPointAngle)) / ((Angle_minus6_Time - Angle_minus7_Time) / 1000000) * (.07692) +
											 ((Angle_minus7 - SetPointAngle) - (Angle_minus8 - SetPointAngle)) / ((Angle_minus7_Time - Angle_minus8_Time) / 1000000) * (.04396) +
											 ((Angle_minus8 - SetPointAngle) - (Angle_minus9 - SetPointAngle)) / ((Angle_minus8_Time - Angle_minus9_Time) / 1000000) * (.01099) +
											 ((Angle_minus9 - SetPointAngle) - (Angle_minus10 - SetPointAngle)) / ((Angle_minus9_Time - Angle_minus10_Time) / 1000000) * (-.02198) +
											 ((Angle_minus10 - SetPointAngle) - (Angle_minus11 - SetPointAngle)) / ((Angle_minus10_Time - Angle_minus11_Time) / 1000000) * (-.05495) +
											 ((Angle_minus11 - SetPointAngle) - (Angle_minus12 - SetPointAngle)) / ((Angle_minus11_Time - Angle_minus12_Time) / 1000000) * (-.08791) +
											 ((Angle_minus12 - SetPointAngle) - (Angle_minus13 - SetPointAngle)) / ((Angle_minus12_Time - Angle_minus13_Time) / 1000000) * (-.12088));


	PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput;
	
	SendSpeed(PIDOutput);
}

void get_angle() {
	// Set previous angle and time to be current angle time, before calculating new values
	Angle_minus13 = Angle_minus12;
	Angle_minus13_Time = Angle_minus12_Time;
	Angle_minus12 = Angle_minus11;
	Angle_minus12_Time = Angle_minus11_Time;
	Angle_minus11 = Angle_minus10;
	Angle_minus11_Time = Angle_minus10_Time;
	Angle_minus10 = Angle_minus9;
	Angle_minus10_Time = Angle_minus9_Time;
	Angle_minus9 = Angle_minus8;
	Angle_minus9_Time = Angle_minus8_Time;
	Angle_minus8 = Angle_minus7;
	Angle_minus8_Time = Angle_minus7_Time;
	Angle_minus7 = Angle_minus6;
	Angle_minus7_Time = Angle_minus6_Time;
	Angle_minus6 = Angle_minus5;
	Angle_minus6_Time = Angle_minus5_Time;
	Angle_minus5 = Angle_minus4;
	Angle_minus5_Time = Angle_minus4_Time;
	Angle_minus4 = Angle_minus3;
	Angle_minus4_Time = Angle_minus3_Time;
	Angle_minus4 = Angle_minus3;
	Angle_minus4_Time = Angle_minus3_Time;
	Angle_minus3 = Angle_minus2;
	Angle_minus3_Time = Angle_minus2_Time;
	Angle_minus2 = Angle_minus1;
	Angle_minus2_Time = Angle_minus1_Time;
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
	float AccelAngle = atan2f(AccelZ, AccelX);

	float AccelAngle = 

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
	else {
		// Calculate gyro angle by integrating (multiplying by timesince last measurment) and adding it to the previous angle measurement
		Angle_0 = Angle_0 + IMU.getGyroY_rads()*TimeDelta;
		// Calculate angle using complamentary filter
		Angle_0 = 0.995 * Angle_0 + (1-0.995) * AccelAngle;
	}
	float w = 0.88;
	Angle_0 = w * Angle_0 + (1 - w) * Angle_minus1;
}

