#include <math.h>
#include <MPU9250.h>

// An MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);

//Defining OUTPUT pins
#define BPin 8
#define APin 7
#define pwmPin 3 // pin 3 = 490HZ, pin 5 = 980Hz

//PID controller gains
#define Gain_Proportional 45.5 //45.5	// previously working value 45.5
#define  Gain_Integral 0.12		// previously working value 0.12
#define  Gain_Derivative 3.		// previously working value 3.0
#define  Gain_Rotor_Speed -0.02	// previously working value -0.02


float MotorSpeed = 0;
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

// History of Error values  (Where arduino max array size float = 8191 elements)
//#define Error_array_len 20
//float Error_array[Error_array_len][2];
//int Error_array_index = 0;


// Angle the PID controller is aiming to achieve 
float SetPointAngle = -0.06058467;
float AngleProportional = 0;
float AngleIntegral = 0;
float AngleDerivative = 0;

// Max motor speed
# define MaxSpeed 1.
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
		//case 'p':
		//	Serial.println("Printing AngularErrorArray...");
		//	for (int i = 0; i < Error_array_len; i++)
		//	{
		//		Serial.print(Error_array[i][0],6); //arduino only has 6-7 digit precision on both float and double
		//		Serial.print(" ");
		//		Serial.println(Error_array[i][1], 6); //arduino only has 6-7 digit precision on both float and double
		//		Error_array[i][0] = 0;
		//		Error_array[i][1] = 0;
		//	}
		//	Serial.println("Cleared Error_array values and resetting Error_array index...");
		//	Error_array_index = 0;
		//	Serial.println("Done.");
		//	switchByte = 's';
		//	break;
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
			set_speed(pid_controller());
		}
		break;
	case 'p':
		get_angle();
	case 's':
		MotorSpeed = 0;
		set_speed(MotorSpeed);	
		get_angle();
		break;
	default:
		MotorSpeed = 0;
		set_speed(MotorSpeed);
		get_angle();
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
	float Error = Angle_0 - SetPointAngle;
	//Serial.print(Angle_0_Time);
	//Serial.print(',');
	//Serial.println(Error, 4);



	if (abs(Error) >= 0.174533) {
		Serial.println("Error: Angle out of bounds");
		switchByte = 's';
		return 0;
	}

	AngleProportional = Gain_Proportional * Error;
	AngleIntegral =		AngleIntegral + Gain_Integral * Error * (Angle_0_Time - Angle_minus1_Time) / 1000000;
/*
	AngleDerivative =	w * AngleDerivative +
						Gain_Derivative * (1-w) * (-2 * (Angle_minus3 - SetPointAngle) + 9 * (Angle_minus2 - SetPointAngle) - 18 * (Angle_minus1 - SetPointAngle)	+ 11 * (Angle_0 - SetPointAngle))
										/ (6 * ((Angle_0_Time - Angle_minus3_Time) / 4)/1000000);
*/


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



/*
	AngleDerivative = Gain_Derivative *		(0.03846 * Angle_0 
											+0.03147 * Angle_minus1
											+0.02448 * Angle_minus2
											+0.01748 * Angle_minus3
											+0.01049 * Angle_minus4
											+0.00350 * Angle_minus5
											-0.00350 * Angle_minus6
											-0.01049 * Angle_minus7
											-0.01748 * Angle_minus8
											-0.02448 * Angle_minus9
											-0.03147 * Angle_minus10
											- 0.03846 * Angle_minus11)/((Angle_0_Time - Angle_minus11_Time) / (12*1000000));
*/


	PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput;

	MotorSpeed = constrain(PIDOutput, -MaxSpeed, MaxSpeed);

//	if (Error_array_index < Error_array_len) {
//		Error_array[Error_array_index][0] = Error;
//		Error_array[Error_array_index][1] = AngleDerivative;
//		Error_array_index += 1;
//	}
//	else if (Error_array_index < Error_array_len + 1) {
//		Serial.println("Array full");
//		Error_array_index += 1;
//	}
	
	return MotorSpeed;
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

	// Normalise accelerometer values in accordance with gravity, important as these values will be combined using 'atan2f' to increase the angles accuracy
	//float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, -gravity, gravity) / gravity;
	//float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -gravity, gravity) / gravity;

	// Decreased interval of constraint to only its working range (+/- 15 degrees from verticle) to help prevent effect of accelerometer spikes.
	float AccelX = constrain((IMU.getAccelX_mss() - AccelxBias) * AccelxSF, 8.0, 10) / gravity;
	float AccelZ = constrain((IMU.getAccelZ_mss() - AccelzBias) * AccelzSF, -5, 5) / gravity;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	float AccelAngle = atan2f(AccelZ, AccelX);

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
		Angle_0 = 0.998 * Angle_0 + (1-0.998) * AccelAngle;
	}
	float w = 0.85;
	Angle_0 = w * Angle_0 + (1 - w) * Angle_minus1;
}

