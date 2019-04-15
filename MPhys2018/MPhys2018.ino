#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <MPU9250.h>
MPU9250 IMU(Wire, 0x68);

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	#include "Wire.h"
#endif

MPU6050 mpu;

#define INTERRUPT_PIN 2

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//PID controller gains					
#define Gain_Proportional	700000		
#define  Gain_Integral		0		
#define  Gain_Derivative	40000
#define  Gain_Rotor_Speed	-.00	

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


volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
	mpuInterrupt = true;
}


// The setup function runs once when you press reset or power the board
void setup() {
	Wire.begin();

	//Initialise serial For debug use
	Serial.begin(115200);
	Serial.setTimeout(2);
	while (!Serial) {
		Serial.println("Waiting for serial, how can this be?");
	}

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin();
	Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
	Fastwire::setup(400, true);
#endif
	
	Serial.println(F("Initializing I2C devices..."));
	mpu.initialize();
	pinMode(INTERRUPT_PIN, INPUT);

	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	//[007999, 008000] -- > [-00169, 000000][007999, 008000] -- > [-00191, 000000][-08000, -08000] -- > [000000, 017731][000161, 000162] -- > [000000, 000003][-00063, -00062] -- > [-00376, 000111][-00126, -00125] -- > [-00191, 000046]

	// supply your own gyro offsets here, scaled for min sensitivity
	mpu.setXGyroOffset(161);
	mpu.setYGyroOffset(-63);
	mpu.setZGyroOffset(-13);
	//mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

	if (devStatus == 0) {
		// turn on the DMP, now that it's ready
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true);

		// enable Arduino interrupt detection
		Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
		Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
		Serial.println(F(")..."));
		attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();

		// set our DMP Ready flag so the main loop() function knows it's okay to use it
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true;

		// get expected DMP packet size for later comparison
		packetSize = mpu.dmpGetFIFOPacketSize();
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}

	Serial.println("Set up successful");
}

// The loop function runs repeatedly until power down or reset
void loop() {
	get_angle();

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
			AngleProportional = 0;
			AngleIntegral = 0;
			AngleDerivative = 0;
			Serial.println(SetPointAngle);
			Serial.println("PID values cleared");
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
		Serial.println(Angle_0,5);
		break;

	case 'c':
		SetPointAngle = Angle_0;
		break;

	case 'g':
		pid_controller();
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
	int16_t SetPoint = constrain(Speed, int16_tMin+1, int16_tMax-1); //-32, 768 .. 32, 767
	myArray[0] = (SetPoint >> 8) & 0xFF;
	myArray[1] = SetPoint & 0xFF;

	Wire.beginTransmission(9); // transmit to device #9
	Wire.write(myArray, 2);              // sends 
	Wire.endTransmission();    // stop transmitting
}

void pid_controller() {
	float Error = Angle_0 - SetPointAngle;

	//Stops motor if device becomes unstable and falls over: (+-) 0.174533 rad = 10 degrees
	if (abs(Error) >= 0.174533) {
		Serial.print("Error: Angle out of bounds ");
		switchByte = 's';
	}

	AngleProportional = Gain_Proportional * Error;
	
	AngleIntegral =	AngleIntegral + Gain_Integral * Error * (Angle_0_Time - Angle_minus1_Time) / 1000000;


	AngleDerivative = Gain_Derivative * (Angle_0 		* (.08333) +
										Angle_minus1	* (.05952) +
										Angle_minus2	* (.03571) +
										Angle_minus3	* (.01190) +
										Angle_minus4	* (-.01190) +
										Angle_minus5	* (-.03571) +
										Angle_minus6	* (-.05952) +
										Angle_minus7	* (-.08333)) / ((Angle_0_Time - Angle_minus7_Time)/(1000000*7));

	SetPointAngle = SetPointAngle - 0.001 * Error;


	if (PIDOutput > 0) {
		PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput + 100;
	}
	else {
		PIDOutput = AngleProportional + AngleIntegral + AngleDerivative + Gain_Rotor_Speed * PIDOutput - 100;
	}

	
	


	Serial.print(micros());
	Serial.print(',');
	Serial.print(SetPointAngle,4);
	Serial.print(',');
	Serial.println(Angle_0,4);
	SendSpeed(PIDOutput);
}

void get_angle() {
	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		if (mpuInterrupt && fifoCount < packetSize) {
			// try to get out of the infinite loop 
			fifoCount = mpu.getFIFOCount();
		}
		// Other intermediate code can go here 
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		fifoCount = mpu.getFIFOCount();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

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

		// Get Euler angles in radians
		Angle_0_Time = micros();
		mpu.dmpGetQuaternion(&q, fifoBuffer);
		mpu.dmpGetEuler(euler, &q);
		
		Angle_0 = -euler[2];
		
		float W_a = 0.99;
		
		Angle_0 = W_a * Angle_0 + (1 - W_a) * Angle_minus1;

		
		
	}

}
	


