
#include <math.h>
#include <MPU9250.h>

//Defining OUTPUT pins
#define BPin 8
#define APin 7
#define pwmPin 3 // pin 3 has pwm frequency of 489Hz , pin 5 is 980Hz

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status; //used by MPU9250 for error checking

//controls loop switch
int switchByte;

long TimePrev = 0;
float AccelX;
float AccelZ;
float AccelAngle;
float GyroY;
float GyroYCal;
float GyroAngle;
float Angle;

float speed = 0;
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
	Serial.println("Set up Successful");
}

// the loop function runs over and over again until power down or reset
void loop() {
	
	if (Serial.available() > 0) {
		switchByte = Serial.read();
		Serial.println((char)switchByte);
	}
	delay(1);
	Serial.print(micros());
	Serial.print(' ');
	Serial.println(analogRead(A0));
	switch (switchByte)
	{
	case 'g':
		get_angle();
		//Serial.print(speed, 16);
		//Serial.print(' ');
		//Serial.println(Angle, 16);

		if ((millis() % 10 == 0))
		{
			
			speed = 0.5;
			set_speed(speed);
		}

		break;
	case 's':
		speed = 0;
		set_speed(speed);
		break;
	case 'f':
		set_speed(-.5);
		break;
	default:
		break;
	}

}

void set_speed(float Speed) {
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
	// read the sensor
	IMU.readSensor();

	// Normalise accelerometer values in accordance with gravity, important as these values will be combined using 'atan2f' to increase the angles accuracy
	AccelX = constrain(IMU.getAccelX_mss(), -9.81, 9.81) / 9.81;
	AccelZ = constrain(IMU.getAccelZ_mss(), -9.81, 9.81) / 9.81;

	// Calculate (accel) angle in x-z plane using normalised x and z accelerometer values
	AccelAngle = atan2f((float)AccelZ, (float)AccelX);

	// Correct gyro data using calibration value from setup
	GyroY = IMU.getGyroY_rads() - GyroYCal;


	long TimeNow = micros();
	long TimeDelta = TimeNow - TimePrev;

	if (TimeDelta > 200000) {
		GyroAngle = AccelAngle;
	}
	else {
		// Calculate (gyro) angle using Y axis gyro values
		GyroAngle = Angle + GyroY * (TimeNow - TimePrev) / 1000000;
	}

	//Set previous time to be current time, used if/ when this function is run again
	TimePrev = TimeNow;


	Angle = 0.98 * GyroAngle + 0.02 * AccelAngle;
}

