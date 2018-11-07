/*
 Name:		FirstDraft.ino
 Created:	10/26/2018 4:15:40 PM
 Author:	Jonathan
*/

#include "libraries\MPU9250\MPU9250.h"
#include "libraries\quaternionFileters\quaternionFilters.h"
#include <math.h>
#include <Wire.h>


#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 2;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed = 13;  // Set up pin 13 led for toggling
int vibPin = 9;  // Vibration motor pin. 

MPU9250 myIMU;

void setup() {

	Wire.begin();
	// TWBR = 12;  // 400 kbit/sec I2C speed
	if (SerialDebug) { Serial.begin(38400); }

	// Set up the interrupt pin, its set as active high, push-pull
	pinMode(intPin, INPUT);
	digitalWrite(intPin, LOW);
	pinMode(myLed, OUTPUT);
	digitalWrite(myLed, HIGH);
	pinMode(vibPin, OUTPUT);
	analogWrite(vibPin, 255);

	// Read the WHO_AM_I register, this is a good test of communication
	byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
	Serial.print("MPU9250 "); Serial.print("I AM "); Serial.print(c, HEX);
	Serial.print(" I should be "); Serial.println(0x71, HEX);

	if (c == 0x71) // WHO_AM_I should always be 0x68
	{
		Serial.println("MPU9250 is online...");

		// Start by performing self test... 
		myIMU.MPU9250SelfTest(myIMU.SelfTest);

		if (SerialDebug) // ...and reporting values
		{
			Serial.print("x-axis self test: acceleration trim within : ");
			Serial.print(myIMU.SelfTest[0], 1); Serial.println("% of factory value");
			Serial.print("y-axis self test: acceleration trim within : ");
			Serial.print(myIMU.SelfTest[1], 1); Serial.println("% of factory value");
			Serial.print("z-axis self test: acceleration trim within : ");
			Serial.print(myIMU.SelfTest[2], 1); Serial.println("% of factory value");
			Serial.print("x-axis self test: gyration trim within : ");
			Serial.print(myIMU.SelfTest[3], 1); Serial.println("% of factory value");
			Serial.print("y-axis self test: gyration trim within : ");
			Serial.print(myIMU.SelfTest[4], 1); Serial.println("% of factory value");
			Serial.print("z-axis self test: gyration trim within : ");
			Serial.print(myIMU.SelfTest[5], 1); Serial.println("% of factory value");
		}

		// Calibrate gyro and accelerometers, load biases in bias registers
		myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

		// Initialize device for active mode read of acclerometer, gyroscope, and
		// temperature
		myIMU.initMPU9250();
		if (SerialDebug) { Serial.println("MPU9250 initialized for active data mode...."); }

		// Read the WHO_AM_I register of the magnetometer, this is a good test of
		// communication
		byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
		if (SerialDebug) {
			Serial.print("AK8963 "); Serial.print("I AM "); Serial.print(d, HEX);
			Serial.print(" I should be "); Serial.println(0x48, HEX);
		}

		// Get magnetometer calibration from AK8963 ROM
		// Initialize device for active mode read of magnetometer
		myIMU.initAK8963(myIMU.magCalibration);

		if (SerialDebug) {
			Serial.println("AK8963 initialized for active data mode....");
			//  Serial.println("Calibration values: ");
			Serial.print("X-Axis sensitivity adjustment value ");
			Serial.println(myIMU.magCalibration[0], 2);
			Serial.print("Y-Axis sensitivity adjustment value ");
			Serial.println(myIMU.magCalibration[1], 2);
			Serial.print("Z-Axis sensitivity adjustment value ");
			Serial.println(myIMU.magCalibration[2], 2);
		}

		// User environmental x-axis correction in milliGauss, should be
		// automatically calculated
		myIMU.magbias[0] = +248.0;
		myIMU.magbias[1] = +200.0; // User environmental y-axis correction in milliGauss						 
		myIMU.magbias[2] = -173.0; // User environmental z-axis correction in milliGauss

	} // if (c == 0x71)
	else
	{
		Serial.print("Could not connect to MPU9250: 0x");
		Serial.println(c, HEX);
		//TODO Error handling
	}
	
	if (SerialDebug) { Serial.print("Setup Complete"); }

} // End of setup

void loop() {
	// If intPin goes high, all data registers have new data
	// On interrupt, check if data ready interrupt
	if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
	{
		myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
		myIMU.getAres();

		// Now we'll calculate the accleration value into actual g's
		// This depends on scale being set
		myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - accelBias[0];
		myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - accelBias[1];
		myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - accelBias[2];

		myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values
		myIMU.getGres();

		// Calculate the gyro value into actual degrees per second
		// This depends on scale being set
		myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
		myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
		myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

		myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values
		myIMU.getMres();
		// Calculate the magnetometer values in milliGauss
		// Include factory calibration per data sheet and user environmental
		// corrections
		// Get actual magnetometer value, this depends on scale being set
		myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes*myIMU.magCalibration[0] -
			myIMU.magbias[0];
		myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes*myIMU.magCalibration[1] -
			myIMU.magbias[1];
		myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes*myIMU.magCalibration[2] -
			myIMU.magbias[2];
	} // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

	  // Must be called before updating quaternions!
	myIMU.updateTime();

	// Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
	// the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
	// (+ up) of accelerometer and gyro! We have to make some allowance for this
	// orientationmismatch in feeding the output to the quaternion filter. For the
	// MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
	// along the x-axis just like in the LSM9DS0 sensor. This rotation can be
	// modified to allow any convenient orientation convention. This is ok by
	// aircraft orientation standards! Pass gyro rate as rad/s
	//  MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
	MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx*DEG_TO_RAD,
		myIMU.gy*DEG_TO_RAD, myIMU.gz*DEG_TO_RAD, myIMU.my,
		myIMU.mx, myIMU.mz, myIMU.deltat);

	// Serial print and/or display at 0.5 s rate independent of data rates
	myIMU.delt_t = millis() - myIMU.count;

	// Define output variables from updated quaternion---these are Tait-Bryan
	// angles, commonly used in aircraft orientation. In this coordinate system,
	// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
	// x-axis and Earth magnetic North (or true North if corrected for local
	// declination, looking down on the sensor positive yaw is counterclockwise.
	// Pitch is angle between sensor x-axis and Earth ground plane, toward the
	// Earth is positive, up toward the sky is negative. Roll is angle between
	// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
	// arise from the definition of the homogeneous rotation matrix constructed
	// from quaternions. Tait-Bryan angles as well as Euler angles are
	// non-commutative; that is, the get the correct orientation the rotations
	// must be applied in the correct order which for this configuration is yaw,
	// pitch, and then roll.
	// For more see
	// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
	// which has additional links.
	myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
		*(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1)
		- *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
	myIMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
		*(getQ() + 2)));
	myIMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
		*(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1)
		- *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
	myIMU.pitch *= RAD_TO_DEG;
	myIMU.yaw *= RAD_TO_DEG;
	// Declination of in Bristol as per:
	// http://www.ngdc.noaa.gov/geomag-web/#declination
	myIMU.yaw -= 1.4; // Correction for declination
	myIMU.roll *= RAD_TO_DEG;

	// update LCD once per half-second independent of read rate
	if (myIMU.delt_t > 500)
	{

		if (SerialDebug)
		{
			Serial.print("Yaw, Pitch, Roll: ");
			Serial.print(myIMU.yaw, 2);
			Serial.print(", ");
			Serial.print(myIMU.pitch, 2);
			Serial.print(", ");
			Serial.println(myIMU.roll, 2);

			Serial.print("rate = ");
			Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
			Serial.println(" Hz");
		}

		float heading = 0;
		if (1000 * myIMU.ay - 1000 < 0) { // Compensate for the belt being worn the wrong way up. 
			Serial.println("Right way up");
			heading = myIMU.yaw;
		}
		else {
			Serial.println("Wrong way up");
			heading = -myIMU.yaw;
		}

		if (heading < 0) { heading += 360; }
		Serial.print("Heading: "); Serial.print(heading); Serial.println(".");

		// Set vibration
		float angleAntiClock = heading;
		float angleClock = 360 - heading;
		float angle = 255;
		if (angleAntiClock < angleClock)
		{
			angle = angleAntiClock;
		}
		else {
			angle = angleClock;
		}

		Serial.print("angle: "); Serial.println(angle);

		float vib = 255.0;
		float deviation = 45;

		float exponent = -0.5 * pow(angle / deviation, 2);
		Serial.print("exponent: "); Serial.println(exponent);

		vib = 255 - 255 * exp(exponent);

		Serial.print("vib: "); Serial.println(vib);

		analogWrite(vibPin, vib);


		myIMU.count = millis();
		myIMU.sumCount = 0;
		myIMU.sum = 0;
	} // if (myIMU.delt_t > 500)
}

