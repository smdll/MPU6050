#include "MPU6050.h"

void MPU6050::init()
{
	Wire.begin();
#if ARDUINO >= 157
	Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
	TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif
	i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
	i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
	while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
	while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
	while (i2cRead(0x75, i2cData, 1));
	if (i2cData[0] != 0x68) // Read "WHO_AM_I" register
		while (1);

	delay(100); // Wait for sensor to stabilize

	while (i2cRead(0x3B, i2cData, 6));
	accX = (i2cData[0] << 8) | i2cData[1];
	accY = (i2cData[2] << 8) | i2cData[3];
	accZ = (i2cData[4] << 8) | i2cData[5];

	float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	float roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

	compAngleX = roll;
	compAngleY = pitch;
	timer = micros();
}

void MPU6050::getAngle(float *X, float *Y)
{
	while (i2cRead(0x3B, i2cData, 14));
	accX = ((i2cData[0] << 8) | i2cData[1]);
	accY = ((i2cData[2] << 8) | i2cData[3]);
	accZ = ((i2cData[4] << 8) | i2cData[5]);
	gyroX = (i2cData[8] << 8) | i2cData[9];
	gyroY = (i2cData[10] << 8) | i2cData[11];
	gyroZ = (i2cData[12] << 8) | i2cData[13];

	float dt = (float)(micros() - timer) / 1000000; // Calculate delta time
	timer = micros();

	float pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
	float roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;

	float gyroXrate = gyroX / 131.0; // Convert to deg/s
	float gyroYrate = gyroY / 131.0; // Convert to deg/s

	if ((roll < -90) || (roll > 90))
		compAngleX = roll;
	if ((pitch < -90) || (pitch > 90))
		compAngleY = pitch;

	compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
	compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

	*X = compAngleX;
	*Y = compAngleY;
}
