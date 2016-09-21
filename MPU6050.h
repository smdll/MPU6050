#ifndef _MPU6050_H
#define _MPU6050_H

#include <Wire.h>
#include <Arduino.h>
#include <inttypes.h>

class MPU6050
{
public:
	void init();
	void getAngle(float *X, float *Y);

	float compAngleX, compAngleY; // Calculated angle using a complementary filter

private:
	float accX, accY, accZ;
	float gyroX, gyroY, gyroZ;

	uint32_t timer;
	uint8_t i2cData[14]; // Buffer for I2C data
	const uint8_t IMUAddress = 0x68; // AD0 is logic low on the PCB
	const uint16_t I2C_TIMEOUT = 1000; // Used to check for errors in I2C communication

	uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop)
	{
		Wire.beginTransmission(IMUAddress);
		Wire.write(registerAddress);
		Wire.write(data, length);
		uint8_t rcode = Wire.endTransmission(sendStop); // Returns 0 on success
		return rcode;
	};

	uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop)
	{
		return i2cWrite(registerAddress, &data, 1, sendStop); // Returns 0 on success
	};

	uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes)
	{
		uint32_t timeOutTimer;
		Wire.beginTransmission(IMUAddress);
		Wire.write(registerAddress);
		uint8_t rcode = Wire.endTransmission(false); // Don't release the bus
		if (rcode)
			return rcode;
	
		Wire.requestFrom(IMUAddress, nbytes, (uint8_t)true); // Send a repeated start and then release the bus after reading
		for (uint8_t i = 0; i < nbytes; i++)
		{
			if (Wire.available())
				data[i] = Wire.read();
			else
			{
				timeOutTimer = micros();
				while (((micros() - timeOutTimer) < I2C_TIMEOUT) && !Wire.available());
					if (Wire.available())
						data[i] = Wire.read();
					else
						return 5; // This error value is not already taken by endTransmission
			}
		}
		return 0; // Success
	};

};

#endif
