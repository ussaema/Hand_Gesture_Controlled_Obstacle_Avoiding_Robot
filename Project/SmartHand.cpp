/*
 * Project.cpp
 *
 * Project: TUM ICS - Humanoid Sensor and Actuators 
 * Author : Oussema Dhaouadi
 */ 
#include "MCU.h"
#include "RF24.h"
#include "RGB_LED.h"
#include "IMU.h"
#include "VibrationMotor.h"


#if DEVICE == 0

int main(void)
{	
	// --- Initilizations
	// mcu
	mcu.init();
	// uart
	uart.Init(UART_BAUD_RATE);
	// imu
	IMU imu(ADDR_SH_IMU, PIN_SH_LED1);
	// calibration button
	mcu.getPinInput(PIN_SH_BTN);
	// Vibration motor
	VibrationMotor vm(PIN_SH_VM);
	// radio transmitter
	RF24 radio(PIN_SH_RF_CE,PIN_SH_RF_CS);
	uint8_t addresses[][6] = {"Smart", "Robot"};
	radio.begin();
	radio.openWritingPipe(addresses[0]);
	radio.openReadingPipe(1, addresses[1]);
	radio.startListening();
	int8_t tx_buffer[2];
	bool rx_buffer[1];
	// rgb leb
	//RGB_LED rgbLED(PIN_SH_LED_R, PIN_SH_LED_G, PIN_SH_LED_B, LED_INTENSITY_ON, LED_INTENSITY_OFF);
	mcu.setPinFunction(PIN_SH_VM, OUTPUT);
	mcu.setPinFunction(PIN_SH_LED2, OUTPUT);
	
	bool obstacle_detected = false;	
	for (;;) {
		
	// *** IMU ***//
		// get (data aquisation) the sensor data and process (complementary and Kalman filtering if activated) it
		// x is the pitch angle y is the roll angle
		imu.GetAndProcessData(false, 0.7);
		// update the rgb led according to the sensor data (for Debug purposes)
		// the color R is not connected to pwm pin, so it will be turned off
		// the color G will map the pitch angle values (the axis x)
		// the color B will map the pitch angle values (the axis y)
		tx_buffer[0] = (int16_t)imu.GetX(); tx_buffer[1] = (int16_t)imu.GetY();
		// rgb led
		//rgbLED.Set(0, linear_mapping(tx_buffer[0], -90, 90, 0, 1), linear_mapping(tx_buffer[1], -90, 90, 0, 1));
		if (1){
			// transmit led
			mcu.setPinLevel(PIN_SH_LED2, HIGH);
			// send through radio channel
			radio.stopListening();
			radio.write(&tx_buffer, sizeof(tx_buffer));
			uart.Write("tx_buffer0"); uart.Write(tx_buffer[0]); uart.Write("\r\n");
			uart.Write("tx_buffer1"); uart.Write(tx_buffer[1]); uart.Write("\r\n");
			radio.startListening();
			_delay_ms(10);
		} else {
			mcu.setPinLevel(PIN_SH_LED2, LOW);
		}
		// receive data from the robot if available
		if (radio.available()){
			radio.read(&rx_buffer, sizeof(rx_buffer));
			//uart.Write("on");
			vm.On();
		} else {
			//uart.Write("off");
			vm.Off();
		}
		_delay_ms(50);
	}
	return 0;
}

#endif