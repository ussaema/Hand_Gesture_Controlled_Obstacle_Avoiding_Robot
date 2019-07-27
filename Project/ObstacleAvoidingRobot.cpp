/*
 * Project.cpp
 *
 * Project: TUM ICS - Humanoid Sensor and Actuators 
 * Author : Oussema Dhaouadi
 */ 
#include "MCU.h"
#include "Infrared.h"
#include "Ultrasonic.h"
#include "Motor.h"
#include "Servo.h"
#include "RF24.h"
#include "UART.h"
#if DEVICE == 1 

int main(void)
{	
	// --- Initilizations
	// mcu
	mcu.init();
	// uart
	uart.Init(UART_BAUD_RATE);
	// radio transmitter
	RF24 radio(PIN_R_RF_CE,PIN_R_RF_CS);
	uint8_t addresses[][6] = {"Smart", "Robot"};
	radio.begin();
	radio.openWritingPipe(addresses[1]);
	radio.openReadingPipe(1, addresses[0]);
	radio.startListening();
	bool tx_buffer[1] = {true};
	int8_t rx_buffer[2];
	// infrared sensors
	Infrared ir_left(PIN_R_IR_LEFT);
	Infrared ir_right(PIN_R_IR_RIGHT);
	Infrared ir_back(PIN_R_IR_BACK);
	// ultrasonic sensor
	Ultrasonic us(PIN_R_US_TRIG, PIN_R_US_ECHO);
	// motors
	Motor motors_left(PIN_R_M_CTRL_LEFT, PIN_R_M_FORWARD_LEFT, PIN_R_M_BACKWARD_LEFT);
	Motor motors_right(PIN_R_M_CTRL_RIGHT, PIN_R_M_FORWARD_RIGHT, PIN_R_M_BACKWARD_RIGHT);
	uint8_t motors_left_velocity, motors_right_velocity = 0;
	// servo
	servo.init();
	servo.write(100);
	
	// --- Configurations
	// uart
	// infrared sensors
	/*ir_left.enableKalman();
	ir_right.enableKalman();
	ir_back.enableKalman();*/
	// ultrasonic sensor
	us.enableKalman();
	// motors
	motors_left.setSpeed(0);
	motors_right.setSpeed(0);
	
	bool obstacle_front = false;
	bool obstacle_left = false;
	bool obstacle_right = false;
	bool obstacle_back = false;
	for (;;) {
		//servo.write(50+pos);
		//for (pos = 0; pos <= 120; pos ++) { servo.write(50+pos); _delay_ms(10); }
		//for (pos = 0; pos <= 120; pos ++) { servo.write(50+120-pos); _delay_ms(10); }
			//uart.Write(us.read()); uart.Write(","); uart.Write(us.read_raw()); uart.Write("\r\n");
	/// *** Wireless Duplex channel ***//
		// read distance sensors and transmit data is obstacle detected
		uart.Write(ir_back.read()); uart.Write("\r\n");
		obstacle_front = us.read() < 30; obstacle_left = ir_left.read() < 140; obstacle_right = ir_right.read() < 70; obstacle_back = ir_back.read() < 155;		
		uart.Write("obstacle_front:"); uart.Write( us.read()); uart.Write("\r\n");
		if (obstacle_front ) { // || obstacle_left || obstacle_right || obstacle_back) {
			radio.stopListening();
			uart.Write("detected"); uart.Write("\r\n");
			radio.write(&tx_buffer, sizeof(tx_buffer));
			radio.startListening();
			_delay_ms(10);
		};
		// receive data from the robot if available
		if (radio.available()){
			radio.read(&rx_buffer, sizeof(rx_buffer));
			if (rx_buffer[0] && rx_buffer[1]){
				uart.Write("rx_buffer[0]"); uart.Write(rx_buffer[0]);  uart.Write("\r\n");
				uart.Write("rx_buffer[1]"); uart.Write(rx_buffer[1]);  uart.Write("\r\n");
			}
			if (rx_buffer[0] < -20){
				motors_left_velocity = (uint8_t)linear_mapping(rx_buffer[0], 20, 80, 0, MAX_VELOCITY);
				motors_right_velocity = (uint8_t)linear_mapping(rx_buffer[0], 20, 80, 0, MAX_VELOCITY);
				motors_left.setSpeed(motors_left_velocity); motors_right.setSpeed(motors_right_velocity);
				uart.Write("forward"); uart.Write("\r\n"); // left
				motors_left.turnForward(); motors_right.turnForward();
			} else if (rx_buffer[0] > 20){
				motors_left_velocity = (uint8_t)linear_mapping(rx_buffer[0], -20, -80, 0, MAX_VELOCITY);
				motors_right_velocity = (uint8_t)linear_mapping(rx_buffer[0], -20, -80, 0, MAX_VELOCITY);
				motors_left.setSpeed(motors_left_velocity); motors_right.setSpeed(motors_right_velocity);
				motors_left.turnBackward(); motors_right.turnBackward();
				uart.Write("backward"); uart.Write("\r\n");
			} else if (rx_buffer[1] > 20){
				motors_left_velocity = (uint8_t)linear_mapping(rx_buffer[1], -20, -80, 0, MAX_VELOCITY);
				motors_right_velocity = 0;
				motors_left.setSpeed(motors_left_velocity); motors_right.setSpeed(motors_right_velocity);
				motors_left.turnForward(); motors_right.turnForward();
				uart.Write("right"); uart.Write("\r\n");
			} else if (rx_buffer[1] < -20){
				motors_left_velocity = 0;
				motors_right_velocity = (uint8_t)linear_mapping(rx_buffer[1], -20, -80, 0, MAX_VELOCITY);
				motors_left.setSpeed(motors_left_velocity); motors_right.setSpeed(motors_right_velocity);
				motors_left.turnForward(); motors_right.turnForward();
				uart.Write("left"); uart.Write("\r\n");
			} else if ( rx_buffer[0] != 0 && rx_buffer[1] != 0) {
				motors_left_velocity = 0;
				motors_right_velocity = 0;
				motors_left.setSpeed(motors_left_velocity); motors_right.setSpeed(motors_right_velocity);
			}
			
			
		} else {
		}
	}
	return 0;
}

#endif