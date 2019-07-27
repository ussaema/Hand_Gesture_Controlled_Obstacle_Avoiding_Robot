/*
 * Kalman.h
 *
 * Created: 09.07.2019 18:28:09
 *  Author: ussaema
 */ 


#ifndef KALMAN_H_
#define KALMAN_H_

class Kalman {
	public:
	float R, Q, A, B, C;
	float cov;
	float x; // Signal without noise
	bool init;

	Kalman(float r, float q, float a, float b, float c);
	
	// estimation
	float filter(float z, float u);
	// prediction
	float predict(float u);
	// observation's uncertainty
	float uncertainty();

};

#endif /* KALMAN_H_ */