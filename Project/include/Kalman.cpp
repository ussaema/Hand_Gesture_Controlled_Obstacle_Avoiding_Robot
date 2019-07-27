/*
 * Kalman.cpp
 *
 * Created: 09.07.2019 18:28:32
 *  Author: ussaema
 */ 
#include "Kalman.h"

Kalman::Kalman(float r, float q, float a, float b, float c) {
	R = r;
	Q = q;
	A = a;
	B = b;
	C = c;
	init = false;
}

float Kalman::filter(float z, float u) {
	if (!init) {
		x = 1 / C * z;
		cov = Q / (C * C);
		init = true;
	}

	else {
		float pred = predict(u);
		float p_cov = uncertainty();

		float K = p_cov * C / (C * C * p_cov + Q);

		x = pred + K * (z - C * pred);
		cov = p_cov - K * C * p_cov;

	}

	return x;

}

float Kalman::predict(float u) {
	return A * x + B * u;
}

float Kalman::uncertainty() {
	return A * A * cov + R;
}