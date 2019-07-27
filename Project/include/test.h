#include "config.h"

int MPU6050_write_reg(int reg, uint8_t data);
int MPU6050_write(int start, const uint8_t *pData, int size);
int MPU6050_read(int start, uint8_t *buffer, int size);
void calibrate_sensors() ;
int read_gyro_accel_vals(uint8_t* accel_t_gyro_ptr) ;
void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) ;