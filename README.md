#  Hand-Gesture-Controlled Obstacle Avoiding Robot with Haptic Feedback
Implemented in C/C++ using Atmel Studio 7.0, without any dependencies. Only the library AVR of the microcontroller needs to be included.
## Project description
The objective of this project is to develop an ”Obstacle Avoiding Robot (OAR)” that moves around a room and avoids obstacles when it encounters them. The moving in the environment will not be automated, but rather controlled by a ”Smart Hand (SH)” that captures basic gestures of the human hand. The detection of the obstacle will be provided to the user as haptic vibrations with pulses indicating how near is the obstacle. This feedback allows the human to avoid this obstacle by changing the moving direction of the OAR. Since the sensors used for the measuring the angles of the gesture and the sensors used for measuring the distances are noisy, a filtering and a sensor fusion is performed. Experiments and results show that the Kalman Filter and the Complementary Filter provides
better estimations depending on the sensor. [Read report here](https://github.com/ussaema/Hand_Gesture_Controlled_Obstacle_Avoiding_Robot/blob/master/Documentation/Report.pdf)

![alt text](https://github.com/ussaema/Hand_Gesture_Controlled_Obstacle_Avoiding_Robot/blob/master/Documentation/overview.jpg?raw=true)

## Obstacle Avoiding Robot: Hardware

![alt text](https://github.com/ussaema/Hand_Gesture_Controlled_Obstacle_Avoiding_Robot/blob/master/Documentation/Hardware_OAR.jpg?raw=true)

## Smart Hand: Hardware

![alt text](https://github.com/ussaema/Hand_Gesture_Controlled_Obstacle_Avoiding_Robot/blob/master/Documentation/Hardware_SH.jpg?raw=true)
