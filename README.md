# KalmanFilterExample
## Project
This is an example on how to use a Kalman Filter to estimate missing sensor axis. It shows the basic design and usage of a Kalman Filter. On the IMU the measurement unit to measure angular speed around the y-axis is broken, it was replaced by an estimate generated from the data of a magnetometer.
## Hardware
An Arduino nano V3.3 is used for processing. The magnetometer is a QMC5883L and as IMU, the MPU6050 is used.
## Software
For simulation, Octave is used since it is an open source alternative to MatLab.
For programming the Arduino, the Arduino IDE is used. If wished, I could do a C++ only with i.e. Makefile. Also other microcontrollers can be targeted if wished. If you're interested, please open an issue and let me know.
## Simulation Results
The Kalman Filter was designed with the help of the datasheets of the sensors. To test and verify it, a test signal is used.
![Signal Curves](Images/SigOverview.png "Waveforms")
As can be seen, the filter seems to work properly. The noise is reduced successfully. In the beginning, a transient can be seen, because the chosen starting vector for angle and angular speed (the signal and it's derivative respectively) didn't match the initial value of the testsignal. Otherwise the signal is filtered nicely. When it comes to the change in Kalman Gain, the difference between the previous and the current gain is used as a metric. As can be seen, it converges to zero as expected.
![Kalman Gain](Images/KGainDiff.png "Difference in Kalman Gains")
This means, the process of recalculating the Kalman Gain in every iteration of the microcontroller can be neglected and the final Kalman Gain can be used directly. Lastly, the angular velocity (derivative of the signal) needs to be invesitgated:
![Derivative](Images/Derivative.png "Angular velocity")
Also the initial transient can be observed. What is also worth noting is the last value. Since the test signal is cosine shaped, the last value is expected to be zero. The reason for this to be incorrect is the filter delay. The filter needs some time to calculate the estimated value and is therefore delayed by some samples. Because of that the derivative didn't reach zero by the end of the test signal.
## Implementation