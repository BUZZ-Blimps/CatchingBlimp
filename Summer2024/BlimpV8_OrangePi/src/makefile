main: main.cpp
	g++ $(pkg-config --cflags eigen3) accelGCorrection.cpp BangBang.cpp baro_acc_kf.cpp BerryIMU_v3.cpp brushless.cpp EMAFilter.cpp Gimbal.cpp gyro_ekf.cpp Kalman_Filter_Tran_Vel_Est.cpp Madgwick_Filter.cpp MotorControl.cpp optical_ekf.cpp Optical_Flow.cpp PID.cpp servo.cpp ServoWrapper.cpp tripleBallGrabber.cpp main.cpp -lwiringPi -lm -o test
