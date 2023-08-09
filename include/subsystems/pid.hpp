#include "main.h"


namespace pid {
    void calibrateInertial();
    void resetEncoders();
    void resetTimers();
    double avgEncoder();
    double slew(double target_speed, double step, double prev_speed);
    double calculatePID(double kP, double kI, double kD, double start_i, double* integral, double derivative, double error, double prev_error);
    void drivePD(int setpoint, int step, double kP, double kD);
    void drivePD(int setpoint);
    void drivePD(int setpoint, int step);
    void angularPD(double setpoint, double kP, double kD);
}