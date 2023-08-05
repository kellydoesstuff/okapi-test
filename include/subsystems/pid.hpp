#include "main.h"

namespace pid {
    void resetEncoders();
    void resetTimers();
    double avgEncoder();
    double slew(double target_speed, double step, double prev_speed);
    double calculatePD(double kP, double kD, double error, double prev_error, double setpoint, double encoders);
    void drivePD(int setpoint, int step, double kP, double kD);
    void drivePD(int setpoint);
    void drivePD(int setpoint, int step);
}
