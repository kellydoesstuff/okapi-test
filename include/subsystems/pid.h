#include "main.h"

namespace pid {
    void resetEncoders();
    void resetTimers();
    double avgEncoder();
    void stop();
    double slew(double target_speed, double step, double prev_speed);
    void drivePID(int setpoint, double kP, double kI, double kD);
    void drivePID(int setpoint);
    void drivePID(int setpoint, int step);
}
