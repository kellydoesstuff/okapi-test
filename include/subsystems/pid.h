#include "main.h"

namespace pid {
    void resetEncoders();
    void resetTimers();
    double avgEncoder();
    void stop();
    double slew(double target_speed, double step, double prev_speed);
    void drivePD(int setpoint, int step, double kP, double kD);
    void drivePD(int setpoint);
    void drivePD(int setpoint, int step);
}
