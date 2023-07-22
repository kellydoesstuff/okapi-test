#include "main.h"

namespace drive {
    void init();
    void opcontrol();
}

namespace pid {
    void resetEncoders();
    double avgEncoder();
    void stop();
    double slew(double target_speed, double step, double prev_speed);
    void drivePID(int setpoint, double kP, double kI, double kD);
    void drivePID(int setpoint);
}

namespace auton {
    double calculate(double x);
    void nothing();
}