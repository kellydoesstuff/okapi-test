#include "main.h"

struct {
    int small_exit_time;
    int big_exit_time;
    int velocity_time;
} timer;

namespace pid {
    void resetEncoders();
    void resetTimers();
    double avgEncoder();
    double slew(double target_speed, double step, double prev_speed);
    double calculatePID(double kP, double kI, double kD, double start_i, double* integral, double error, double prev_error, double setpoint, double encoders);
    void drivePD(int setpoint, int step, double kP, double kD);
    void drivePD(int setpoint);
    void drivePD(int setpoint, int step);
}
