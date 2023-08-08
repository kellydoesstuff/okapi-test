#include "main.h"

namespace drive {
    void init();
    void opcontrol();
    void stop();
    void drivemV(double power);
    void drivemV(double left_power, double right_power);
}