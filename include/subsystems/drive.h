#include "main.h"

namespace drive {
    void init();
    void opcontrol();
}

namespace pid {
    void resetEncoders();
    double avgEncoder();
    void stop();
    void drivePID();
}

namespace auton {
    void nothing();
}