#include "main.h"

namespace auton {
    
    // 4 inch wheels, 12.5663706144 per 900 motor ticks (one full rotation of motor)
    // distance = circumference * gear ratio * rotation
   
    double calculate (double x) { // converts inches to degrees
        double rotations{x/12.57};
        int degrees {rotations*360};
        return degrees;
    }
    
    void nothing () {
        pid::drivePID(calculate(-24.0));
    }

}