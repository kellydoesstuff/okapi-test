#include "main.h"
#include <iostream>

//initialize motors
Motor LF(-1), LB(-2), RF(3), RB(4);

// Chassis Controller - lets us drive the robot around with open- or closed-loop control
auto chassis = ChassisControllerBuilder()
    .withMotors (
        {-1,-2},
        {3,4}
    )
    // 200rpm inserts, 1 to 1 ratio
    .withDimensions({AbstractMotor::gearset::blue, 
    // 4 in wheel diameter, 11.5 wheel track 
    (1.0 / 1.0)}, {{4_in, 35.4_cm}, imev5BlueTPR})
    .build();


namespace drive {

    void init() {
        LF.setBrakeMode(AbstractMotor::brakeMode::coast);
        LB.setBrakeMode(AbstractMotor::brakeMode::coast);
        RF.setBrakeMode(AbstractMotor::brakeMode::coast);
        RB.setBrakeMode(AbstractMotor::brakeMode::coast);
    }

    void opcontrol() {
        Controller master;

        //set tank drive
        chassis->getModel()->tank(master.getAnalog(ControllerAnalog::rightY),
                                  master.getAnalog(ControllerAnalog::leftX));
    }

}