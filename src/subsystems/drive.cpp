#include "main.h"
#include <iostream>

//initialize motors (this would also be where you initialize motors/sensors for other subsystems)
Motor LF(-11), LB(-1), RF(20), RB(10);

MotorGroup left({LF,LB});
MotorGroup right({RF,RB});

// chassis controller - lets us drive the robot around with open- or closed-loop control
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
       
        left.setEncoderUnits(AbstractMotor::encoderUnits::degrees);
        right.setEncoderUnits(AbstractMotor::encoderUnits::degrees);

        pid::resetEncoders();
    }

    void opcontrol() {
        Controller master;

        //set tank drive
        chassis->getModel()->tank(master.getAnalog(ControllerAnalog::rightY),
                                  master.getAnalog(ControllerAnalog::leftX));
    }

    void stop() {
        left.moveVelocity(0);
        right.moveVelocity(0);
    }

    void drivemV(double power) {
        left.moveVoltage(power);
        right.moveVoltage(power);
    }

}