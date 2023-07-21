#include "main.h"
#include <iostream>

//initialize motors (this would also be where you initialize motors/sensors for other subsystems)
Motor LF(-1), LB(-2), RF(3), RB(4);

MotorGroup left({LF,LB});
MotorGroup right({RF,RB});

Timer timer;
Timer timeout_timer;

// chassis Ccntroller - lets us drive the robot around with open- or closed-loop control
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

namespace pid {
    
    void resetEncoders () {
        left.tarePosition();
        right.tarePosition();
    }

    double avgEncoder() {
        return (fabs(left.getPosition()) + fabs(right.getPosition()))/2;
    }

    void stop() {
        left.moveVoltage(0);
        right.moveVoltage(0);
    }

    void drivePID(int setpoint, double kP, double kI, double kD) {
        
        timeout_timer.placeHardMark(); //record when function was called
        QTime timeout {5_s}; // variable with the type QTime, represents the amount of time needed for loop to break
        resetEncoders();
        bool startPID {true};
        
        double error;
        int power;
        double derivative;
        double prev_error{0.0};
        double power;
        int powercap {11000};

        int direction {abs(setpoint) / setpoint};
        int setpoint {abs(setpoint)};

        while (startPID) {
            
            error = setpoint - avgEncoder();
            derivative = error - prev_error;
            
            power = (error * kP + derivative * kD) * direction;
            
            if (direction * power >= powercap) {
                power = powercap;
            } else if (direction * power <= -powercap) {
                power = -powercap;
            }

            prev_error = error;

            pros::lcd::print(0, "encoder value >> %5.2f", avgEncoder());
            pros::lcd::print(1, "error >> %5.2f", error);
            pros::lcd::print(2, "power >> %5.2f", power);

            left.moveVoltage(power);
            right.moveVoltage(power);

            // exit conditions
            if (avgEncoder() > setpoint) { // overshot
                timer.placeHardMark();
            }
            if (timer.getDtFromHardMark() > 0.3_s) {
                startPID = false;
            }
            if (timeout_timer.getDtFromHardMark() > timeout) { // if it stays for too long
                startPID = false;
            }

            pros::delay(10);
        }

        timer.clearHardMark();
        timeout_timer.clearHardMark();
        stop();
    }
}