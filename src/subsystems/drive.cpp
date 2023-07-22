#include "main.h"
#include <iostream>

//initialize motors (this would also be where you initialize motors/sensors for other subsystems)
Motor LF(-11), LB(-1), RF(20), RB(10);

MotorGroup left({LF,LB});
MotorGroup right({RF,RB});


Timer timer;
Timer timeout_timer;

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
        return (left.getPosition() + right.getPosition())/2;
    }

    void stop() {
        left.moveVelocity(0);
        right.moveVelocity(0);
    }

    double slew(double target_speed, double step, double prev_speed) {
        if (target_speed > prev_speed + step){
            prev_speed += step;
        } else if (target_speed < prev_speed - step){
            prev_speed -= step;
        } else {
            prev_speed = target_speed;
        }

        return prev_speed;
    }

    void drivePID(int setpoint, double kP, double kI, double kD) {

        resetEncoders();
        
        // timer stuff
        timeout_timer.placeHardMark(); //record when function was called
        QTime timeout {5_s}; // variable with the type QTime, represents the amount of time needed for loop to break

        // variables
        bool startPID {true};
        double error;
        double derivative;
        double prev_error{0.0};
        double power;
        // slew
        double prev_power{0};
        int step{50};
        int powercap {11000}; // max mV is 12,000
        int count {0};
        // int direction {abs(setpoint) / setpoint};
        // setpoint = abs(setpoint);

        while (abs(error) > 6 || count == 0) {
            count = 1;
            error = setpoint - avgEncoder();
            derivative = error - prev_error;
            
            power = (error * kP + derivative * kD);
            
            if (power >= powercap) {
                power = powercap;
            } else if (power <= -powercap) {
                power = -powercap;
            }

            prev_error = error;
            
            power = slew(power,step,prev_power);
            
            left.moveVoltage(power);
            right.moveVoltage(power);

            prev_power = power;

            pros::lcd::print(0, "encoder value >> %5.2f", avgEncoder());
            pros::lcd::print(1, "error >> %5.2f", error);
            pros::lcd::print(2, "power >> %5.2f", power);

            // exit conditions...no good needs to be edited
            // if (abs(derivative) <= 6)
            //     timer.placeHardMark();
            // if (avgEncoder() > setpoint) { // overshot
            //     timer.placeHardMark();
            // }
            // if (timer.getDtFromHardMark() > 2_s) {
            //     startPID = false;
            // }
            // if (timeout_timer.getDtFromHardMark() > timeout) { // if it stays for too long
            //     startPID = false;
            // }

            pros::delay(10);
        }
        pros::lcd::clear();
        pros::lcd::print(3, "Exited");
        timer.clearHardMark();
        timeout_timer.clearHardMark();
        stop();
    }

    void drivePID(int setpoint) {
        drivePID(setpoint, 31.425, 0, 4.9); // default constants
    }
}

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