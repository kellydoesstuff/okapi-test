#include "main.h"
#include <iostream>

Timer small_exit_time;
Timer big_exit_time;
Timer velocity_time;


/*----NOTE----
Everything PID is written in mV, not voltage cause I got syntax messed up lol (why kP is so outrageously high).
In the future instead of motor.moveVoltage() which is in mV, use motor.move(), which is in volts from a range 
of -127 to 127.
-------------*/

namespace pid {

    Motor LF(-11), LB(-1), RF(20), RB(10);

    MotorGroup left({LF,LB});
    MotorGroup right({RF,RB});
    
    void resetEncoders () {
        left.tarePosition();
        right.tarePosition();
    }

    void resetTimers() {
        small_exit_time.clearHardMark();
        big_exit_time.clearHardMark();
        velocity_time.clearHardMark();
    }

    double avgEncoder() {
        return (left.getPosition() + right.getPosition())/2;
    }

    void stop() {
        left.moveVelocity(0);
        right.moveVelocity(0);
    }


    //--EVERYTHING POSITIONAL PD--//

    double slew(double target_speed, double step, double prev_speed) {
        // note: any step under 700 is pretty unstable
        if (target_speed > prev_speed + step) {
            prev_speed += step;
        } else if (target_speed < prev_speed - step) {
            prev_speed -= step;
        } else { 
            prev_speed = target_speed;
        }
        
        return prev_speed;
    }

    void drivePD(int setpoint, int step, double kP, double kD) {

        resetEncoders();

        // variables
        bool startPID {true};
        double error;
        double derivative;
        double prev_error{0.0};
        double power;
        
        // slew
        double prev_power{0};
        int powercap {11000}; // max mV is 12,000

        // exit conditions
        int small_error{5};
        int big_error{10};

        while (startPID) {
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

            // debug stuff
            pros::lcd::print(0, "encoder value >> %5.2f", avgEncoder());
            pros::lcd::print(1, "error >> %5.2f", error);
            pros::lcd::print(2, "power >> %5.2f", power);
            
            // exit conditions
            // if robot gets close to target with a acceptable error range, make sure it's there for a short amnt of time
            if (abs(error) < small_error) {
                small_exit_time.placeHardMark();
                big_exit_time.clearHardMark();
                if (small_exit_time.getDtFromHardMark() > 1_s) {
                    pros::lcd::print(3, "in small error");
                    startPID = false;
                }
            }

            // if robot is close to target, start timer. if robot doesn't get closer within certian time, exit. 
            // doesn't run while small exit runs
            if (abs(error) < big_error) {
                big_exit_time.placeHardMark();
                if (big_exit_time.getDtFromHardMark() > 1.2_s) {
                    startPID = false;
                }
            }

            // if motor velocity is 0, exit
            if (abs(derivative) <= 0.05) {
                velocity_time.placeHardMark();
                if (velocity_time.getDtFromHardMark() > 1_s) {
                    startPID = false;
                }
            }

            pros::delay(10); // don't hog cpu
        }
        pros::lcd::clear();
        pros::lcd::print(3, "Exited");
        resetTimers();
        stop();
    }

    void drivePD(int setpoint) {
        drivePD(setpoint, 4500, 31.425, 4.9); // default constants w/ default step
    }

    void drivePD(int setpoint, int step) {
        drivePD(setpoint, step, 31.425, 4.9); // default constants w/ custom step
    } 

}