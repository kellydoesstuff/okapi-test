#include "main.h"
#include <iostream>

int small_exit_time{0};
int big_exit_time{0};
int velocity_time{0};
double integral{0};


/*----NOTE----
Everything PID is written in mV, not voltage cause I got syntax messed up lol (why kP is so outrageously high).
In the future instead of motor.moveVoltage() which is in mV, use motor.move(), which is in volts from a range 
of -127 to 127.
-------------*/

namespace pid {

    Motor LF(-11), LB(-1), RF(20), RB(10);

    MotorGroup left({LF,LB});
    MotorGroup right({RF,RB});

    IMU inertial (5);

    void calibrateInertial() {
        int err = inertial.reset();
        pros::lcd::print(0, "inertial calibrate : reset (done) error: %d", err);
        int n {0};
        
        while (inertial.isCalibrating()) {
            pros::lcd::print(0, "inertial calibrate : is calibrating : %d", n+=10);

            pros::delay(20);
        }

        pros::lcd::print(0, "inertial calibrate : is calibrating (done)  ");

        pros::delay(500);
    }
    
    void resetEncoders () {
        left.tarePosition();
        right.tarePosition();
    }

    void resetTimers() {
        big_exit_time = 0;
        small_exit_time = 0;
        velocity_time = 0;
    }

    double avgEncoder() {
        return (left.getPosition() + right.getPosition())/2;
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

    double calculatePID(double kP, double kI,  double kD, double start_i, double* integral, double error, double prev_error) {
       double derivative{error-prev_error};
       
       if (kI != 0) { // if kI is active
        
        if (fabs(error) < start_i)  // add integral when in range of start_i
            *integral += error;

        if (util::sgn(error) != util::sgn(prev_error))  // prevent integral windup
            *integral = 0;
        
        return (error * kP + derivative * kD + *integral * kI);
       
       }
       
       return (error * kP + derivative * kD);
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

        // heading correction
        double heading_kP{1};
        double heading_kI{1};
        double heading_kD{1};
        int og_heading{inertial.get()};
        int heading_prev_error{0};
        int current_heading;
        int heading_error;
        int heading_power;
        
        // exit conditions
        int small_error{5};
        int big_error{10};

        while (startPID) {
            
            // drive pd
            error = setpoint - avgEncoder();
            power = calculatePID(kP, 0, kD, 0, 0, error, prev_error);
            prev_error = error;
            power = util::clip_num(power, powercap, -powercap);
            power = slew(power, step, prev_power);
            prev_power = power;
            
            // heading correction
            heading_error = og_heading - inertial.get();
            heading_power = calculatePID(heading_kP, heading_kI, heading_kD, 15.0, &integral, heading_error, heading_prev_error);
            drive::drivemV(power-heading_power, power+heading_power);

            // debug stuff
            pros::lcd::print(0, "encoder value >> %5.2f", avgEncoder());
            pros::lcd::print(1, "error >> %5.2f", error);
            pros::lcd::print(2, "power >> %5.2f", power);
            
            // exit conditions
            // if robot gets close to target with a acceptable error range, make sure it's there for a short amnt of time
            if (abs(error) < small_error) {
                small_exit_time += util::DELAY_TIME;
               big_exit_time = 0;
                if (small_exit_time > 1000) {
                    pros::lcd::print(3, "in small error");
                    startPID = false;
                }
            }

            // if robot is close to target, start timer. if robot doesn't get closer within certian time, exit. 
            // doesn't run while small exit runs
            if (abs(error) < big_error) {
                big_exit_time += util::DELAY_TIME;
                if (big_exit_time > 1200) {
                    startPID = false;
                }
            }

            // if motor velocity is 0, exit
            if (abs(derivative) <= 0.05) {
                velocity_time += util::DELAY_TIME;
                if (velocity_time > 1000) {
                    startPID = false;
                }
            }

            pros::delay(10); // don't hog cpu
        }
        pros::lcd::clear();
        pros::lcd::print(3, "Exited");
        resetTimers();
        drive::stop();
    }

    void drivePD(int setpoint) {
        drivePD(setpoint, 4500, 31.425, 4.9); // default constants w/ default step
    }

    void drivePD(int setpoint, int step) {
        drivePD(setpoint, step, 31.425, 4.9); // default constants w/ custom step
    } 

}