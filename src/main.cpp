#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep 
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/motors.hpp" // IWYU pragma: keep

//Autonomous paths
ASSET(Auto_txt);

//Toggles
bool trapdoorToggleState = false;
bool tongueToggleState = false;
bool ballstopToggleState = false;

//Controllers
pros::Controller controllerPrimary(pros::E_CONTROLLER_MASTER);
pros::Controller controllerSecondary(pros::E_CONTROLLER_PARTNER);

// Motor group
pros::MotorGroup leftDrive({-19, 20, -18}, pros::MotorGears::blue);
pros::MotorGroup rightDrive({14, -12, 13}, pros::MotorGears::blue);
pros::MotorGroup intake({-15, 16}, pros::v5::MotorGears::blue);

//Pneumatics
pros::adi::DigitalOut trapdoor('G');
pros::adi::DigitalOut ballstop('B');
pros::adi::DigitalOut tongue('H');

// Drivetrain settings
lemlib::Drivetrain drivetrain(&leftDrive, // left motor group
                              &rightDrive, // right motor group
                              13.944001 , // 10 inch track width
                              lemlib::Omniwheel::NEW_325, // using new 4" omnis
                              360, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

// imu
pros::Imu imu(11);
// vertical tracking wheel encoder
pros::Rotation verticalEncoder(-9);
// vertical tracking wheel
lemlib::TrackingWheel verticalTrackingWheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 3);

// odometry settings
lemlib::OdomSensors sensors(&verticalTrackingWheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateralController(20, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              2, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angularController(2.2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateralController, // lateral PID settings
                        angularController, // angular PID settings
                        sensors // odometry sensors
);

// initialize function. Runs on program startup
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    // print position to brain screen
    pros::Task screen_task([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // delay to save resources
            pros::delay(20);
        }
    });
}

void autonomous() {
    chassis.setPose(55.648, 17.604, 270);
    intake.move(127);
    chassis.moveToPose(55.648, 17.604, 270, 3000);
    chassis.moveToPose(24.068, 22.29, 270, 3000);
    chassis.moveToPoint(7.744, 23.05, 3000);
    chassis.turnToPoint(3.697, 37.9, 2000);
    chassis.moveToPose(3.697, 37.9, -30, 3000, {.maxSpeed = 60});
    chassis.moveToPose(23, 23, 225, 4000, {.forwards = false});
    chassis.moveToPose(15, 15, 225, 1500);
}

void opcontrol() {
    while (true) {
        // get left y and right x positions
        int leftY = controllerPrimary.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controllerPrimary.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // move the robot
        chassis.curvature(leftY, rightX);

        if (controllerPrimary.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   {
            intake.move(127);
        }
        else if (controllerSecondary.get_digital(pros::E_CONTROLLER_DIGITAL_R1))   {
            intake.move(127);
        }
        else if (controllerPrimary.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {
            intake.move(-127);
        }
        else if (controllerSecondary.get_digital(pros::E_CONTROLLER_DIGITAL_R2))   {
            intake.move(-127);
        }


        if (controllerPrimary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))   {
            if (trapdoorToggleState == false)   {
                trapdoor.set_value(true);
                tongue.set_value(false);
            }
            else {
                trapdoor.set_value(false);

            }
            trapdoorToggleState = !trapdoorToggleState;
        }
        if (controllerSecondary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1))   {
            if (trapdoorToggleState == false)   {
                trapdoor.set_value(true);
                tongue.set_value(false);
            }
            else {
                trapdoor.set_value(false);
                
            }
            trapdoorToggleState = !trapdoorToggleState;
        }
        if (controllerPrimary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))   {
            if (ballstopToggleState == false)   {
                ballstop.set_value(true);
            }
            else {
                ballstop.set_value(false);

            }
            ballstopToggleState = !ballstopToggleState;
        }
        if (controllerSecondary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2))   {
            if (ballstopToggleState == false)   {
                ballstop.set_value(true);
            }
            else {
                ballstop.set_value(false);

            }
            ballstopToggleState = !ballstopToggleState;
        }

        //Tongue
        if (controllerPrimary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (tongueToggleState == false) {
               tongue.set_value(true);
               trapdoor.set_value(false);
            }
            else {
                tongue.set_value(false);
            }
            tongueToggleState = !tongueToggleState;
        }
        if (controllerSecondary.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            if (tongueToggleState == false) {
               tongue.set_value(true);
            }
            else {
                tongue.set_value(false);
            }
            tongueToggleState = !tongueToggleState;
        }
        // delay to save resources
        pros::delay(25);
    }
}