#include "main.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);


// drivetrain motors
pros::Motor left_back_motor(16,pros::E_MOTOR_GEAR_600, false); // port 1, not reversed
pros::Motor left_front_motor(14,pros::E_MOTOR_GEAR_600, true); // port 2, not reversed
pros::Motor left_top_motor(15,pros::E_MOTOR_GEAR_600, false); // port 4, reversed
pros::Motor right_front_motor(13,pros::E_MOTOR_GEAR_600, false); // port 3, reversed
pros::Motor right_back_motor(12,pros::E_MOTOR_GEAR_600, true); // port 4, reversed.
pros::Motor right_top_motor(11,pros::E_MOTOR_GEAR_600, true); // port 4, reversed
 
// drivetrain motor groups
pros::MotorGroup left_side_motors({left_front_motor, left_back_motor, left_top_motor});
pros::MotorGroup right_side_motors({right_front_motor, right_back_motor, right_top_motor});

// left tracking wheel encoder
pros::ADIEncoder encoderLeft('G', 'H', true); // ports A and B, reversed
// back tracking wheel encoder
pros::ADIEncoder encoderBack('E', 'F', false); // ports C and D, not reversed

pros::Imu imu(17); // port 2