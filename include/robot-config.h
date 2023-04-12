#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/imu.h"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"


extern pros::Motor left_front_motor; // port 1, not reversed
extern pros::Motor left_back_motor; // port 2, not reversed
extern pros::Motor left_top_motor; // port 4, reversed
extern pros::Motor right_front_motor; // port 3, reversed
extern pros::Motor right_back_motor; // port 4, reversed
extern pros::Motor right_top_motor; // port 4, reversed
 
// drivetrain motor groups
extern pros::MotorGroup left_side_motors;
extern pros::MotorGroup right_side_motors;
 
 
// left tracking wheel encoder
extern pros::ADIEncoder encoderLeft;
// back tracking wheel encoder
extern pros::ADIEncoder encoderBack;
 
// inertial sensor
extern pros::Imu imu;

extern pros::Controller master;
