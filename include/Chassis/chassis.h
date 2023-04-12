#include "pros/motors.hpp"
class Chassis {
  public:
    //DRIVE HELPER FUNCTIONS
    static void setVel(double Right, double Left);
    static void setVolt(double Right, double Left);
    static void setStop(pros::motor_brake_mode_e mode);

    //DRIVER CONTROL FUNCTIONS
    static void robotCentric();
};