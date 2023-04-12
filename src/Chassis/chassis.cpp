#include "main.h"

//DRIVE HELPER FUNCTIONS
void Chassis::setVel(double Right, double Left){
  right_back_motor.move_velocity(Right);
  right_front_motor.move_velocity(Right);
  right_top_motor.move_velocity(Right);
  left_back_motor.move_velocity(Left);
  left_front_motor.move_velocity(Left);
  left_top_motor.move_velocity(Left);
}

void Chassis::setVolt(double Right, double Left){
  right_back_motor.move(Right);
  right_front_motor.move(Right);
  right_top_motor.move(Right);
  left_back_motor.move(Left);
  left_front_motor.move(Left);
  left_top_motor.move(Left);
}

void Chassis::setStop(pros::motor_brake_mode_e mode){
  right_back_motor.set_brake_mode(mode);
  right_front_motor.set_brake_mode(mode);
  right_top_motor.set_brake_mode(mode);
  left_back_motor.set_brake_mode(mode);
  left_front_motor.set_brake_mode(mode);
  left_top_motor.set_brake_mode(mode);
}

//DRIVER CONTROL FUNCTIONS
int PREVEL = 0;

void Chassis::robotCentric(){
  int controllerY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int controllerX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

  //Deadzone
  if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 5) {controllerY = 0;}
  if(abs(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 5) {controllerX = 0;}

  int LeftVel = -(controllerY + controllerX);
  int RightVel = -(controllerY - controllerX);


  int RVEL = 5* RightVel;
  int LVEL = 5* LeftVel;

  if (RVEL > 600) {
		RVEL = 600;
	} 
  else if (RVEL < -600){ 
      RVEL = -600;
  }

  if (LVEL > 600) {
		LVEL = 600;
	} 
  else if (LVEL < -600){
    LVEL = -600;
  }

  if (RVEL > PREVEL){
    pros::lcd::print(2,"%d",PREVEL);
  }
  
  setVel(RVEL,LVEL);
}