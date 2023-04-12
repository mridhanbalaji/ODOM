#include "main.h"

//GLOBAL COORDINATES
Point Odom::globalPoint = {0, 0, 0};
Point Odom::prevGlobalPoint = {0, 0, 0};
Point Odom::globalDeltaPoint = {0, 0, 0};

//LOCAL COORDINATES
Point Odom::localDeltaPoint = {0, 0};

//SENSOR VALUES
//encoder
encoderType Odom::encoderVal = {0, 0, 0}; //leftEnc, rightEnc, backEnc
encoderType Odom::prevEncoderVal = {0, 0, 0};
encoderType Odom::deltaEncoderVal = {0, 0, 0};
//angle
double Odom::currentAngle = 0.0;
double Odom::prevAngle = 0.0;
double Odom::deltaAngle = 0.0;

//ODOMETRY FUNCTIONS
void Odom::updateSensors(){
  currentAngle = Math::getRadians(imu.get_rotation());
  deltaAngle = currentAngle - prevAngle;
  prevAngle = currentAngle;

  encoderVal.left = Math::degToInch(encoderLeft.get_value()); //leftE
  encoderVal.right = encoderVal.left - Math::arclength(currentAngle, track / 2); //rightE
  encoderVal.back = Math::degToInch(encoderBack.get_value()); //backE

  deltaEncoderVal.left = encoderVal.left - prevEncoderVal.left; //leftE
  deltaEncoderVal.right = encoderVal.right - prevEncoderVal.right; //rightE
  deltaEncoderVal.back = encoderVal.back - prevEncoderVal.back; //backE

  prevEncoderVal.left = encoderVal.left; //leftE
  prevEncoderVal.right = encoderVal.right; //rightE
  prevEncoderVal.back = encoderVal.back; //backe

  //deltaAngle = -(deltaEncoderVal.right - deltaEncoderVal.left) / track;
  //currentAngle = deltaAngle + prevAngle;
  //prevAngle = currentAngle;
}
void Odom::updatePosition(){
  //Polar coordinates
  localDeltaPoint.x = (deltaAngle + (deltaEncoderVal.back / backEncOffset)) * backEncOffset;
  localDeltaPoint.y = (deltaEncoderVal.left + deltaEncoderVal.right) / 2;

  //Cartesian coordinates
  globalDeltaPoint.x = (localDeltaPoint.y * sin(prevAngle + deltaAngle/2)) + (localDeltaPoint.x * cos(prevAngle + deltaAngle/2)); 
  globalDeltaPoint.y = (localDeltaPoint.y * cos(prevAngle + deltaAngle/2)) - (localDeltaPoint.x * sin(prevAngle + deltaAngle/2));
  globalDeltaPoint.angle = deltaAngle;

  //X & y Position
  globalPoint.x = globalDeltaPoint.x + prevGlobalPoint.x;
  globalPoint.y = globalDeltaPoint.y + prevGlobalPoint.y;
  globalPoint.angle = currentAngle;

  prevGlobalPoint.x = globalPoint.x;
  prevGlobalPoint.y = globalPoint.y;
}

void Odom::reset(){
  encoderLeft.reset();  encoderBack.reset();
  prevEncoderVal.left = 0.0; prevEncoderVal.right = 0.0; prevEncoderVal.back = 0.0;
  prevAngle = 0.0;
  prevGlobalPoint.x = 0.0; 
  prevGlobalPoint.y = 0.0;
}

void Odom::setPosition(double newX, double newY, double newAngle){
  reset();
  prevAngle = newAngle;
  prevGlobalPoint.x = newX;
  prevGlobalPoint.y = newY;
}

//ODOMETRY THREAD
int Odom::Odometry(){
  while(true) {
    Odom::updateSensors();
    Odom::updatePosition();
    
    pros::lcd::print(1, "X: %f", Odom::globalPoint.x);
    pros::lcd::print(2,"Y: %f", Odom::globalPoint.y);
    pros::lcd::print(3,"Heading: %f", Math::getDeg(Odom::globalPoint.angle));
    
    pros::lcd::print(4, "L: %f", Odom::encoderVal.left);
    pros::lcd::print(5, "R: %f", Odom::encoderVal.right);
    pros::lcd::print(6,"B: %f", Odom::encoderVal.back);

    pros::c::task_delay(10);
  }
  return 0;
}