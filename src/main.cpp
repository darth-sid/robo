#include "main.h"

//================================================================================================================
//   | _____|_|  |___  | |   |_____ | |   |_____ | |   ____     ____
//   | |_|___     ___| | |   _____| | |        / / /   \ \ \   / / /
//   |____ \ \   |___  | |   | _____|_|       / / /     \ \ \ / / /
//    ___| | |    ___| | |   | |_|__ _       / / /       \ \ / / /
//   |_____/_/   |_____| |   |______|_|     /_/_/         \___/_/
//================================================================================================================

//CONSTANTSS===============================================================================================
//PORTS
#define FRONT_LEFT_PORT 7 //reverse
#define MIDDLE_LEFT_PORT 8 //ok
#define BACK_LEFT_PORT 3//reverse
#define FRONT_RIGHT_PORT 6 //ok
#define MIDDLE_RIGHT_PORT 9//reverse
#define BACK_RIGHT_PORT 10//ok

#define DIFFMOTOR1 13
#define DIFFMOTOR2 14

#define PNEUM_PORT_1 6
#define PNEUM_PORT_2 7

#define INERTIAL_PORT 15
#define VISION_PORT 0

#define LEFT_ROT_PORT 0
#define RIGHT_ROT_PORT 0
#define BACK_ROT_PORT 0

//misc
#define TRACKER_DISTANCE 13.5
#define TRACKER_DISTANCE_BACK 4.5
//================================================================================================================

//COMPONENTS======================================================================================================
//motors
pros::Motor driveFrontLeft(FRONT_LEFT_PORT, MOTOR_GEARSET_18, true);//200rpm
pros::Motor driveMiddleLeft(MIDDLE_LEFT_PORT, MOTOR_GEARSET_18, false);//200rpm
pros::Motor driveBackLeft(BACK_LEFT_PORT, MOTOR_GEARSET_18, true);//200rpm
pros::Motor driveFrontRight(FRONT_RIGHT_PORT, MOTOR_GEARSET_18, false);//200rpm
pros::Motor driveMiddleRight(MIDDLE_RIGHT_PORT, MOTOR_GEARSET_18, true);//200rpm
pros::Motor driveBackRight(BACK_RIGHT_PORT, MOTOR_GEARSET_18, false);//200rpm

pros::Motor diff1(DIFFMOTOR1,false);
pros::Motor diff2(DIFFMOTOR2,false);
//pneum
pros::ADIDigitalOut pneumatic1(PNEUM_PORT_1);
pros::ADIDigitalOut pneumatic2(PNEUM_PORT_2);

pros::Controller Vcontroller(pros::E_CONTROLLER_MASTER);

pros::Vision visionSensor(VISION_PORT);
pros::Imu inertial(INERTIAL_PORT);
//rot
pros::Rotation left_rot(LEFT_ROT_PORT);
pros::Rotation right_rot(RIGHT_ROT_PORT);
pros::Rotation back_rot(BACK_ROT_PORT);
//================================================================================================================
float tare_right;
float tare_left;
float tare_back;

//GLOBAL X
float pos_x = 0;
//GLOBAL Y
float pos_y = 0;

float prev_y_encoder = 0;
float prev_x_encoder = 0;
float prev_angle = 0;


bool clamped1 = false;
bool clamped2 = false;
//================================================================================================================

void diffyliffy(){
  int dir = -1*Vcontroller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
  if (abs(dir) < 10 ){
    if(Vcontroller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
      diff1.move(127);
      diff2.move(-127);
    }
    else if(Vcontroller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
      diff1.move(-127);
      diff2.move(127);
    }
    else{
      diff1.move(0);
      diff2.move(0);
    }
  }
  else{
    diff1.move(dir);
    diff2.move(dir);

  }
}

//scuffed rot sensor stuff
float right_rot_pos(){
 	return right_rot.get_position() - tare_right;
 }
float left_rot_pos(){
 	return left_rot.get_position() - tare_left;
 }
float back_rot_pos(){
	return back_rot.get_position() - tare_back;
}

//run motors
void setDriveMotors(int l, int r) {
	driveFrontLeft = l;
	driveFrontRight = r;
	driveBackLeft = l;
	driveBackRight = r;
	driveMiddleLeft = l;
	driveMiddleRight = r;
	}

void brake_coast(){
	driveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveMiddleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	driveMiddleRight.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
}
void brake_hold(){
	driveFrontLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveFrontRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveBackLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveBackRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveMiddleLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
	driveMiddleRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}



// control motors with controller joystick
void setDrive() {
   int leftJoystickYInput = Vcontroller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
   int leftJoystickXInput = Vcontroller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);//*dir;

   //Deadzones (8,8)
   if (abs(leftJoystickYInput) < 10) {
     leftJoystickYInput = 0;
   }
   if (abs(leftJoystickXInput) < 10) {
     leftJoystickXInput = 0;
   }
   setDriveMotors((leftJoystickYInput+leftJoystickXInput), (leftJoystickYInput+(-leftJoystickXInput)));
   //  setDrive (leftJoystickYInput, rightJoystickYInput);
 }

//toggles pneumatic mogo clamp
void clamp(){
	if(Vcontroller.get_digital_new_press(DIGITAL_R1)){
		clamped1 = !clamped1;
	}
  if(Vcontroller.get_digital_new_press(DIGITAL_R2)){
		clamped2 = !clamped2;
	}
	pneumatic1.set_value(clamped1);
  pneumatic2.set_value(clamped2);
}

//returns bots orientation in radians
double getAngle(){
	double TRACKER_SCALAR = 1.025;
	double encoder_angle = (right_rot_pos()*2.75*M_PI/36000 - left_rot_pos()*2.75*M_PI/36000)/(TRACKER_DISTANCE)*TRACKER_SCALAR;
  pros::lcd::set_text(1, std::to_string(inertial.get_yaw()));
  return inertial.get_yaw();
  //return encoder_angle;
}

void updateCoords(){
	double angle = getAngle();
	double delta_angle = angle-prev_angle;
	//tracked values in inches(ideal centidegrees to inches conversion: 2.75*pi/36000)
	double curr_y_encoder = (2.75*M_PI/36000)*(left_rot_pos()+right_rot_pos())/2;
	double curr_x_encoder = (2.75*M_PI/36000)*back_rot_pos();
	int delta_local_y = (curr_y_encoder-prev_y_encoder)*100000;
	int delta_local_x = (curr_x_encoder-prev_x_encoder)*100000;// - TRACKER_DISTANCE_BACK*getAngle();
	//account for turning arc due to offset from center
	//delta_x_encoder -= 4.5*delta_angle;
	//radius of respective arcs
	/*double ry = delta_y_encoder/delta_angle;
	ry = (std::isnan(ry)) ? 0 : ry;
 	double rx = (delta_x_encoder/delta_angle);
	rx = (std::isnan(rx)) ? 0 : rx;*/
	//displacement calc based on chord-arc relationship
	//double delta_local_x = 2*rx*sin(delta_angle/2);
	//double delta_local_y = 2*ry*sin(delta_angle/2);
	int distance = sqrt(pow(delta_local_y,2)+pow(delta_local_x,2));
	//distance = (std::isnan(distance)) ? 0 : distance;
	/*if(abs(delta_angle)<=0.5){
		//displacement calc assuming straight line movement
		delta_local_x = delta_x_encoder;
		delta_local_y = delta_y_encoder;
	}*/
	//account for backwards movement
	if(delta_local_y < 0) angle += M_PI;
	float adjustment = (std::isnan(atan((delta_local_x)/(delta_local_y)))) ? M_PI/2 : atan((delta_local_x)/(delta_local_y));
	pros::lcd::set_text(1, std::to_string((angle-adjustment)));
	pos_x += cos(angle - adjustment)*distance;
	pos_y += sin(angle - adjustment)*distance;
	prev_x_encoder = curr_x_encoder;
	prev_y_encoder = curr_y_encoder;
	prev_angle = angle;
}

double localPos(){
  return (4.0*M_PI/720.0)*(driveBackLeft.get_position()+driveMiddleLeft.get_position()+driveFrontLeft.get_position()+driveBackRight.get_position()+driveMiddleRight.get_position()+driveFrontRight.get_position())/6;
}

void pidMoveSimple(double target, double tolerance, double KP){
  double error = target-localPos();
  while (abs(error) > tolerance){
    setDriveMotors(error*KP,error*KP);
    error = target-localPos();
  }
  setDriveMotors(0,0);
}


void pidMove(double target, double tolerance, double kP, double kPa){
  //distance to target
  double error = target-localPos();
  //deviation from being straight
  double angleError = 0-getAngle();

  while (true){//fabs(error) > tolerance){
    double pwr = (error*kP > 127) ? 127 : error*kP;
    double lpwr = pwr+angleError*kPa;
    double rpwr = pwr-angleError*kPa;

    pros::lcd::set_text(2, std::to_string(lpwr));
    pros::lcd::set_text(3, std::to_string(rpwr));
    setDriveMotors(lpwr,rpwr);
    error = target-localPos();
  }

}

void tare_rot(){
	while (tare_right == 0){
		tare_right = right_rot.get_position();
		pros::delay(1);
	}
	while (tare_left == 0){
		tare_left = left_rot.get_position();
		pros::delay(1);
	}
	while (tare_back == 0){
		tare_back = back_rot.get_position();
		pros::delay(1);
	}
}

void pp(double x, double y, double angle, double kP){
  double d = pow(pow(x-pos_x, 2) + pow(y-pos_y, 2),0.5);
  double r = pow(d,2)/(2*x);
  double pwr = (d*kP > 127) ? 127 : d*kP;

  setDriveMotors(pwr,pwr);
}

void driveFwd(double distance, double tolerance, double kP){
  double start_y = ((right_rot_pos() + left_rot_pos())/2)*2.75*M_PI/36000;
  double error = distance;
  while (abs(error) > tolerance){
    double delta_y = ((right_rot_pos() + left_rot_pos())/2)*2.75*M_PI/36000 - start_y;
    error = distance - delta_y;
    setDriveMotors(error*kP,error*kP);
  }
  setDriveMotors(0,0);

}


void splitPID(double targetl, double targetr, double tolerance, double kP, double max_speed){
  brake_hold();
	double refL = driveFrontLeft.get_position();
	double refR = driveFrontRight.get_position();
	double errorL = targetl - (driveFrontLeft.get_position()-refL);
	double errorR = targetr - (driveFrontRight.get_position()-refR);
	while (fabs(errorL) > tolerance || fabs(errorR) > tolerance){
    double pwrL = (errorL*kP > max_speed) ? max_speed : errorL*kP;
    double pwrR = (errorR*kP > max_speed) ? max_speed : errorR*kP;
		setDriveMotors(pwrL,pwrR);
		errorL = targetl - (driveFrontLeft.get_position()-refL);
		errorR = targetr - (driveFrontRight.get_position()-refR);
		pros::lcd::set_text(1,std::to_string((driveFrontLeft.get_position()-refL)));
		pros::lcd::set_text(2,std::to_string((driveFrontRight.get_position()-refR)));
	}
	setDriveMotors(0,0);
  brake_coast();
}

void splitPID(double targetl, double targetr, double tolerance, double kP){
  splitPID(targetl, targetr, tolerance, kP, 127);
}

void setAngle(double target, double tolerance, double kP){
	brake_hold();
	double error = target-inertial.get_yaw();
	while (fabs(error) > tolerance){
		error = target-inertial.get_yaw();
		setDriveMotors(error*kP, -error*kP);
		pros::lcd::set_text(1, std::to_string(inertial.get_yaw()));
		pros::lcd::set_text(2, std::to_string(error));
	}
	setDriveMotors(0,0);
	brake_coast();
}

void bumrush(double target_raw){
	double target = target_raw * 4/3;
	double ref = localPos();
	double d = 0;
	while (d < target_raw){
		setDriveMotors(127,127);
		d = localPos()-ref;
	}
	setDriveMotors(0,0);
	pneumatic1.set_value(false);
	pros::delay(200);
	diff1.move_relative(-750,127);
	diff2.move_relative(-750,127);

}

void bumrush2(double target_raw){
	double target = target_raw * 4/3;
	double ref = localPos();
	double d = 0;
	while (d < target_raw){
		setDriveMotors(-127,-127);
		d = localPos()-ref;
	}
	setDriveMotors(0,0);
	pneumatic2.set_value(false);
	pros::delay(200);
	diff1.move_relative(2000,127);
	diff2.move_relative(-2000,127);

}

void auton1(){
  //setAngle(90,1,1);
  pneumatic1.set_value(true);
	pneumatic2.set_value(true);
  diff1.move_relative(-100,127);
	diff2.move_relative(-100,127);
	bumrush(48);
	pros::delay(500);
	splitPID(-1200,1200,10,0.5);
	diff1.move_relative(-2000,127);
	diff2.move_relative(-2000,127);
	splitPID(3400,3400,10,0.5);
	//pros::delay(1000);
	diff1.move_relative(1000,127);
	diff2.move_relative(1000,127);
	pros::delay(500);
	pneumatic1.set_value(true);
	splitPID(0,-1000,10,0.5);
  pros::delay(250);
  diff1.move_relative(500,127);
	diff2.move_relative(500,127);
  //bumrush2(40);
  diff1.move_relative(500,127);
	diff2.move_relative(500,127);
	//splitPID(-2600,-2600,30,0.5);
  setDriveMotors(-127,-127);
  pros::delay(1500);
	pneumatic2.set_value(false);
	//diff1.move_relative(500,127);
	//diff2.move_relative(500,127);
  pros::delay(300);
	diff1.move(127);
	diff2.move(-127);
  pros::delay(500);
	splitPID(1500,1500,15,0.5);
	diff1.move(0);
	diff2.move(0);
}

void auton2(){
  pneumatic1.set_value(true);
	pneumatic2.set_value(true);
	bumrush(48);
	pros::delay(500);
  //diff1.move_relative(-1000,127);
	//diff2.move_relative(-1000,127);
  //splitPID(-475,0,10,0.5);
  /*setDriveMotors(-127,-127);
  pros::delay(1000);
  setDriveMotors(0,0);*/
  splitPID(-2000,-2000,10,0.5);
  pros::delay(250);
  splitPID(-800,600,10,0.7,100);
  pros::delay(750);
  //splitPID(-825,-825,50,0.2);
  setDriveMotors(-127, -127);
  pros::delay(500);
  //pneumatic2.set_value(false);
  diff1.move(127);
	diff2.move(-127);
  pros::delay(500);
  diff1.move(0);
	diff2.move(0);
  splitPID(1500,1500,15,0.5);
}

void auton3(){
  pneumatic1.set_value(true);
  pneumatic2.set_value(true);
  diff1.move_relative(-100,127);
  diff2.move_relative(-100,127);
  splitPID(800,800,15,0.3);
  splitPID(0,715,15,0.5);
  splitPID(2500,2500,15,0.3);
  pros::delay(200);
  splitPID(225,225,50,0.3);
  //splitPID(0,100,50,0.6);
  pros::lcd::set_text(5, "scre");
  pneumatic1.set_value(false);
  pros::delay(200);
  splitPID(-2500,-2500,15,0.3);
  splitPID(-600,600,15,0.8);
  pneumatic1.set_value(true);
  diff1.move_relative(-500,127);
	diff2.move_relative(-500,127);
  setDriveMotors(-127,-127);
  pros::delay(750);
  setDriveMotors(0,0);
  pros::delay(500);
  pneumatic2.set_value(false);
  diff1.move(127);
	diff2.move(-127);
  pros::delay(1000);
  diff1.move(0);
	diff2.move(0);
  pros::delay(250);
  splitPID(-400,400,15,0.8);
  setDriveMotors(127,127);
  pros::delay(500);
  setDriveMotors(0, 0);

  //splitPID(0,200,15,0.5);
  //bumrush(50);
}
//================================================================================================================

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	/*visionSensor.clear_led();
	pros::vision_signature_s_t yellow_mogo = pros::Vision::signature_from_utility(1, 2911, 4681, 3796, -6527, -5567, -6046, 3.000, 0);
	pros::vision_signature_s_t blue_mogo = pros::Vision::signature_from_utility(2, -4319, -3039, -3678, 12895, 15519, 14206, 3.000, 0);
	pros::vision_signature_s_t red_mogo = pros::Vision::signature_from_utility(3, 12385, 14505, 13446, -1945, -607, -1276, 3.000, 0);
	visionSensor.set_signature(1, &yellow_mogo);
	visionSensor.set_signature(2, &blue_mogo);
	visionSensor.set_signature(3, &red_mogo);
	pros::lcd::set_text(7, "reset");*/
	tare_rot();
	diff1.tare_position();
	//inertial.reset();
	driveBackLeft.tare_position();
	driveMiddleLeft.tare_position();
	driveFrontLeft.tare_position();
	driveBackRight.tare_position();
	driveMiddleRight.tare_position();
	driveFrontRight.tare_position();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabl2ing the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  auton3();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	pros::vision_object_s_t mogo;
	while(true){
		/*mogo = visionSensor.get_by_size(0);
		pros::lcd::set_text(1, "sig:" + std::to_string(mogo.signature));
		pros::lcd::set_text(2, "pos:" + std::to_string(mogo.x_middle_coord));*/
    	diffyliffy();
		setDrive();
    	clamp();
    	//updateCoords();
		//clamp();
		//ringmaster();
		pros::lcd::set_text(1, std::to_string(inertial.get_yaw()));
    	pros::lcd::set_text(2, std::to_string(localPos()));
    	//pros::lcd::set_text(2, std::to_string(pos_x) + "," + std::to_string(pos_y));
    	pros::delay(10);
	}
}
