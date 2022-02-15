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
#define FRONT_LEFT_PORT 16 //reverse
#define MIDDLE_LEFT_PORT 19 //ok
#define BACK_LEFT_PORT 14 //reverse
#define FRONT_RIGHT_PORT 17 //ok
#define MIDDLE_RIGHT_PORT 18 //reverse
#define BACK_RIGHT_PORT 15 //ok

#define DIFFMOTOR1 13
#define DIFFMOTOR2 20

#define PNEUM_PORT_1 8
#define PNEUM_PORT_2 0

#define VISION_SENSOR_PORT 0

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

pros::Vision visionSensor(VISION_SENSOR_PORT);

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
  if(Vcontroller.get_digital_new_press(DIGITAL_L2)){
		clamped2 = !clamped2;
	}
	pneumatic1.set_value(clamped1);
  //pneumatic2.set_value(clamped2);
}

//returns bots orientation in radians
double getAngle(){
	double TRACKER_SCALAR = 1.025;
	double encoder_angle = (right_rot_pos()*2.75*M_PI/36000 - left_rot_pos()*2.75*M_PI/36000)/(TRACKER_DISTANCE)*TRACKER_SCALAR;
	return encoder_angle;
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

//================================================================================================================

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	visionSensor.clear_led();
	pros::vision_signature_s_t yellow_mogo = pros::Vision::signature_from_utility(1, 2911, 4681, 3796, -6527, -5567, -6046, 3.000, 0);
	pros::vision_signature_s_t blue_mogo = pros::Vision::signature_from_utility(2, -4319, -3039, -3678, 12895, 15519, 14206, 3.000, 0);
	pros::vision_signature_s_t red_mogo = pros::Vision::signature_from_utility(3, 12385, 14505, 13446, -1945, -607, -1276, 3.000, 0);
	visionSensor.set_signature(1, &yellow_mogo);
	visionSensor.set_signature(2, &blue_mogo);
	visionSensor.set_signature(3, &red_mogo);
	pros::lcd::set_text(7, "reset");
	tare_rot();
  diff1.tare_position();
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
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

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
		//pros::lcd::set_text(1, std::to_string(getAngle()*180/M_PI));
		//pros::lcd::set_text(2, std::to_string(pos_x) + "," + std::to_string(pos_y));
    pros::delay(10);
	}
}
