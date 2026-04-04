#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup left_motors({-6, 8, -5}, pros::MotorGears::blue);
pros::MotorGroup right_motors({9, -10, 7}, pros::MotorGears::blue);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors, // left motor group
                              &right_motors, // right motor group
                              10.75, // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);

pros::Imu imu(21);
pros::Rotation horizontal_encoder(-3);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -4.45);

pros::Controller master(pros::E_CONTROLLER_MASTER);


// odometry settings
lemlib::OdomSensors sensors(nullptr, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);


pros::Motor intake(2);
pros::Motor lever(20);

pros::adi::DigitalOut blocker('D');
pros::adi::DigitalOut lift('G');
pros::adi::DigitalOut matchloader('H');
pros::adi::DigitalOut wing('C');

bool drive_arcade = false;
bool intake_toggle = false;
bool reverse_toggle = false;
bool lift_toggle = false;
bool wing_toggle = false;
bool matchloader_toggle = false;
bool start_down = false;

void score(){
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle){
    intake.move(127);
    intake_toggle = true;
    lever.move(127);
    blocker.set_value(true);
    pros::delay(800);
    lever.move(-127);
    intake_toggle = false;
    pros::delay(1000);
    lever.move_velocity(0);
  } else {
    intake.move(127);
    intake_toggle = true;
    lever.move_velocity(70);
    blocker.set_value(true);
    pros::delay(1000);
    lever.move(-127);
    intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void score_driver(){
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle){
    intake.move(127);
    intake_toggle = true;
    lever.move(127);
    blocker.set_value(true);
    pros::delay(800);
    while (master.get_digital(DIGITAL_R2)) {
      pros::delay(20);
    }
    lever.move(-127);
    intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  } else {
    intake.move(127);
    intake_toggle = true;
    lever.move_velocity(100);
    blocker.set_value(true);
    pros::delay(1000);
    while (master.get_digital(DIGITAL_R2)) {
      pros::delay(20);
    }
    lever.move(-127);
    intake_toggle = false;
    pros::delay(1000);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}


void controls() {
  lever.move(-127);
  pros::delay(800);
  lever.set_zero_position(0);
  lever.move(0);
  while (true) {
    bool r1_new = master.get_digital_new_press(DIGITAL_R1);
    bool l1_new = master.get_digital_new_press(DIGITAL_L1);
    bool r1 = master.get_digital(DIGITAL_R1);
    bool l1 = master.get_digital(DIGITAL_L1);

    if ((r1_new && l1) || (l1_new && r1)) {
      lift_toggle = !lift_toggle;
      blocker.set_value(false);
      pros::delay(250);
    } else if (l1_new) {
      intake_toggle = !intake_toggle;
      blocker.set_value(false);
    } else if (lift_toggle){
      wing_toggle = false;
    } else if (r1) {
      wing_toggle = false;
      start_down = false;
    } else if (start_down){
      wing_toggle = false;
    }
      else if (!r1 && !start_down) {
      wing_toggle = true;
    }

    if (master.get_digital_new_press(DIGITAL_X)){
      lever.move(-127);
      pros::delay(800);
      lever.set_zero_position(0);
      lever.move(0);
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
      drive_arcade = !drive_arcade;
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
      master.rumble(drive_arcade ? "." : "..");
    }

    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      matchloader_toggle = !matchloader_toggle;
    }

    if (master.get_digital(DIGITAL_L2)) {
      intake_toggle = false;
      reverse_toggle = true;
    } else {
      reverse_toggle = false;
    }

    if (master.get_digital_new_press(DIGITAL_R2)) {
      pros::Task score_task(score_driver);
    }
    pros::delay(20);
  }
}


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
            pros::delay(80);
        }
    });
}

void disabled() {
  // . . .
}

void competition_initialize() {
  // . . .
}

void autonomous() {
	chassis.setPose(0,0,0);
	chassis.moveToPoint(0,20,1000);
}

void opcontrol() {
  pros::Task controlTask(controls);

  while (true) {
	int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
	int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
	int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (drive_arcade)
      chassis.arcade(leftY, rightX);
    else
      chassis.tank(leftY, rightY);

    if (reverse_toggle) {
      intake.move(-90);
    }

    if (intake_toggle && !reverse_toggle) {
      intake.move(127);
    } else if (!reverse_toggle) {
      intake.move(0);
    }

    matchloader.set_value(matchloader_toggle);
    lift.set_value(lift_toggle);
    wing.set_value(wing_toggle);
    pros::delay(20);
  }
}
