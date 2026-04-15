#include "main.h"
#include "lemlib/api.hpp"

pros::MotorGroup left_motors({-6, 8, -5}, pros::MotorGears::blue);
pros::MotorGroup right_motors({9, -10, 7}, pros::MotorGears::blue);
// drivetrain settings
lemlib::Drivetrain drivetrain(&left_motors,               // left motor group
                              &right_motors,              // right motor group
                              10.75,                      // 10 inch track width
                              lemlib::Omniwheel::NEW_275, // using new 4" omnis
                              600,                        // drivetrain rpm is 360
                              2                           // horizontal drift is 2 (for now)
);

pros::Imu imu(21);
pros::Rotation horizontal_encoder(-3);
lemlib::TrackingWheel horizontal_tracking_wheel(&horizontal_encoder, lemlib::Omniwheel::NEW_2, -4.45);

pros::Controller master(pros::E_CONTROLLER_MASTER);

// odometry settings
lemlib::OdomSensors sensors(nullptr,                    // vertical tracking wheel 1, set to null
                            nullptr,                    // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr,                    // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu                        // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15,  // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              100, // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20   // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2,   // proportional gain (kP)
                                              0,   // integral gain (kI)
                                              18,  // derivative gain (kD)
                                              3,   // anti windup
                                              1,   // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3,   // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0    // maximum acceleration (slew)
);

// create the chassis
lemlib::Chassis chassis(drivetrain,         // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors             // odometry sensors
);

enum class Auton
{
  Left9,
  Left6,
  Right6,
  Right9,
};

std::unordered_map<int, Auton> createAutonMap()
{
  return {
      {1, Auton::Right6},
      {2, Auton::Right9},
      {3, Auton::Left6},
      {4, Auton::Left9},
  };
}

const char *autonToString(Auton auton)
{
  switch (auton)
  {
  case Auton::Left6:
    return "Left Six";
  case Auton::Left9:
    return "Left Nine";
  case Auton::Right6:
    return "Right Six";
  case Auton::Right9:
    return "Right Nine";
  default:
    return "Unknown";
  }
}

std::unordered_map<int, Auton> autonMap = createAutonMap();

int autonCount = 1;

void on_left_button()
{
  autonCount += 1;
  if (autonCount == 7)
  {
    autonCount = 1;
  }
}

void on_right_button()
{
  autonCount -= 1;
  if (autonCount == 0)
  {
    autonCount = 6;
  }
}

pros::Motor intake(2);
pros::Motor lever(20);

pros::adi::DigitalOut blocker('D');
pros::adi::DigitalOut lift('G');
pros::adi::DigitalOut matchloader('H');
pros::adi::DigitalOut wing('C');

bool drive_arcade = true;
bool intake_toggle = false;
bool reverse_toggle = false;
bool lift_toggle = false;
bool wing_toggle = false;
bool matchloader_toggle = false;
bool start_down = false;
bool score_intake_toggle = false;

void score()
{
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle)
  {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move(127);
    blocker.set_value(true);
    pros::delay(1000);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(1000);
    lever.move_velocity(0);
  }
  else
  {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move_velocity(70);
    blocker.set_value(true);
    pros::delay(1200);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void score_three()
{
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle)
  {
    intake.move(-50);
    score_intake_toggle = true;
    //lever.move(127);
    lever.move_relative(500, 127);
    blocker.set_value(true);
    pros::delay(700);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(1000);
    lever.move_velocity(0);
  }
  else
  {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move_velocity(70);
    blocker.set_value(true);
    pros::delay(300);
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void score_driver()
{
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  if (!lift_toggle)
  {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move(127);
    blocker.set_value(true);
    pros::delay(1000);
    while (master.get_digital(DIGITAL_R2))
    {
      pros::delay(20);
    }
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(400);
    lever.move_velocity(0);
  }
  else
  {
    intake.move(-50);
    score_intake_toggle = true;
    lever.move_velocity(100);
    blocker.set_value(true);
    pros::delay(1200);
    while (master.get_digital(DIGITAL_R2))
    {
      pros::delay(20);
    }
    lever.move(-127);
    score_intake_toggle = false;
    pros::delay(1000);
    lever.move_velocity(0);
  }
  lever.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void controls()
{
  lever.move(-127);
  pros::delay(800);
  lever.set_zero_position(0);
  lever.move(0);
  while (true)
  {
    bool r1_new = master.get_digital_new_press(DIGITAL_R1);
    bool l1_new = master.get_digital_new_press(DIGITAL_L1);
    bool r1 = master.get_digital(DIGITAL_R1);
    bool l1 = master.get_digital(DIGITAL_L1);

    if ((r1_new && l1) || (l1_new && r1))
    {
      lift_toggle = !lift_toggle;
      blocker.set_value(false);
      pros::delay(250);
    }
    else if (l1_new)
    {
      intake_toggle = !intake_toggle;
      blocker.set_value(false);
    }
    else if (lift_toggle)
    {
      wing_toggle = false;
    }
    else if (r1)
    {
      wing_toggle = false;
      start_down = false;
    }
    else if (start_down)
    {
      wing_toggle = false;
    }
    else if (!r1 && !start_down)
    {
      wing_toggle = true;
    }

    if (master.get_digital_new_press(DIGITAL_X))
    {
      lever.move(-127);
      pros::delay(800);
      lever.set_zero_position(0);
      lever.move(0);
    }

    if (master.get_digital_new_press(DIGITAL_UP))
    {
      drive_arcade = !drive_arcade;
      master.set_text(0, 0, drive_arcade ? "Drive: Arcade" : "Drive: Tank");
      master.rumble(drive_arcade ? "." : "..");
    }

    if (master.get_digital_new_press(DIGITAL_DOWN))
    {
      matchloader_toggle = !matchloader_toggle;
    }

    if (master.get_digital(DIGITAL_L2))
    {
      intake_toggle = false;
      reverse_toggle = true;
    }
    else
    {
      reverse_toggle = false;
    }

    if (master.get_digital_new_press(DIGITAL_R2))
    {
      pros::Task score_task(score_driver);
    }

    if (master.get_digital_new_press(DIGITAL_B))
    {
      pros::Task score_task(score_three);
    }
    pros::delay(20);
  }
}

void initialize()
{
  pros::lcd::initialize(); // initialize brain screen
  chassis.calibrate();     // calibrate sensors
  pros::lcd::register_btn2_cb(on_left_button);
  pros::lcd::register_btn0_cb(on_right_button);
  // print position to brain screen
  pros::Task screen_task([&]()
                         {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(5, "Auton: %s", autonToString(autonMap[autonCount]));
            // delay to save resources
            pros::delay(80);
        } });
}

void disabled()
{
  // . . .
}

void competition_initialize()
{
  // . . .
}

void right_starter(){
  wing.set_value(true);
  intake.move(127);
  chassis.setPose(45, 6, 270);
  chassis.moveToPoint(24, 24, 750, {.earlyExitRange = 3});
  pros::delay(400);
  matchloader.set_value(true);
  chassis.moveToPoint(10, 48.5, 1000);
  pros::delay(50);
  matchloader.set_value(false);
  pros::delay(300);
  matchloader.set_value(true);
  pros::delay(200);
  chassis.swingToPoint(40, 46, lemlib::DriveSide::RIGHT, 800, {.forwards = false, .earlyExitRange=5});
  chassis.moveToPoint(40, 46, 250, {.forwards = false});
  chassis.swingToHeading(90, lemlib::DriveSide::RIGHT, 1100, {});
  pros::delay(800);
  pros::Task scoreTask(score);
  chassis.waitUntilDone();
  chassis.tank(-127, -127);
}
void left_starter(){
  wing.set_value(true);
  intake.move(127);
  chassis.setPose(45, -6, 270);
  chassis.moveToPoint(24, -24, 750, {.earlyExitRange = 3});
  pros::delay(400);
  matchloader.set_value(true);
  chassis.moveToPoint(13, -49, 1000);
  pros::delay(50);
  matchloader.set_value(false);
  pros::delay(300);
  matchloader.set_value(true);
  
  pros::delay(200);
  chassis.swingToPoint(42, -46, lemlib::DriveSide::LEFT, 800, {.forwards = false, .earlyExitRange=5});
  chassis.moveToPoint(42, -46, 250, {.forwards = false});
  chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1100, {});
  pros::delay(800);
  pros::Task scoreTask(score);
  chassis.waitUntilDone();
  chassis.tank(-127, -127);
  
}

void six_ball_right()
{
  right_starter();
  matchloader.set_value(false);
  pros::delay(400);
  chassis.setPose(27, 47, chassis.getPose().theta);
  chassis.turnToPoint(32, 57, 300);
  chassis.moveToPoint(32, 57, 500);
  wing.set_value(false);
  chassis.turnToHeading(90, 400);
  chassis.moveToPoint(11, 55, 2000, {.forwards = false});
  chassis.turnToHeading(130, 400);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  start_down = true;
}

void six_ball_left()
{
  left_starter();
  matchloader.set_value(false);
  pros::delay(400);
  chassis.setPose(27, 47, chassis.getPose().theta);
  chassis.turnToPoint(32, 57, 300);
  chassis.moveToPoint(32, 57, 500);
  wing.set_value(false);
  chassis.turnToHeading(90, 400);
  chassis.moveToPoint(11, 55, 2000, {.forwards = false});
  chassis.turnToHeading(130, 400);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  start_down = true;
}

void nine_ball_right()
{
  right_starter();
  pros::delay(800);
  chassis.setPose(27, 47, chassis.getPose().theta);
  matchloader.set_value(true);
  intake.move(127);
  pros::delay(400);
  chassis.moveToPoint(55, 46, 1700, {.maxSpeed=55}, false);
  chassis.tank(25, 25);
  pros::delay(100);
  chassis.moveToPoint(38, 46, 1200, {.forwards = false});
  chassis.turnToPoint(-8, 5.5, 500);
  matchloader.set_value(false);
  chassis.moveToPose(-3, 5.5, 235, 2800, {.horizontalDrift = 8, .lead = 0.3}, false);
  intake.move(-70);
  chassis.tank(40, 40);
  pros::delay(2200);
  // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  // chassis.moveToPoint(26, 20, 1000, {.forwards = false});
  // wing.set_value(false);
  // chassis.turnToHeading(275, 1000);
  // chassis.moveToPoint(-10, 26, 2000, {});
  // chassis.turnToHeading(240, 400);
  // chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
  start_down = true;
  //TODO: fix wing
}

void nine_ball_left()
{
  left_starter();
  pros::delay(800);
  chassis.setPose(27, -47, chassis.getPose().theta);
  matchloader.set_value(true);
  intake.move(127);
  pros::delay(400);
  chassis.moveToPoint(55, -46, 1700, {.maxSpeed=55}, false);
  chassis.tank(25, 25);
  pros::delay(100);
  chassis.moveToPoint(38, -46, 1200, {.forwards = false});
  chassis.turnToPoint(0, -5, 500, {.forwards = false});
  matchloader.set_value(false);
  lift.set_value(true);
  lift_toggle = true;
  chassis.moveToPose(0, -5, 305, 2800, {.forwards = false, .horizontalDrift = 8, .lead = 0.3}, false);
  pros::Task scoreMidTask(score);
  chassis.tank(-20, -20);
  pros::delay(2200);
}

void autonomous()
{
  auto it = autonMap.find(autonCount);
  Auton selected = (it != autonMap.end()) ? it->second : Auton::Right6;

  switch (selected)
  {
  case Auton::Left6:
    six_ball_left();
    break;
  case Auton::Left9:
    nine_ball_left();
    break;
  case Auton::Right6:
    six_ball_right();
    break;
  case Auton::Right9:
    nine_ball_right();
    break;
  default:
    break;
  }
}

void opcontrol()
{
  pros::Task controlTask(controls);
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
  while (true)
  {
    int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    int rightY = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
    int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
    if (drive_arcade)
      chassis.arcade(leftY, rightX);
    else
      chassis.tank(leftY, rightY);

    if (reverse_toggle)
    {
      intake.move(-90);
    }

    if (intake_toggle && !reverse_toggle)
    {
      intake.move(127);
    }

    if (score_intake_toggle){
      intake.move(-50);
    } else if (intake_toggle && !reverse_toggle)
    {
      intake.move(127);
    } else if (!reverse_toggle)
    {
      intake.move(0);
    }

    matchloader.set_value(matchloader_toggle);
    lift.set_value(lift_toggle);
    wing.set_value(wing_toggle);
    pros::delay(20);
  }
}
