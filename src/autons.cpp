#include "vex.h"

/**
 * Resets the constants for auton movement.
 * Modify these to change the default behavior of functions like
 * drive_distance(). For explanations of the difference between
 * drive, heading, turning, and swinging, as well as the PID and
 * exit conditions, check the docs.
 */

void default_constants(){
  // Each constant set is in the form of (maxVoltage, kP, kI, kD, startI).
  chassis.set_drive_constants(10, 1.5, 0, 10, 0);
  chassis.set_heading_constants(6, .4, 0, 1, 0);
  chassis.set_turn_constants(12, .4, .03, 3, 15);
  chassis.set_swing_constants(12, .3, .001, 2, 15);

  // Each exit condition set is in the form of (settle_error, settle_time, timeout).
  chassis.set_drive_exit_conditions(0.35, 200, 5000);
  chassis.set_turn_exit_conditions(0.35, 200, 3000);
  chassis.set_swing_exit_conditions(0.35, 200, 3000);
}

/**
 * Sets constants to be more effective for odom movements.
 * For functions like drive_to_point(), it's often better to have
 * a slower max_voltage and greater settle_error than you would otherwise.
 */

void odom_constants(){
  default_constants();
  chassis.heading_max_voltage = 10;
  chassis.drive_max_voltage = 8;
  chassis.drive_settle_error = 0.25;
  chassis.boomerang_lead = .5;
  chassis.drive_min_voltage = 0;
  chassis.drive_timeout = 5000;
  chassis.turn_timeout = 1500;
  chassis.swing_timeout = 1500;
}

void prog_skills() {
  chassis.set_coordinates(64, 23.25, 0);
  odom_constants();
  descoreP.set(true);

  // Intake blocks from the red left center stack
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 4.5;
  chassis.drive_timeout = 1870;
  chassis.drive_to_pose(44.25, 47, -45);
  odom_constants();
  chassis.turn_timeout = 520;
  chassis.turn_to_angle(-135);

  // Score into the red mid goal
  chassis.drive_timeout = 1125;
  chassis.drive_max_voltage = 4;
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_distance(-18.5, -135);
  bottomTriStateP.set(false);
  topTriStateP.set(true);
  intakeMotors.spin(forward, 50, vex::velocityUnits::pct);
  wait(0.5, seconds);
  chassis.set_coordinates(60, 60, -135);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);

  // Intake from the red left match loader
  odom_constants();
  thread b([] {
    wait(0.4, seconds);
    bottomTriStateP.set(true);
    topTriStateP.set(false);
  });
  chassis.drive_timeout = 2500;
  chassis.drive_to_point(17.5, 28);
  chassis.turn_timeout = 535;
  chassis.turn_to_angle(180);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  odom_constants();
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 28, 180);
  matchLoadP.set(true);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 2500;
  chassis.drive_to_pose(23.5, 0, 180);
  
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 12, 180);
  odom_constants();

  // Go to other side and reset pose
  chassis.drive_timeout = 1300;
  chassis.drive_to_point(10, 38);
  odom_constants();
  intakeMotors.stop();
  chassis.drive_distance(-56, 180);
  matchLoadP.set(false);
  wait(0.15, seconds);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 144 - DistanceBack.objectDistance(inches) - 4, 180);

  // Score blocks into the blue left long goal
  chassis.drive_max_voltage = 5;
  chassis.boomerang_lead = 0.2;
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(220);
  chassis.drive_timeout = 1400;
  chassis.drive_to_point(20.25, 118);
  chassis.turn_timeout = 600;
  chassis.turn_to_angle(0);
  wait(0.2, seconds);
  odom_constants();
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 117, 0);
  chassis.drive_max_voltage = 7;
  chassis.drive_timeout = 700;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(23.4, 95, 0);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 2250;
  chassis.drive_distance(-20, 0);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  odom_constants();
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 101, 0);

  // Intake from the blue left match loader
  matchLoadP.set(true);
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 2850;
  chassis.drive_to_pose(23.5, 144, 0);
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 132, 0);
  chassis.drive_max_voltage = 7;
  chassis.drive_timeout = 900;

  // Score into blue left long goal
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_point(24, 90);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 2250;
  chassis.drive_distance(-20, 0);
  odom_constants();

  // Intake red blocks from blue park zone
  matchLoadP.set(false);
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 101, 0);
  odom_constants();
  chassis.drive_min_voltage = 3;
  chassis.drive_max_voltage = 6;
  chassis.boomerang_lead = 0.5;
  chassis.drive_timeout = 1450;
  chassis.drive_to_pose(57, 136, 76);
  odom_constants();
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  chassis.drive_min_voltage = 0;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(47, 80);

  // Reset Robot Pose
  chassis.swing_timeout = 650;
  chassis.left_swing_to_angle(0);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 700;
  chassis.drive_distance(10, 0);
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, 135, 0);

  // Score zone blocks into the mid goal
  odom_constants();
  chassis.swing_timeout = 400;
  chassis.right_swing_to_angle(45);
  chassis.drive_timeout = 1500;
  chassis.drive_to_point(75.5, 89);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(-45);
  chassis.drive_timeout = 800;
  chassis.swing_timeout = 350;
  chassis.right_swing_to_angle(45);
  chassis.turn_timeout = 400;
  chassis.turn_to_angle(45);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 250;
  chassis.drive_distance(-10, 45);
  odom_constants();
  bottomTriStateP.set(false);
  topTriStateP.set(true);
  double totalTime = 2.2;
  int steps = 45;
  double stepTime = totalTime / steps;

  for(int i = 0; i <= steps; i++) {
    double t = i / (double)steps;
    double curve = t * t;
    double speed = 30 + (100 - 30) * curve;

    intakeMotors.spin(forward, speed, vex::velocityUnits::pct);
    wait(stepTime, seconds);
  }
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  intakeMotors.stop();

  // Intake the 4 blue right center blocks
  chassis.turn_timeout = 350;
  chassis.turn_to_angle(45);
  wait(0.15, seconds);
  chassis.set_coordinates(84, 84, 45);
  chassis.drive_max_voltage = 5;
  chassis.drive_timeout = 1500;
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_distance(20, 45);

  // Score into blue right long goal
  odom_constants();
  chassis.drive_timeout = 2200;
  chassis.drive_to_point(126.5, 115);
  chassis.turn_timeout = 575;
  chassis.turn_to_angle(0);
  odom_constants();
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, 112, 0);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 8;
  chassis.drive_timeout = 650;
  chassis.drive_to_point(120, 96);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  wait(1.05, seconds);
  odom_constants();

  // Intake from the blue right match loader
  matchLoadP.set(true);
  thread t([] {
    wait(1.4, seconds);
    bottomTriStateP.set(false);
    topTriStateP.set(false);
  });
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 2850;
  chassis.drive_to_pose(121, 144, 0);
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, 132, 0);
  odom_constants();

  // Go to other side and reset pose
  chassis.drive_timeout = 1300;
  chassis.drive_to_point(135.5, 106);
  odom_constants();
  intakeMotors.stop();
  chassis.drive_distance(-49, 0);
  matchLoadP.set(false);
  wait(0.15, seconds);
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, DistanceBack.objectDistance(inches) + 4, 0);

  // Score blocks into the right red long goal
  chassis.drive_max_voltage = 6;
  chassis.boomerang_lead = 0.2;
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(35);
  chassis.drive_timeout = 1400;
  chassis.drive_to_point(123.25, 27);
  chassis.turn_timeout = 600;
  chassis.turn_to_angle(180);
  wait(0.2, seconds);
  odom_constants();
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 27, 180);
  chassis.drive_max_voltage = 7;
  chassis.drive_timeout = 700;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(120, 54, 180);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  wait(2.25, seconds);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  odom_constants();

  // Intake from the red right match loader
  matchLoadP.set(true);
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 2600;
  chassis.drive_to_pose(119.2, 0, 0);
  chassis.turn_timeout = 400;
  chassis.turn_to_angle(180);
  wait(0.15, seconds);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 12, 180);
  chassis.drive_max_voltage = 7;
  chassis.drive_timeout = 900;

  // Score into red right long goal
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(121.5, 54, 180);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  wait(2.25, seconds);
  odom_constants();

  // Park
  matchLoadP.set(false);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 43, 180);
  odom_constants();
  chassis.drive_max_voltage = 9;
  chassis.boomerang_lead = 0.5;
  chassis.drive_timeout = 1450;
  chassis.drive_min_voltage = 4.5;
  chassis.drive_to_pose(85, 11, -100);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  intakeMotors.spin(reverse, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 0;
  chassis.drive_max_voltage = 7;
  chassis.drive_distance(20.25, -100);
}

void solo_awp_counter() {
    chassis.set_coordinates(86.25, 20.5, 90);
    odom_constants();
    
    // Intake from the red left match loader
    chassis.drive_timeout = 1000;
    chassis.drive_to_point(120, 20);
    matchLoadP.set(true);
    chassis.turn_timeout = 500;
    chassis.turn_to_angle(180);
    chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 20, 180);
    intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
    chassis.drive_min_voltage = 2;
    chassis.drive_max_voltage = 4.5;
    chassis.drive_timeout = 550;
    chassis.drive_to_point(120, 0);
    chassis.turn_timeout = 200;
    chassis.turn_to_angle(180);
    chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 9, 180);
    wait(0.1, seconds);
    odom_constants();

    // Score into right long goal
    chassis.drive_timeout = 800;
    chassis.drive_to_point(120.5, 52);
    bottomTriStateP.set(true);
    topTriStateP.set(false);
    chassis.drive_timeout = 1100;
    chassis.drive_max_voltage = 3;
    chassis.drive_distance(-20, 180);
    bottomTriStateP.set(false);
    topTriStateP.set(false);
    matchLoadP.set(false);

    // Swing and reset pose
    chassis.swing_timeout = 700;
    chassis.left_swing_to_angle(-90);
    odom_constants();
    wait(0.07, seconds);
    chassis.set_coordinates(144 - DistanceBack.objectDistance(inches) - 4, DistanceLeft.objectDistance(inches) + 5.5, -90);
    
    // Intake right center blocks
    chassis.boomerang_lead = 0.2;
    chassis.drive_timeout = 850;
    chassis.drive_to_pose(96, 47.5, -60);
    chassis.turn_timeout = 450;
    chassis.turn_to_angle(-90);

    // Intake Left center blocks
    chassis.drive_timeout = 1300;
    chassis.drive_to_pose(45, 46.5, -90);
    chassis.turn_timeout = 470;
    chassis.turn_to_angle(-135);

    // Score into left long goal
    odom_constants();
    chassis.drive_timeout = 900;
    chassis.drive_to_point(18, 28);
    chassis.turn_timeout = 500;
    chassis.turn_to_angle(180);
    odom_constants();
    chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 28, 180);
    intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
    chassis.drive_max_voltage = 8;
    chassis.drive_timeout = 570;
    chassis.boomerang_lead = 0.1;
    chassis.drive_to_point(24.2, 50);
    bottomTriStateP.set(true);
    topTriStateP.set(false);
    intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
    chassis.drive_timeout = 1250;
    chassis.drive_max_voltage = 3;
    chassis.drive_distance(-20, 180);
    chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 43, 180);
    bottomTriStateP.set(false);
    topTriStateP.set(false);
    odom_constants();

    // Intake from the red left match loader
    matchLoadP.set(true);
    chassis.drive_min_voltage = 2;
    chassis.drive_max_voltage = 4.5;
    chassis.drive_timeout = 1175;
    chassis.drive_to_pose(23.5, -5, 180);
    chassis.turn_timeout = 400;
    chassis.turn_to_angle(180);
    wait(0.15, seconds);
    chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 12, 180);
    odom_constants();

    // Score into the mid goal
    chassis.drive_timeout = 1500;
    chassis.drive_to_point(67.5, 64.25);
    bottomTriStateP.set(false);
    topTriStateP.set(true);
    chassis.drive_max_voltage = 2;
    chassis.drive_distance(-10, -135);
    intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
}

void left_mid_elims() {
  chassis.set_coordinates(53.5, 20.5, -90);
  odom_constants();

  // Intake from left match loader
  chassis.drive_timeout = 1000;
  chassis.drive_to_point(21.5, 20);
  matchLoadP.set(true);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 20, 180);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 980;
  chassis.drive_to_point(24, 0);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 12, 180);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(21, 52);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 1275;
  chassis.drive_distance(-20, 180);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  matchLoadP.set(false);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 43, 180);
  odom_constants();

  // Descore
  chassis.drive_timeout = 1000;
  chassis.drive_to_pose(33, 32, 135);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(170);
  odom_constants();
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(27.5, 57.5, 180);
  wait(2, seconds); // 4.5sec for elims
  odom_constants();

  // Score into mid goal
  descoreP.set(true);
  wait(0.15, seconds);
  chassis.boomerang_lead = 0.1;
  chassis.drive_to_pose(42, 47, 135);
  chassis.turn_timeout = 550;
  chassis.turn_to_angle(-135);
  chassis.drive_timeout = 500;
  chassis.drive_distance(-15, -135);
  bottomTriStateP.set(false);
  topTriStateP.set(true);
  chassis.drive_max_voltage = 2;
  chassis.drive_distance(-10, -135);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  wait(1.5, seconds);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
}

void left_4ball_elims() {
  chassis.set_coordinates(53.5, 20.5, -90);
  odom_constants();

  // Intake from left match loader
  chassis.drive_timeout = 1000;
  chassis.drive_to_point(21.5, 20);
  matchLoadP.set(true);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 20, 180);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 900;
  chassis.drive_to_point(24, 5);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(23.5, 52);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 1100;
  chassis.drive_distance(-20, 180);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  matchLoadP.set(false);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 43, 180);
  odom_constants();

  // Descore
  chassis.drive_timeout = 1000;
  chassis.drive_to_pose(33, 32, 135);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(170);
  odom_constants();
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(28, 60, 180);
  wait(4.5, seconds);
  odom_constants();
}

void right_4ball_elims() {
  chassis.set_coordinates(90.5, 20.5, 90);
  odom_constants();

  // Intake from left match loader
  chassis.drive_timeout = 1000;
  chassis.drive_to_point(122.5, 20);
  matchLoadP.set(true);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 20, 180);
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 900;
  chassis.drive_to_point(120, 5);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(120.5, 52);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 1100;
  chassis.drive_distance(-20, 180);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  matchLoadP.set(false);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 43, 180);
  odom_constants();

  // Descore
  chassis.drive_timeout = 1000;
  chassis.drive_to_pose(135, 32, 135);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(170);
  odom_constants();
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(132, 60, 180);
  chassis.turn_to_angle(180);
}

void left_7ball_elims() {
  chassis.set_coordinates(56.5, 23.5, 0);
  odom_constants();

  // Intake from center stack
  intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_max_voltage = 7;
  chassis.drive_timeout = 970;
  chassis.drive_to_point(48, 48);
  chassis.turn_timeout = 400;
  chassis.turn_to_angle(-135);
  odom_constants();

  // Intake from left match loader
  chassis.drive_timeout = 950;
  chassis.drive_to_point(22, 22);
  matchLoadP.set(true);
  chassis.turn_timeout = 425;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 22, 180);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 1050;
  chassis.drive_to_point(24, 5);
  odom_constants();

  // Score into long goal
  chassis.drive_timeout = 780;
  chassis.drive_to_point(23.5, 52);
  bottomTriStateP.set(true);
  topTriStateP.set(false);
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 1750;
  chassis.drive_distance(-20, 180);
  bottomTriStateP.set(false);
  topTriStateP.set(false);
  matchLoadP.set(false);
  intakeMotors.stop();
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 43, 180);
  odom_constants();

  // Descore
  chassis.drive_timeout = 950;
  chassis.drive_to_pose(33, 32, 135);
  chassis.turn_timeout = 400;
  chassis.turn_to_angle(170);
  odom_constants();
  chassis.drive_timeout = 950;
  chassis.drive_to_pose(27.75, 58, 180);
  chassis.turn_to_angle(180);
  odom_constants();
}
