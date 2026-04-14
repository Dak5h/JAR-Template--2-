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
  chassis.set_drive_exit_conditions(1.5, 300, 5000);
  chassis.set_turn_exit_conditions(1, 300, 3000);
  chassis.set_swing_exit_conditions(1, 300, 3000);
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
  chassis.swing_max_voltage = 12;
  chassis.drive_min_voltage = 0;
  chassis.drive_settle_error = 3;
  chassis.boomerang_lead = .5;
  chassis.swing_timeout = 3000;
  chassis.turn_timeout = 3000;
  chassis.drive_timeout = 5000;
}

void elims_right_low_split() {
  odom_constants();
  chassis.set_coordinates(86, 23, 0);

  // Intake center stack
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1000;
  chassis.boomerang_lead = 0.4;
  chassis.drive_to_pose(96, 48, 55);

  // Intake under long goal
  chassis.drive_max_voltage = 6;
  chassis.boomerang_lead = 0.3;
  chassis.drive_to_pose(118.5, 59.2, 85);
  matchLoadP.set(true);
  wait(125, msec);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(0);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceBack.objectDistance(inches) + 5, 144 - DistanceRight.objectDistance(inches) - 5.5, 0);

  // Score blocks into long goal
  chassis.drive_timeout = 670;
  chassis.drive_to_pose(117, 54, -5);
  matchLoadP.set(false);
  chassis.swing_timeout = 1000;
  chassis.left_swing_to_angle(180);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 200;
  chassis.drive_max_voltage = 12;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 725;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);

  // Intake from match loader
  chassis.drive_min_voltage = 3.5;
  matchLoadP.set(true);
  chassis.drive_to_pose(120, 15, 180);
  chassis.drive_max_voltage = 2.5;
  chassis.drive_timeout = 1000;
  chassis.drive_to_pose(120, 10, 180);
  odom_constants();
  chassis.drive_distance(-12, 180);
  BottomIntake.stop();

  // Score into low goal
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(-45);
  matchLoadP.set(false);
  chassis.drive_timeout = 1400;
  chassis.drive_to_pose(80.5, 61, -45);
  intakeLiftP.set(true);
  BottomIntake.spin(reverse, 100, vex::velocityUnits::pct);
  wait(1000, msec);
  intakeLiftP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Decore long goal
  chassis.drive_to_point(104.75, 43);
  chassis.turn_to_angle(200);
  chassis.drive_distance(-17);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(170);
}

void elims_right_7ball() {
  odom_constants();
  chassis.set_coordinates(86, 23, 0);

  // Intake center stack
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1000;
  chassis.boomerang_lead = 0.4;
  chassis.drive_to_pose(96, 48, 55);

  // Intake from match loader
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(135);
  chassis.drive_to_point(125, 15);
  matchLoadP.set(true);
  chassis.turn_to_angle(180);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 20, 180);
  chassis.drive_min_voltage = 2.5;
  chassis.drive_max_voltage = 2.5;
  chassis.drive_timeout = 1500;
  chassis.drive_to_pose(120, 0, 180);
  odom_constants();
  
  // Score into long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_pose(120, 50, 180);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 400;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 850;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);

  // Descore
  chassis.drive_to_point(110, 36);
  chassis.turn_to_angle(190);
  chassis.drive_distance(-22);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(170);

}