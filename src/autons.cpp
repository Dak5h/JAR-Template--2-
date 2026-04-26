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
  chassis.set_coordinates(85.25, 20.5, 90);

  // Intake from right match loader
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 920;
  chassis.drive_to_point(121, 20);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(180);
  chassis.drive_min_voltage = 5;
  chassis.drive_max_voltage = 5;
  chassis.drive_timeout = 800;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(121, 0, 180);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 700;
  chassis.drive_to_point(122, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1100;
  chassis.drive_max_voltage = 3;
  chassis.drive_distance(-20, 180);
  ballLockP.set(false);
  matchLoadP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);

  // Descore
  chassis.drive_timeout = 950;
  chassis.drive_to_point(110.6, 36);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(190);
  chassis.drive_timeout = 1400;
  chassis.drive_max_voltage = 6;
  chassis.drive_distance(-24.5);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(180);
  odom_constants();
  wait(2800, msec);
  wingP.set(true);
  wait(150, msec);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);

  // Intake center stack
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(-140);
  chassis.drive_timeout = 800;
  chassis.drive_to_point(96, 45);
  matchLoadP.set(true);
  wingP.set(false);

  // Score low goal
  BottomIntake.stop();
  chassis.turn_to_angle(-42);
  matchLoadP.set(false);
  wait(100, msec);
  chassis.drive_distance(17);
  intakeLiftP.set(true);
  BottomIntake.spin(reverse, 80, vex::velocityUnits::pct);
  wait(500, msec);
  intakeLiftP.set(false);
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
  chassis.turn_timeout = 470;
  chassis.turn_to_angle(135);
  matchLoadP.set(true);
  chassis.drive_timeout = 1150;
  chassis.drive_to_point(126, 15);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 20, 180);
  chassis.drive_min_voltage = 4.5;
  chassis.drive_max_voltage = 4.5;
  chassis.drive_timeout = 870;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(120, 0, 180);
  odom_constants();
  
  // Score into long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(120, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 850;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 800;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);

  // Descore
  chassis.drive_timeout = 800;
  chassis.drive_to_point(110, 36);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(193);
  chassis.drive_distance(-24.5);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(180);
}

void quals_counter_soloawp() {
  odom_constants();
  chassis.set_coordinates(85.25, 20.5, 90);

  // Intake from right match loader
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 900;
  chassis.drive_to_point(120.5, 20);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(180);
  chassis.drive_min_voltage = 6;
  chassis.drive_max_voltage = 6;
  chassis.drive_timeout = 850;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(120.5, 0, 180);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 700;
  chassis.drive_to_point(120.75, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 400;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 700;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  wait(0.1, msec);
  odom_constants();

  // Reset Pose
  chassis.turn_timeout = 800;
  chassis.turn_to_angle(270);
  chassis.set_coordinates(144 - DistanceBack.objectDistance(inches) - 5, DistanceLeft.objectDistance(inches) + 5.5, -90);

  // Intake right center stack
  chassis.drive_timeout = 600;
  chassis.drive_to_point(96, 47);
  chassis.drive_timeout = 1350;
  chassis.drive_to_point(45, 51);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(-135);
  odom_constants();

  // Reset Pose
  chassis.drive_timeout = 1100;
  chassis.drive_to_point(14, 28);
  BottomIntake.stop();
  chassis.turn_timeout = 470;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5, 28, 180);

  // Score into long goal
  chassis.drive_timeout = 550;
  chassis.drive_to_point(23.7, 50);
  ballLockP.set(true);
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 550;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 850;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 42.75, 180);

  // Intake from match loader
  chassis.drive_min_voltage = 4;
  chassis.drive_to_pose(24, 9.5, 180);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 120;
  chassis.drive_to_pose(24, 0, 180);
  odom_constants();
  chassis.drive_distance(-12, 180);
  matchLoadP.set(false);

  // Score into mid goal
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(-135);
  BottomIntake.stop();
  matchLoadP.set(false);
  chassis.drive_timeout = 1250;
  chassis.drive_to_point(66.5, 65);
  angleChangeP.set(true);
  BottomIntake.spin(forward, 50, vex::velocityUnits::pct);
  chassis.drive_timeout = 1200;
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 2;
  chassis.drive_distance(-20, -135);
  BottomIntake.stop();
  angleChangeP.set(false);
  odom_constants();
}

void quals_regular_soloawp() {
  odom_constants();
  chassis.set_coordinates(85.25, 20.5, 90);

  // Intake from right match loader
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 920;
  chassis.drive_to_point(121, 20);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(180);
  chassis.drive_min_voltage = 4;
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 820;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(121, 0, 180);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 700;
  chassis.drive_to_point(121, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 520;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 700;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  odom_constants();

  // Reset Pose
  chassis.turn_timeout = 800;
  chassis.turn_to_angle(270);
  chassis.set_coordinates(144 - DistanceBack.objectDistance(inches) - 5, DistanceLeft.objectDistance(inches) + 5.5, -90);

  // Intake right center stack
  chassis.drive_timeout = 600;
  chassis.drive_to_point(96, 47);
  chassis.drive_timeout = 1600;
  chassis.drive_to_point(43, 50);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(-140);
  odom_constants();

  // Score into mid goal
  BottomIntake.stop();
  matchLoadP.set(false);
  chassis.drive_timeout = 675;
  chassis.drive_to_point(59, 67);
  angleChangeP.set(true);
  BottomIntake.spin(forward, 40, vex::velocityUnits::pct);
  chassis.drive_timeout = 650;
  chassis.drive_min_voltage = 2;
  chassis.drive_max_voltage = 2;
  chassis.drive_distance(-20, -135);
  BottomIntake.stop();
  angleChangeP.set(false);
  odom_constants();
  chassis.set_coordinates(64, 64, -135);

  // Reset Pose
  matchLoadP.set(true);
  chassis.drive_timeout = 1250;
  chassis.drive_to_point(20, 24);
  chassis.turn_timeout = 430;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 20, 180);

  // Intake from match loader
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 4;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(23.75, 9, 180);
  chassis.drive_max_voltage = 4;
  chassis.drive_timeout = 60;
  chassis.drive_to_pose(23.75, 0, 180);
  odom_constants();

  // Score into long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(23.25, 50);
  ballLockP.set(true);
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 550;
  chassis.drive_distance(-25, 180);
  chassis.turn_timeout = 850;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 42.75, 180);

  // Decore long goal
  chassis.drive_to_point(13.75, 34);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(180);
  chassis.drive_min_voltage = 4;
  chassis.drive_distance(-23);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(170);
}

void elims_left_7ball() {
  odom_constants();
  chassis.set_coordinates(58, 23, 0);

  // Intake center stack
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1000;
  chassis.boomerang_lead = 0.4;
  chassis.drive_to_pose(48, 48, -55);

  // Intake from match loader
  chassis.turn_timeout = 470;
  chassis.turn_to_angle(-135);
  matchLoadP.set(true);
  chassis.drive_timeout = 1150;
  chassis.drive_to_point(18, 15);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 20, 180);
  chassis.drive_min_voltage = 4.5;
  chassis.drive_max_voltage = 4.5;
  chassis.drive_timeout = 950;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(24, 0, 180);
  odom_constants();
  
  // Score into long goal
  chassis.drive_timeout = 800;
  chassis.drive_to_point(23, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 850;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 850;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 42.75, 180);

  // Decore long goal
  chassis.drive_to_point(12.7, 33);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(172);
  chassis.drive_min_voltage = 4;
  chassis.drive_distance(-23);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(180);
}

void elims_left_4ball() {
  odom_constants();
  chassis.set_coordinates(58.75, 20.5, -90);

  // Intake from right match loader
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 920;
  chassis.drive_to_point(24, 20);
  chassis.turn_timeout = 440;
  chassis.turn_to_angle(180);
  chassis.drive_min_voltage = 3.5;
  chassis.drive_max_voltage = 3.5;
  chassis.drive_timeout = 800;
  chassis.boomerang_lead = 0.2;
  chassis.drive_to_pose(24, 0, 180);
  odom_constants();

  // Score into right long goal
  chassis.drive_timeout = 700;
  chassis.drive_to_point(24, 50);
  ballLockP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 500;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 700;
  chassis.turn_to_angle(180);
  ballLockP.set(false);
  matchLoadP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 42.75, 180);

  // Decore long goal
  chassis.drive_to_point(12.7, 33);
  BottomIntake.stop();
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(172);
  chassis.drive_min_voltage = 4;
  chassis.drive_distance(-23);
  chassis.turn_timeout = 500;
  chassis.turn_to_angle(140);
  matchLoadP.set(true);
}

void prog_skills() {
  odom_constants();
  chassis.set_coordinates(58, 23, 0);

  // Intake left center stack
  wingP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1100;
  chassis.drive_max_voltage = 7;
  chassis.boomerang_lead = 0.4;
  chassis.drive_to_pose(48, 48, -55);
  matchLoadP.set(true);

  // Score into mid goal
  chassis.turn_timeout = 470;
  chassis.turn_to_angle(-135);
  chassis.drive_timeout = 750;
  chassis.drive_to_pose(60, 60, -135);
  angleChangeP.set(true);
  BottomIntake.spin(forward, 30, vex::velocityUnits::pct);
  chassis.drive_timeout = 370;
  chassis.drive_min_voltage = 1;
  chassis.drive_max_voltage = 1;
  chassis.drive_distance(-20, -135);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  BottomIntake.spin(reverse, 10, vex::velocityUnits::pct);
  chassis.drive_to_point(17.5, 20);
  BottomIntake.spin(reverse, 100, vex::velocityUnits::pct);
  chassis.turn_timeout = 450;
  chassis.turn_to_angle(180);
  angleChangeP.set(false);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.set_coordinates(DistanceRight.objectDistance(inches) + 5.5, 20, 180);

  // Intake from match loader
  chassis.drive_min_voltage = 3.5;
  chassis.boomerang_lead = 0.1;
  chassis.drive_timeout = 1800;
  chassis.drive_max_voltage = 3.5;
  chassis.drive_to_pose(23, -5, 180);
  odom_constants();

  // Go to other side
  chassis.drive_to_point(10, 42);
  thread b([] {
    wait(0.6, seconds);
    BottomIntake.stop();
    matchLoadP.set(false);
  });
  chassis.drive_to_point(8, 105);

  // Score into blue left long goal
  chassis.drive_max_voltage = 5;
  chassis.drive_to_point(21, 112);
  chassis.turn_to_angle(0);
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 112, 0);
  chassis.drive_timeout = 650;
  chassis.drive_to_point(22.5, 93);
  ballLockP.set(true);
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1000;
  chassis.drive_distance(-20, 0);
  chassis.drive_distance(-20);
  ballLockP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 101.25, 0);

  // Intake from match loader
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 3.5;
  chassis.drive_timeout = 1000;
  chassis.drive_to_pose(23.5, 132.5, 0);
  chassis.drive_min_voltage = 4;
  chassis.drive_max_voltage = 4;
  chassis.drive_to_pose(23.5, 144, 0);
  odom_constants();

  // Score into blue left long goal again
  chassis.drive_timeout = 800;
  chassis.drive_to_point(23, 93);
  ballLockP.set(true);
  matchLoadP.set(false);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1000;
  chassis.drive_distance(-20, 0);
  chassis.drive_distance(-20);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(DistanceLeft.objectDistance(inches) + 5.5, 101.25, 0);
  wingP.set(true);
  odom_constants();

  // Intake red blocks from blue park zone
  chassis.drive_min_voltage = 3;
  chassis.drive_max_voltage = 6;
  chassis.boomerang_lead = 0.5;
  chassis.drive_timeout = 1450;
  chassis.drive_to_pose(52, 136, 76);
  odom_constants();
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  ballLockP.set(false);
  chassis.drive_min_voltage = 9;
  chassis.drive_max_voltage = 9;
  chassis.drive_distance(82, 80);
  odom_constants();
  chassis.turn_to_angle(180);
  wait(200, msec);

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 144 - DistanceBack.objectDistance(inches) - 5, 180);
  odom_constants();

  // Score into mid goal
  BottomIntake.stop();
  chassis.drive_to_point(77.1, 91);
  chassis.turn_to_angle(-45);
  chassis.swing_timeout = 600;
  chassis.right_swing_to_angle(45);
  chassis.drive_max_voltage = 3.5;
  chassis.drive_timeout = 800;
  chassis.drive_distance(-10, 45);
  chassis.drive_min_voltage = 3;
  chassis.drive_max_voltage = 3;
  chassis.drive_timeout = 250;
  chassis.drive_distance(3, 45);
  chassis.set_coordinates(86, 86, 45);
  odom_constants();
  angleChangeP.set(true);
  wait(50, msec);
  BottomIntake.spin(forward, 20, vex::velocityUnits::pct);
  wait(3500, msec);

  // Go to long goal while intaking center blocks
  odom_constants();
  wingP.set(true);
  thread t([] {
    wait(0.2, seconds);
    BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
    angleChangeP.set(false);
  });
  chassis.drive_max_voltage = 7;
  chassis.drive_to_point(128, 120);
  chassis.turn_to_angle(0);
  wait(100, msec);
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, 120, 0);
  chassis.drive_timeout = 500;
  chassis.drive_to_point(120, 93);
  ballLockP.set(true);
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 200;
  chassis.drive_distance(-20, 0);
  chassis.turn_timeout = 400;
  chassis.turn_to_angle(0);
  chassis.drive_timeout = 600;
  chassis.drive_distance(-20, 0);
  ballLockP.set(false);
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceRight.objectDistance(inches) - 5.5, 101.25, 0);

  // Intake from match loader
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1150;
  chassis.drive_min_voltage = 3;
  chassis.drive_to_pose(121, 131.25, 0);
  chassis.drive_max_voltage = 3;
  chassis.drive_to_pose(121, 144, 0);
  odom_constants();

  // Go to other side
  chassis.drive_to_point(134, 102);
  thread c([] {
    wait(0.6, seconds);
    BottomIntake.stop();
    matchLoadP.set(false);
  });
  chassis.drive_to_point(135, 39);

  // Score into red right long goal
  chassis.drive_max_voltage = 5;
  chassis.drive_to_point(121.5, 32);
  chassis.turn_to_angle(180);
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 32, 180);
  chassis.drive_timeout = 650;
  chassis.drive_to_point(121.5, 50);
  ballLockP.set(true);
  matchLoadP.set(true);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 650;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 750;
  chassis.turn_to_angle(180);
  chassis.drive_timeout = 800;
  chassis.drive_distance(-20, 180);
  ballLockP.set(false);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);

  // Intake from match loader
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 3.5;
  chassis.drive_timeout = 1150;
  chassis.drive_to_pose(120, 11.5, 180);
  chassis.drive_max_voltage = 3.5;
  chassis.drive_to_pose(120, 0, 180);
  odom_constants();

  // Score into blue left long goal again
  chassis.drive_timeout = 800;
  chassis.drive_to_point(122.5, 50);
  ballLockP.set(true);
  matchLoadP.set(false);
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 600;
  chassis.drive_distance(-20, 180);
  chassis.turn_timeout = 700;
  chassis.turn_to_angle(180);
  chassis.drive_distance(-20, 180);
  BottomIntake.stop();
  odom_constants();

  // Reset Pose
  odom_constants();
  chassis.set_coordinates(144 - DistanceLeft.objectDistance(inches) - 5.5, 42.75, 180);
  chassis.turn_timeout = 800;
  chassis.turn_to_angle(270);
  ballLockP.set(false);
  chassis.set_coordinates(144 - DistanceBack.objectDistance(inches) - 5, DistanceLeft.objectDistance(inches) + 5.5, -90);

  // Intake right center stack
  BottomIntake.spin(forward, 100, vex::velocityUnits::pct);
  chassis.drive_timeout = 1200;
  chassis.drive_max_voltage = 5;
  chassis.drive_to_point(93.2, 47);
  chassis.turn_to_angle(-45);
  BottomIntake.stop();
  chassis.drive_distance(18, -45);
  intakeLiftP.set(true);
  BottomIntake.spin(reverse, 60, vex::velocityUnits::pct);
  wait(800, msec);
  odom_constants();

  // Park
  chassis.swing_max_voltage = 6;
  chassis.swing_timeout = 600;
  chassis.right_swing_to_angle(-8);
  BottomIntake.stop();
  intakeLiftP.set(false);
  chassis.drive_to_point(96, 14);
  chassis.left_swing_to_angle(-95);
  BottomIntake.spin(reverse, 100, vex::velocityUnits::pct);
  chassis.drive_min_voltage = 8.5;
  chassis.drive_max_voltage = 8.5;
  chassis.drive_distance(43, -93);
}