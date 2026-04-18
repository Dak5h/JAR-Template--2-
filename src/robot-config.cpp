#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller1 = controller(primary);

// Drive Motors
motor LeftFront = motor(PORT11, ratio6_1, true);
motor LeftMiddle = motor(PORT12, ratio6_1, true);
motor LeftBack = motor(PORT13, ratio6_1, true);
motor RightFront = motor(PORT15, ratio6_1, false);
motor RightMiddle = motor(PORT16, ratio6_1, false);
motor RightBack = motor(PORT17, ratio6_1, false);

// Mech Motors
motor BottomFullIntake = motor(PORT14, ratio6_1, false);
motor BottomHalfIntake = motor(PORT2, ratio6_1, true);
motor_group BottomIntake = motor_group(BottomFullIntake, BottomHalfIntake);

// Sensors
distance DistanceLeft = distance(PORT1);
distance DistanceRight = distance(PORT18);
distance DistanceBack = distance(PORT20);

// Pneumatics
pneumatics angleChangeP = pneumatics(Brain.ThreeWirePort.B);
pneumatics ballLockP = pneumatics(Brain.ThreeWirePort.A);
pneumatics wingP = pneumatics(Brain.ThreeWirePort.C);
pneumatics intakeLiftP = pneumatics(Brain.ThreeWirePort.D);
pneumatics matchLoadP = pneumatics(Brain.ThreeWirePort.E);
pneumatics midDescoreP = pneumatics(Brain.ThreeWirePort.F);
void vexcodeInit( void ) {
  // nothing to initialize
}