#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen.
brain  Brain;
controller Controller1 = controller(primary);

// Drive Motors
motor LeftFront = motor(PORT12, ratio6_1, true);
motor LeftMiddle = motor(PORT13, ratio6_1, true);
motor LeftBack = motor(PORT14, ratio6_1, true);
motor RightFront = motor(PORT19, ratio6_1, false);
motor RightMiddle = motor(PORT18, ratio6_1, false);
motor RightBack = motor(PORT17, ratio6_1, false);

// Mech Motors
motor LeftIntakeMotor = motor(PORT11, ratio6_1, false);
motor RightIntakeMotor = motor(PORT20, ratio6_1, true);
motor_group intakeMotors = motor_group(LeftIntakeMotor, RightIntakeMotor);

// Sensors
distance DistanceLeft = distance(PORT1);
distance DistanceRight = distance(PORT10);
distance DistanceBack = distance(PORT2);

// Pneumatics
pneumatics matchLoadP = pneumatics(Brain.ThreeWirePort.A);
pneumatics descoreP = pneumatics(Brain.ThreeWirePort.B);
pneumatics midDescoreP = pneumatics(Brain.ThreeWirePort.C);
pneumatics bottomTriStateP = pneumatics(Brain.ThreeWirePort.D);
pneumatics topTriStateP = pneumatics(Brain.ThreeWirePort.E);

void vexcodeInit( void ) {
  // nothing to initialize
}