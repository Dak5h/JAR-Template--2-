#include "vex.h"

using namespace vex;
competition Competition;

/*---------------------------------------------------------------------------*/
/*                             VEXcode Config                                */
/*                                                                           */
/*  Before you do anything else, start by configuring your motors and        */
/*  sensors. In VEXcode Pro V5, you can do this using the graphical          */
/*  configurer port icon at the top right. In the VSCode extension, you'll   */
/*  need to go to robot-config.cpp and robot-config.h and create the         */
/*  motors yourself by following the style shown. All motors must be         */
/*  properly reversed, meaning the drive should drive forward when all       */
/*  motors spin forward.                                                     */
/*---------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------*/
/*                             JAR-Template Config                           */
/*                                                                           */
/*  Where all the magic happens. Follow the instructions below to input      */
/*  all the physical constants and values for your robot. You should         */
/*  already have configured your motors.                                     */
/*---------------------------------------------------------------------------*/

Drive chassis(

//Pick your drive setup from the list below:
//ZERO_TRACKER_NO_ODOM
//ZERO_TRACKER_ODOM
//TANK_ONE_FORWARD_ENCODER
//TANK_ONE_FORWARD_ROTATION
//TANK_ONE_SIDEWAYS_ENCODER
//TANK_ONE_SIDEWAYS_ROTATION
//TANK_TWO_ENCODER
//TANK_TWO_ROTATION
//HOLONOMIC_TWO_ENCODER
//HOLONOMIC_TWO_ROTATION
//
//Write it here:
TANK_ONE_FORWARD_ROTATION,

//Add the names of your Drive motors into the motor groups below, separated by commas, i.e. motor_group(Motor1,Motor2,Motor3).
//You will input whatever motor names you chose when you configured your robot using the sidebar configurer, they don't have to be "Motor1" and "Motor2".

//Left Motors:
motor_group(LeftFront, LeftMiddle, LeftBack),

//Right Motors:
motor_group(RightFront, RightMiddle, RightBack),

//Specify the PORT NUMBER of your inertial sensor, in PORT format (i.e. "PORT1", not simply "1"):
PORT16,

//Input your wheel diameter. (4" omnis are actually closer to 4.125"):
4,

//External ratio, must be in decimal, in the format of input teeth/output teeth.
//If your motor has an 84-tooth gear and your wheel has a 60-tooth gear, this value will be 1.4.
//If the motor drives the wheel directly, this value will be 1:
0.429,

//Gyro scale, this is what your gyro reads when you spin the robot 360 degrees.
//For most cases 360 will do fine here, but this scale factor can be very helpful when precision is necessary.
360,

/*---------------------------------------------------------------------------*/
/*                                  PAUSE!                                   */
/*                                                                           */
/*  The rest of the drive constructor is for robots using POSITION TRACKING. */
/*  If you are not using position tracking, leave the rest of the values as  */
/*  they are.                                                                */
/*---------------------------------------------------------------------------*/

//If you are using ZERO_TRACKER_ODOM, you ONLY need to adjust the FORWARD TRACKER CENTER DISTANCE.

//FOR HOLONOMIC DRIVES ONLY: Input your drive motors by position. This is only necessary for holonomic drives, otherwise this section can be left alone.
//LF:      //RF:    
PORT1,     -PORT2,

//LB:      //RB: 
PORT3,     -PORT4,

//If you are using position tracking, this is the Forward Tracker port (the tracker which runs parallel to the direction of the chassis).
//If this is a rotation sensor, enter it in "PORT1" format, inputting the port below.
//If this is an encoder, enter the port as an integer. Triport A will be a "1", Triport B will be a "2", etc.
PORT15,

//Input the Forward Tracker diameter (reverse it to make the direction switch):
-2.2675,

//Input Forward Tracker center distance (a positive distance corresponds to a tracker on the right side of the robot, negative is left.)
//For a zero tracker tank drive with odom, put the positive distance from the center of the robot to the right side of the drive.
//This distance is in inches:
0.25,

//Input the Sideways Tracker Port, following the same steps as the Forward Tracker Port:
PORT17,

//Sideways tracker diameter (reverse to make the direction switch):
2.00,

//Sideways tracker center distance (positive distance is behind the center of the robot, negative is in front):
2.00
);

int current_auton_selection = 2;
bool auto_started = false;

void toggleDescoreP(){
  descoreP.set(!descoreP.value());
}

void toggleMatchLoadP(){
  matchLoadP.set(!matchLoadP.value());
}



/**
 * Function before autonomous. It prints the current auton on the brain screen.
 * Use the controller D-pad to select: Right = next auton, Left = previous auton.
 * Add anything else you may need, like resetting pneumatic components.
 */

void pre_auton() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  default_constants();

  while(!auto_started){
    Brain.Screen.clearScreen();
    Brain.Screen.printAt(5, 20, "JAR Template v1.2.0");
    Brain.Screen.printAt(5, 40, "Battery Percentage:");
    Brain.Screen.printAt(5, 60, "%d", Brain.Battery.capacity());
    Brain.Screen.printAt(5, 80, "Chassis Heading Reading:");
    Brain.Screen.printAt(5, 100, "%f", chassis.get_absolute_heading());
    Brain.Screen.printAt(5, 120, "Selected Auton (D-pad L/R):");
    const char* auton_name = "";
    switch(current_auton_selection){
      case 0:
        auton_name = "prog_skills";
        break;
      case 1:
        auton_name = "solo_awp_counter";
        break;
      case 2:
        auton_name = "left_mid_elims";
        break;
      case 3:
        auton_name = "left_4ball_elims";
        break;
      case 4:
        auton_name = "right_4ball_elims";
        break;
      case 5:
        auton_name = "left_7ball_elims";
        break;
    }
    Brain.Screen.printAt(5, 140, false, "%s", auton_name);

    // Controller: selected auton + live inertial
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Auton: %s", auton_name);
    Controller1.Screen.setCursor(2, 1);
    Controller1.Screen.print("Head: %.1f", chassis.get_absolute_heading());
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("P: %.1f R: %.1f", (float)chassis.Gyro.pitch(deg), (float)chassis.Gyro.roll(deg));

    // Controller D-pad: Right = next auton, Left = previous auton
    if (Controller1.ButtonRight.pressing()) {
      while (Controller1.ButtonRight.pressing()) { task::sleep(10); }
      current_auton_selection++;
      if (current_auton_selection > 5) current_auton_selection = 0;
    } else if (Controller1.ButtonLeft.pressing()) {
      while (Controller1.ButtonLeft.pressing()) { task::sleep(10); }
      current_auton_selection--;
      if (current_auton_selection < 0) current_auton_selection = 5;
    }
    task::sleep(10);
  }
}

/**
 * Auton function, which runs the selected auton. Case 0 is the default,
 * and will run in the brain screen goes untouched during preauton. Replace
 * drive_test(), for example, with your own auton function you created in
 * autons.cpp and declared in autons.h.
 */

void autonomous(void) {
  auto_started = true;
  switch(current_auton_selection) { 
    case 0:
      prog_skills();
      break;

    case 1:
      solo_awp_counter();
      break;

    case 2:
      left_mid_elims();
      break;

    case 3:
      left_4ball_elims();
      break;

    case 4:
      right_4ball_elims();
      break;

    case 5:
      left_7ball_elims();
      break;
 }
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // B toggles pistons to (bottom=false, top=true). Persists until L1/L2/R1 is pressed.
  static bool bPistonsToggled = false;
  bool lastBPressed = false;

  // User control code here, inside the loop
  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................

    //Replace this line with chassis.control_tank(); for tank drive 
    //or chassis.control_holonomic(); for holo drive.
    chassis.control_arcade();

    // B toggle: press once to set pistons, persists until L1/L2/R1 overrides
    bool bPressed = Controller1.ButtonB.pressing();
    if (bPressed && !lastBPressed) {
      bPistonsToggled = !bPistonsToggled;
    }
    lastBPressed = bPressed;

    // Intake control - L1/L2 or B toggle false = normal intake
    if(Controller1.ButtonL1.pressing()) {
      bPistonsToggled = false;
      bottomTriStateP.set(false);
      topTriStateP.set(false);
      intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
    } else if(Controller1.ButtonL2.pressing()) {
      bPistonsToggled = false;
      bottomTriStateP.set(false);
      topTriStateP.set(false);
      intakeMotors.spin(reverse, 100, vex::velocityUnits::pct);
    } else if(Controller1.ButtonR1.pressing()){
      if (bPistonsToggled) {
        bottomTriStateP.set(false);
        topTriStateP.set(true);
        intakeMotors.spin(forward, 40, vex::velocityUnits::pct);
      } else {
        bottomTriStateP.set(true);
        topTriStateP.set(false);
        intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
      }
    } else if(Controller1.ButtonR2.pressing()) {
      bottomTriStateP.set(false);
      topTriStateP.set(true);
      intakeMotors.spin(forward, 100, vex::velocityUnits::pct);
    } else if(bPistonsToggled) {
      bottomTriStateP.set(false);
      topTriStateP.set(true);
      intakeMotors.stop();
    } else {
      bottomTriStateP.set(false);
      topTriStateP.set(false);
      intakeMotors.stop();
    }

    // Angle change control
    Controller1.ButtonX.pressed(toggleDescoreP);
    Controller1.ButtonA.pressed(toggleMatchLoadP);


    wait(10, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}

