using namespace vex;

extern brain Brain;
extern controller Controller1;

// Drive Motors
extern motor LeftFront;
extern motor LeftMiddle;
extern motor LeftBack;
extern motor RightFront;
extern motor RightMiddle;
extern motor RightBack;

// Mech Motors
extern motor LeftIntakeMotor;
extern motor RightIntakeMotor;
extern motor_group intakeMotors;

// Sensors
extern optical Blockhold;
extern distance DistanceLeft;
extern distance DistanceRight;
extern distance DistanceBack;
extern distance lowBlock;
extern optical highBlock;
extern optical parkZone;

// Pneumatics
extern pneumatics matchLoadP;
extern pneumatics descoreP;
extern pneumatics midDescoreP;
extern pneumatics bottomTriStateP;
extern pneumatics topTriStateP;
void  vexcodeInit( void );