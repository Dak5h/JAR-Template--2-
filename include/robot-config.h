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
extern motor BottomFullIntake;
extern motor BottomHalfIntake;
extern motor_group BottomIntake;

// Sensors
extern optical Blockhold;
extern distance DistanceLeft;
extern distance DistanceRight;
extern distance DistanceBack;
extern distance lowBlock;
extern optical highBlock;
extern optical parkZone;

// Pneumatics
extern pneumatics angleChangeP;
extern pneumatics ballLockP;
extern pneumatics wingP;
extern pneumatics intakeLiftP;
extern pneumatics matchLoadP;
extern pneumatics midDescoreP;
void  vexcodeInit( void );