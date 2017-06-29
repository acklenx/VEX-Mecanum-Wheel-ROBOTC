// GLOBAL CONSTANTS
static const short MODE_EXIT 					= 0;
static const short MODE_REMOTECONTROL = 1;
static const short MODE_DRIVETEST 		= 2;
static const short MODE_TRACKLINE			= 3;
static const short MODE_BEHAVIORAL 		= 4;
static const short MODE_DISCOVERY 		= 5;
static const short MODE_MAPPING 			= 6;
static const short MODE_DEFENSIVE			= 7;
static const short MODE_DECIDING			= 100;
static short ROBOT_MODE 							= MODE_DECIDING;

//==========================================================
// GLOBAL VARIABLES FOR SPEED
//==========================================================
static const short SPEED_FRONT_DEFAULT	= 30;
static const short SPEED_REAR_DEFAULT 	= 30;
static const short SPEED_RIGHT_DEFAULT 	= 40;
static const short SPEED_LEFT_DEFAULT 	= 40;
static const short SPEED_ROTATE_DEFAULT = 30;

//==========================================================
// GLOBAL VARIABLES FOR DEFENSIVE MODE
//==========================================================
static const short DEFENSE_FRONT_THRESHOLD 	= 18;
static const short DEFENSE_REAR_THRESHOLD 	= 12;
static const short DEFENSE_RIGHT_THRESHOLD 	= 12;
static const short DEFENSE_LEFT_THRESHOLD 	= 12;

//==========================================================
// THE GLOBAL VARIABLES FOR MOVING DIRECTION
// THERE ARE 8 DIRECTIONS OF TRAVEL,
// THESE CAN BE USED FOR NAVIGATIONAL PURPOSES.
// NUMERIC VALUES CORRESPOND TO DEGREES FROM A
// NORMAL X Y CARTESIAN COORD SYSTEM
//==========================================================
static const short DIRECTION_RIGHT 				= 0;
static const short DIRECTION_RIGHT_FRONT 	= 45;
static const short DIRECTION_FRONT 				= 90;
static const short DIRECTION_LEFT_FRONT 	= 135;
static const short DIRECTION_LEFT 				= 180;
static const short DIRECTION_LEFT_REAR 		= 225;
static const short DIRECTION_REAR 				= 270;
static const short DIRECTION_RIGHT_REAR 	= 315;
static short DIRECTION 										= 90; // front is default

//==========================================================
// GLOBAL CONSTANTS FOR LATERAL OR OBLIQUE MOVEMENT
// CONVERSION OF WHEEL TO ROBOT MOVEMENT
//==========================================================
static const float MOVEMENT_LINEAR_ADJUSTER 	= 21.0;
static const float MOVEMENT_LATERAL_ADJUSTER 	= 0.45;
static const float MOVEMENT_OBLIQUE_ADJUSTER 	= 0.45;

//==========================================================
//  PAUSE TIMES
//==========================================================
static const int PAUSETIME = 150;

//==========================================================
//  TIMER LIMITS IN MILLISECONDS
//==========================================================
static const short 	BEHAVIORAL_TIME_LIMIT = 2000;
static const int 		IEC_ERROR_TIMEOUT			= 1000;

//==========================================================
//  SENSOR VALUES
//==========================================================
static int sonarFrontValGlobal;
static int sonarRearValGlobal;
static int sonarRightValGlobal;
static int sonarLeftValGlobal;
static int lineFollower1ValGlobal;
static int lineFollower2ValGlobal;
static int lineFollower3ValGlobal;
static int bFrontBumperPressed;
