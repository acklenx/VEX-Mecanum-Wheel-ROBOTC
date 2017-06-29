/*
	LCDManager.h

	This header file declares and defines all the functions necessary
	for interaction with the VEX LCD component.
	-	Display Menu Options
	-	Handle input from LCD buttons

	LCD MENU FLOW
	Exit <- Initial[RemoteControlMode] <-> DriveTest <-> TrackLine <-> Behavioral
	<-> DiscoveryMode <-> MappingMode -> Initial[RemoteControlMode] -> ...

	This is the menu order and list, which maps to the MODE
	There will be a starte diagram for the MODE
	0	Exit
	1	Initial[RemoteControlMode]  ... Menu Starts Here
	2	DriveTest
	3	TrackLine
	4	Behavioral
	5	DiscoveryMode
	6	MappingMode
	7 DefensiveeMode

	LCD Button Pressed Numeric Input
	0:  No buttons pressed
	1:  Left button is pressed
	2:  Center button is pressed
	3:  Left and Center buttons are pressed
	4:  Right button is pressed
	5:  Left and Right buttons are pressed
	6:  Center and Right buttons are pressed
	7:  Left, Center, and Right buttons are pressed

*/

#include "SentinalGlobals.h"

string OKSELECTION 			= " <    [OK]    > ";
string OKSELECTIONEXIT 	= "[EXIT]  [OK]  > ";
string EXIT 						= "     [EXIT]     ";
string UP								= "[UP]            ";
string EMPTY 						= " ";

//==========================================================
// PRIMARY FUNCTION DECLARATIONS
// These functions will return a MODE, which is one of the
// global constants shown above, a number 0 through 6
// These functions are called from the main c program task
//==========================================================
short displayLCDChoice_Initial();
short displayLCDChoice_DriveTest();
short displayLCDChoice_TrackLine();
short displayLCDChoice_Behavioral();
short displayLCDChoice_Discovery();
short displayLCDChoice_Mapping();
short displayLCDChoice_Defensive();
//==========================================================
// HELPER FUNCTION DECLARATIONS
// These functions are called by the PRIMARY FUNCTIONS
//==========================================================
void populateLCDMenu( char *str1, char *str2 );
void showIECValuesOnLCD(tSensors quadEncoder);
void showIECValuesOnLCD();
void showSonarValuesOnLCD();
void showLineFollowerValuesOnLCD();

//==========================================================
// FUNCTIONS FOR LISTENING TO JOYSTICK COMMANDS
// These commands will override or interrupt the LCD input
//==========================================================
short listenJoystick();

//==========================================================
// ALL FUNCTION DEFINITIONS FOR THE REST OF THIS FILE
//==========================================================

//==========================================================
// 	displayLCDChoice_Initial
//
//
//==========================================================
short displayLCDChoice_Initial()
{
	writeDebugStreamLine("displayLCDChoice_Initial");

	populateLCDMenu( "REMOTE CONTROL?", OKSELECTIONEXIT );
	//displayLCDChar(1,0,222); // show the up arrow at the far left
	//displayLCDChar(1,1,32); // show the up arrow at the far left

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	//wait1Msec(PAUSETIME); // slow things down a bit

	if( 			LCDButton == 1 || joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_EXIT;
		return	MODE_EXIT;
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed
	{
		return MODE_REMOTECONTROL;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_DriveTest();
	}
	else
				return	MODE_EXIT; // exit function, too many buttons pressed simultaneously
} // end displayLCDChoice_Initial

//==========================================================
// 	displayLCDChoice_DriveTest
//	params - none
//
//==========================================================
short displayLCDChoice_DriveTest()
{
	writeDebugStreamLine("displayLCDChoice_DriveTest");

	populateLCDMenu("DRIVE TEST?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 			LCDButton == 1 || joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Initial();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed "OK"
	{
		return MODE_DRIVETEST;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_TrackLine();
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_DriveTest

//==========================================================
// 	displayLCDChoice_TrackLine
//	params - none
//
//==========================================================
short displayLCDChoice_TrackLine()
{
	writeDebugStreamLine("displayLCDChoice_TrackLine");

	populateLCDMenu("TRACK LINE?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 			LCDButton == 1 || joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_DriveTest();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed "OK"
	{
		return MODE_TRACKLINE;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Behavioral();
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_TrackLine

//==========================================================
// 	displayLCDChoice_Behavioral
//	params - none
//
//==========================================================
short displayLCDChoice_Behavioral()
{
	writeDebugStreamLine("displayLCDChoice_Behavioral");

	populateLCDMenu("BEHAVIORAL?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 			LCDButton == 1 || joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_TrackLine();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2) // 2:  Center button is pressed "OK"
	{
		return MODE_BEHAVIORAL;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Discovery();
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_Behavioral

//==========================================================
// 	displayLCDChoice_Discovery
//	params - none
//
//==========================================================
short displayLCDChoice_Discovery()
{
	writeDebugStreamLine("displayLCDChoice_Discovery");

	populateLCDMenu("DISCOVERY?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 			LCDButton == 1 || joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Behavioral();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed "OK"
	{
		return MODE_DISCOVERY;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Mapping();
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_Discovery

//==========================================================
// 	displayLCDChoice_Mapping
//	params - none
//
//==========================================================
short displayLCDChoice_Mapping()
{
	writeDebugStreamLine("displayLCDChoice_Mapping");

	populateLCDMenu("MAPPING?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 		LCDButton == 1 	|| joystickBtn == 1 ) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Behavioral();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed "OK"
	{
		return MODE_MAPPING;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Defensive(); // return to initial mode
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_Mapping

//==========================================================
// 	displayLCDChoice_Defensive
//	params - none
//
//==========================================================
short displayLCDChoice_Defensive()
{
	writeDebugStreamLine("displayLCDChoice_Defensive");

	populateLCDMenu("DEFENSIVE?", OKSELECTION);

	short LCDButton = 0;
	short joystickBtn = 0;

	// Infinite loop, waiting for user to press button.
	while(LCDButton == 0 && joystickBtn == 0 )
	{
		LCDButton = nLCDButtons;
		joystickBtn = listenJoystick();
		wait1Msec(PAUSETIME); // slow things down a bit
		if( LCDButton > 0 || joystickBtn > 0 )
			break; // exit the while loop
	} // end while loop

	wait1Msec(PAUSETIME); // slow things down a bit

	if( 		LCDButton == 1 || joystickBtn == 1) // 1:  Left button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Mapping();
	}
	else if( 	LCDButton == 2 || joystickBtn == 2 ) // 2:  Center button is pressed "OK"
	{
		return MODE_DEFENSIVE;
	}
	else if( 	LCDButton == 4 || joystickBtn == 4 ) // 4:  Right button is pressed
	{
		ROBOT_MODE = MODE_DECIDING;
		displayLCDChoice_Initial(); // return to initial mode
	}
	else
		return MODE_EXIT; // exit function, too many buttons pressed simultaneously

} // end displayLCDChoice_Defensive

//==========================================================
// 	populateLCDMenu
//	params - none
//
//   note that ...
//   function("MyString");
//
//   is similar to ...
//
//	char *s = "MyString";
//  function(s)
//
//==========================================================
void populateLCDMenu(char *str1, char *str2)
{
	//writeDebugStreamLine("populateLCDMenu");

	// Clear the LCD
	clearLCDLine(0);
	clearLCDLine(1);

	// Populate first line of LCD with the string parameter
	setLCDPosition(0,0);
	displayLCDCenteredString(0, str1);

	// Populate second line of LCD
	setLCDPosition(1,0);
	displayLCDString(1, 0, str2);

	if( str2 == EXIT )
	{
		displayLCDChar(1,0,197); // show the up arrow at the far left
	}

	wait1Msec(200);

} // end populateLCDMenu

//==========================================================
// 	showIECValuesOnLCD
//	params - none
//  prints the IEC values for each motor onto the first line
// 	of the LCD
//  function(s)
//
//==========================================================
void showIECValuesOnLCD(tSensors quadEncoder)
{
	// create infinite loop that shows sensor values on LCD
	//while(true)
	//{
		clearLCDLine(0);
		//displayLCDNumber(0, 0, nMotorEncoder(motorIEC_LF)	);
		//displayLCDNumber(0, 9, nMotorEncoder(motorIEC_RF)	);
		displayLCDNumber(0, 9, quadEncoder	);
		wait1Msec(200);
	//}

	//clearLCDLine(0);
	//displayLCDNumber(0, 0, nMotorEncoder(motorIEC_LF)	);
	//displayLCDNumber(0, 9, nMotorEncoder(motorIEC_RF)	);
	//displayLCDNumber(0, 9, quadEncoder	);
	//displayLCDNumber(0, 0, nMotorEncoder(motorIEC_LR)	);
	//displayLCDNumber(0, 9, nMotorEncoder(motorIEC_RR)	);
	//wait1Msec(PAUSETIME);
} // end showIECValuesOnLCD

//==========================================================
// 	showSonarValuesOnLCD
//  Shows sonar values on the First line of the LCD
//
//==========================================================
void showSonarValuesOnLCD()
{
	clearLCDLine(0);
	displayLCDString(0,0,"Fr: ");
	displayLCDNumber( 0, 5, sonarFrontValGlobal);
	displayLCDString(0,9,"Rr: ");
	displayLCDNumber( 0, 13, sonarRearValGlobal);
}

//==========================================================
// 	showLineFollowerValuesOnLCD
//  Show line follower values on the First line of the LCD
//
//==========================================================
void showLineFollowerValuesOnLCD()
{
	clearLCDLine(0);
	displayLCDNumber( 0, 	0, 	lineFollower1ValGlobal);
	displayLCDNumber( 0, 	6, 	lineFollower2ValGlobal);
	displayLCDNumber( 0, 	12, lineFollower3ValGlobal);
}

//==========================================================
// 	showIECValuesOnLCD
//	params - none
//  prints the IEC values for each motor onto the first line
// 	of the LCD
//  function(s)
//
//==========================================================
void showIECValuesOnLCD()
{

} // end showIECValuesOnLCD

short listenJoystick()
{
	// default nothing was pressed on joystick
	int returnVal = 0;

	if( vexRT[Btn5U] == 1 )
		returnVal = 1; // corresponds to LCD 1
  else if( vexRT[Btn5D] == 1 )
  	returnVal = 2; // corresponds to LCD 2
  else if( vexRT[Btn6U] == 1 )
  	returnVal = 4;	// corresponds to LCD 4
 	else
 		returnVal = 0;

	return returnVal;
}
