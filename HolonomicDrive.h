/*
		HolonomicDrive.h
		This file declares and defines functions for a 4 wheel
		robotic vehicle with the Mecanum wheels.
		The 4 Mecanum wheels are placed at the four corners of a
		square shaped chasis.

		Motor definitions are declared in the main .c file.
		The four motors are defined as:
		motor_LF		motor_RF
		motor_LR    	motor_RR

		The roller parts of each Mecanum wheel are oriented such that when
		looking directly down onto the robot, the top roller will be angled
		pointing inward to the center of the square robot.

		For the moveXXX, rotateXXX, and traverseXXX functions, there
		is a code used to help the logic in the motorAdjustPower function.
		=========MOVING DIRECTION=======
		DIAGONAL FRONT RIGHT		0
		FRONT 									1
		DIAGONAL FRONT LEFT			2
		TRAVERSE LEFT						3
		DIAGONAL REAR LEFT			4
		BACK 										5
		DIAGONAL REAR RIGHT			6
		TRAVERSE RIGHT					7
		=================================

*/


#include "LCDManager.h"
// including LCDManager.h will also
// include SentinalGlobals.h
//#include "SentinalGlobals.h"

// CLEANUP, RESET FUNCTIONS
void resetMotorEncoders();

// MOTOR IEC FEEDBACK & MOTOR CONTROL
void moveDistance(int speed, 			// desired speed
									float distance, // desired distance
									short dir_RF,		// direction RF wheel
									short dir_RR,		// direction LF wheel
									short dir_LF,		// direction RR wheel
									short dir_LR );	// direction LR wheel


void motorAdjustPower(		int speed, 			// desired speed
													float *actualDistance, // array
													bool b_RF,			// activate RF wheel
													bool b_LF,			// activate LF wheel
													bool b_RR,			// activate RR wheel
													bool b_LR, 			// activate LR wheel
													short activeMotorCount);

// LINEAR MOTION
void moveForward(							short speed,
															int ms,
															float distance); // inches
void moveBackward(						short speed,
															int ms,
															float distance); // inches
void moveForwardReact(				short speed	);
void moveBackwardReact(				short speed );
void moveTraverseRight(				short speed,
															int ms,
															float distance ); // inches
void moveTraverseLeft(				short speed,
															int ms,
															float distance) ; // inches
void moveTraverseRightReact(	short speed	);
void moveTraverseLeftReact(		short speed );
void moveDiagonalFrontLeft(		short speed, int ms);
void moveDiagonalFrontRight(	short speed, int ms);
void moveDiagonalRearRight(		short speed, int ms);
void moveDiagonalRearLeft(		short speed, int ms);

// ROTATIONARY MOTION
void moveRotateClockWise(					short speed, int ms);
void moveRotateCounterClockWise(	short speed, int ms);

// TEST FUNCTIONS
void runDrivingTest(					short speed,
															int ms,
															int pauseMilliseconds,
															float distance);
short runDrivingTestBasic(		short speed,
															int ms,
															int pauseMilliseconds,
															bool b_forward,
															bool b_backward,
															bool b_traverseRight,
															bool b_traverseLeft,
															float distance);
void runDrivingTestDiagonal(	short speed,
															int ms,
															int pauseMilliseconds);
void runDrivingTestTurns(			short speed,
															int ms,
															int pauseMilliseconds);

bool collisionDetected();


//====================================================================
//	resetMotorEncoders
//
//
//====================================================================
void resetMotorEncoders()
{
	resetMotorEncoder(motor_RF);
	resetMotorEncoder(motor_LF);
	resetMotorEncoder(motor_RR);
	resetMotorEncoder(motor_LR);
}

//====================================================================
//	moveDistance
//  Move the designaged wheels the desired distance
// 	1 Revolution of IEC = 627.2 ticks
//	60 Tooth Gears : 36 Tooth Gears = 1.67 ratio between gears
//  4" Diameter Mecanum Wheel
//			Wheel Circumference = 2*PI*r = 2*PI*2 = 12.6"
//  If 1 Revolution of IEC moves the last gear by a ratio of 1.67 Revolutions
//  then 1 Revolution of IEC equals 1.67*12.6" = 21" linear inches
//  (627.2 Ticks)/21" = 30 Ticks per inch
// 	1 Tick = .03 inches
//	1 inch = 30 Ticks
//
// 	This function must be mindful of the global variable
//  DIRECTION
//  There are 8 possible predefined values for DIRECTION
//  defined in SentinalGlobals.h
//====================================================================
void moveDistance(	int speed, 			// desired speed
										float distance,  // distance in inches
										short dir_RF,		// direction RF wheel
										short dir_RR,		// direction LF wheel
										short dir_LF,		// direction RR wheel
										short dir_LR )	// direction LR wheel
{
	writeDebugStreamLine("moveDistance ");

	// when DIRECTION is DIRECTION_FRONT or DIRECTION_REAR
	// then use default behavior for linear forward or backward
	// movement

	// establish local variables for the feedback and control system
	float diffArray[] 			= { 0.0, 0.0, 0.0, 0.0 };
	float actualDistance[]	= { 0.0, 0.0, 0.0, 0.0 };
	bool b_distanceAchieved = false;
	short activeMotorCount = 0;

	resetMotorEncoders();

	// EMERGENCY!!! COLLISION DETECTED
	if( collisionDetected() )
	{
		stopAllMotors();
		resetMotorEncoders();

		// Write something to the LCD
		// to do ...

		// exit from function
		return;
	}

	// Start sanity check for the IECs, which
	// sometimes do not work
	// clear the timer
	// later, we will check to see if IEC's have
	// a value of zero
	clearTimer(T4);

	// turn on only the needed motors, using the boolean flags
	if( dir_RF != 0 )
	{
		activeMotorCount += 1;
		motor[motor_RF]  = speed*dir_RF;
	}

	if( dir_LF != 0 )
	{
		activeMotorCount += 1;
		motor[motor_LF]  = speed*dir_LF;
	}

	if( dir_RR != 0 )
	{
		activeMotorCount += 1;
		motor[motor_RR]  = speed*dir_RR;
	}

	if( dir_LR != 0 )
	{
		activeMotorCount += 1;
		motor[motor_LR]  = speed*dir_LR;
	}

	// EMERGENCY!!! COLLISION DETECTED
	if( collisionDetected() )
	{
		stopAllMotors();
		resetMotorEncoders();

		// Write something to the LCD
		// to do ...

		// exit from function
		return;
	}

	if( activeMotorCount == 0 )
	{
		writeDebugStreamLine("Error: activeMotorCount %d", activeMotorCount);
		return;
	}



	// Let the motors turn for 100 ms before
	// continuing
	wait1Msec(100);

	// check to see if the IECs have registered any movement

	// Special conditions must be incorporated for
	// lateral movement
	// if the DIRECTION global variable equals the following
	// values, then do the following ...
	//
	// if DIRECTION == DIRECTION_RIGHT
	//
	// if DIRECTION == DIRECTION_LEFT
	//
	while( true )
	{
		// There are 30 Ticks counted per linear inch traveled.
		// Sample the distance traveled by each wheel
		// by counting the ticks and dividing by 30
		if( dir_RF != 0  )
		{
			actualDistance[0] = getMotorEncoder(motor_RF)
											/MOVEMENT_LINEAR_ADJUSTER;
		}

		if( dir_LF != 0  )
		{
			actualDistance[1] = getMotorEncoder(motor_LF)
										/MOVEMENT_LINEAR_ADJUSTER;
		}

		if( dir_RR != 0 )
		{
			actualDistance[2] = getMotorEncoder(motor_RR)
									/MOVEMENT_LINEAR_ADJUSTER;
		}

		if( dir_LR != 0)
		{
			actualDistance[3] = getMotorEncoder(motor_LR)
										/MOVEMENT_LINEAR_ADJUSTER;
		}

		// EMERGENCY!!! COLLISION DETECTED
		if( collisionDetected() )
		{
			stopAllMotors();
			resetMotorEncoders();

			// Write something to the LCD
			// to do ...

			// exit from function
			return;
		}


		// Check to see if any wheels have travelled far enough
		for( int i = 0; i < 4; i++ )
		{
		  // establish a local variable for actual distance
		  // traveled for each wheel
			// use abs() in to keep the math positive
		  // under forward or backward travel, we use
		  // default linear measurement.
			float actualDistanceVar = abs(actualDistance[i]);

			// after the Timer T4 has counted 1 second, we check
			// for any movement in the IECs
			// If no movement in the IECs, then we terminate
			// this function to eliminate the possibility of
			// an infinite moving action
			if( time1[T4] > IEC_ERROR_TIMEOUT &&
					actualDistanceVar == 0.0    )
			{
				// we have a problem with an encoder
				// post a messege to the LCD???
				// abort this function to prevent a runaway
				// condition
				writeDebugStreamLine("moveDistance IEC_ERROR_TIMEOUT ");
				resetMotorEncoders();
				stopAllMotors();
				return;
			}

			// for traversling left or right, the actual distance
			// of the robot traveled will be different from the
			// actual distance the wheel moved
			// for Lateral movement, only 30% of the wheel
			// movement is converted to lateral robot movement
			// Therefore, multiply the local variable by .33

			if( DIRECTION == DIRECTION_RIGHT ||
					DIRECTION == DIRECTION_LEFT )
			{
		  	actualDistanceVar = actualDistanceVar*MOVEMENT_LATERAL_ADJUSTER;
			}


			// If any wheel traveled the distance, then break out of loop
			if( actualDistanceVar >= distance )
			{
				b_distanceAchieved = true;
				resetMotorEncoders();
				stopAllMotors();
				break; // break out of for loop
			} // end if
		} // end for loop

		// if b_distanceAchieved flag is true, then we are done
		// exit the function
		if( b_distanceAchieved )
		{
			resetMotorEncoders();
			stopAllMotors();
			break; // break out of while loop
		}

		// Use this function if we want to adjust the power
		// to each motor using negative closed loop feedback
		/**
		motorAdjustPower(	speed,
											actualDistance,
											b_RF, b_LF, b_RR, b_LR,
											activeMotorCount);
		*/

		wait1Msec(50);
	} // end while

	resetMotorEncoders();
	stopAllMotors();
} // end moveDistance


//====================================================================
//	motorAdjustPower
// 	adjust each motor speed power by using information from
// 	the IEC (Integrated Encoder Module) for each motor and
//	using a formula
//
// 	CAUTION!!! THIS FUNCTION IS NOT WORKING RELIABLY
//	ERRADIC BEHAVIOR AT LOW SPEEDS
//  DISCONTINUE USAGE UNTIL IT CAN BE FIXED OR MODIFIED TO WORK
//  WITHOUT PROBLEMS
//
//====================================================================
void motorAdjustPower(		int speed, 			// desired speed
													float *actualDistance,  // distance in inches
													bool b_RF,			// activate RF wheel
													bool b_LF,			// activate LF wheel
													bool b_RR,			// activate RR wheel
													bool b_LR,			// activate LR wheel
													short activeMotorCount)
{
	writeDebugStreamLine("motorAdjustPower ");
	// Closed loop
	// feedback adjustment of the power to each wheel
	// find the total average ticks and then adjust each
	// wheel so it has a little more or less power

	// First sum up the distance traveled by each wheel and
	// store into a variable
	float sumDistance = 0.0;

	if( b_RF )
	{
		sumDistance += actualDistance[0];
	}

	if( b_LF )
	{
		sumDistance += actualDistance[1];
	}

	if( b_RR )
	{
		sumDistance += actualDistance[2];
	}

	if( b_LR)
	{
		sumDistance += actualDistance[3];
	}

	float avgDistance =	sumDistance / activeMotorCount;

	writeDebugStreamLine("avgDistance %d", avgDistance);

	float newSpeed = 0.0;
	int dither = 2;

	// turn on only the needed motors, using the boolean flags
	if( b_RF && abs(avgDistance) > 0.0 )
	{
		// modify feedback equation with a dither variable
		if( 			abs(actualDistance[0]) < abs(avgDistance) ) {
			dither = -2;
		} else if(abs(actualDistance[0]) > abs(avgDistance) ) {
			dither = 2;
		} else {
			dither = 0;
		}

		newSpeed  = speed * (speed/(speed * (actualDistance[0]/avgDistance ) + dither ));
		writeDebugStreamLine("set speed RF: %d", newSpeed);
		motor[motor_RF] = (int)newSpeed;
	}

	if( b_LF && abs(avgDistance) > 0.0  )
	{
		// modify feedback equation with a dither variable
		if( 			abs(actualDistance[1]) < abs(avgDistance) ) {
			dither = -2;
		} else if(abs(actualDistance[1]) > abs(avgDistance) ) {
			dither = 2;
		} else {
			dither = 0;
		}

		newSpeed  = speed * (speed/(speed * (actualDistance[1]/avgDistance ) + dither ));
		writeDebugStreamLine("set speed LF: %d", newSpeed);
		motor[motor_LF] = (int)newSpeed;
	}

	if( b_RR && abs(avgDistance) > 0.0 )
	{
		// modify feedback equation with a dither variable
		if( 			abs(actualDistance[2]) < abs(avgDistance) ) {
			dither = -2;
		} else if(abs(actualDistance[2]) > abs(avgDistance) ) {
			dither = 2;
		} else {
			dither = 0;
		}
		newSpeed  = speed * (speed/(speed * (actualDistance[2]/avgDistance ) + dither ));
		writeDebugStreamLine("set speed RR: %d", newSpeed);
		motor[motor_RR] = (int)newSpeed;
	}

	if( b_LR && abs(avgDistance) > 0.0 )
	{
		// modify feedback equation with a dither variable
		if( 				abs(actualDistance[3]) < avgDistance ) {
			dither = -2;
		} else if(	abs(actualDistance[3]) > avgDistance ) {
			dither = 2;
		} else {
			dither = 0;
		}

		newSpeed  = speed * (speed/(speed * (actualDistance[3]/avgDistance ) + dither ));
		writeDebugStreamLine("set speed LR: %d", newSpeed);
		motor[motor_LR] = (int)newSpeed;
	}

} // motorAdjustPower

//====================================================================
//	moveForward
//
//
//====================================================================
void moveForward(	short speed,
									int 	ms,
									float distance) // inches
{
	writeDebugStreamLine("moveForward speed=%d", speed );

	resetMotorEncoders();

	DIRECTION = DIRECTION_FRONT;

	// We want to move for a period of time
	if( ms != 0 )
	{
		// start the motors, using the designated amount of speed
		motor[motor_LF]  = speed;
		motor[motor_LR]  = speed;
		motor[motor_RF]  = speed;
		motor[motor_RR]  = speed;

		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		stopAllMotors();
		return;
	}

	// We want to move a specific distance
	if( distance != 0 )
	{
		moveDistance(	speed,
									distance,  // desired distance inches
									1,1,1,1);
	}

	stopAllMotors();

} // end moveForward

//====================================================================
//	moveBackward
//
//
//====================================================================
void moveBackward(	short speed,
										int 	ms,
										float distance) // inches
{
	writeDebugStreamLine("moveBackward speed=%d", speed );

	resetMotorEncoders();

	DIRECTION = DIRECTION_REAR;

	// We want to move for a period of time
	if( ms != 0 )
	{
		// start the motors, using the designated amount of speed
		motor[motor_LF]  = -speed;
		motor[motor_LR]  = -speed;
		motor[motor_RF]  = -speed;
		motor[motor_RR]  = -speed;

		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		stopAllMotors();
		return;
	}

	// We want to move a specific distance
	if( distance != 0 )
	{
		moveDistance(	speed,
									distance,  // desired distance inches
									-1,-1,-1,-1);
	}

	stopAllMotors();

} // end moveBackward

//====================================================================
//	moveForwardReact
//
//
//====================================================================
void moveForwardReact(	short speed )
{
	writeDebugStreamLine("moveForwardReact speed=%d", speed );
	resetMotorEncoders();
	// start the motors, using the designated amount of speed
	motor[motor_LF]  = speed;
	motor[motor_LR]  = speed;
	motor[motor_RF]  = speed;
	motor[motor_RR]  = speed;

} // end moveForwardReact

//====================================================================
//	moveBackwardReact
//
//
//====================================================================
void moveBackwardReact(	short speed )
{
	writeDebugStreamLine("moveBackwardReact speed=%d", speed );
	moveForwardReact( -speed );
} // end moveBackwardReact

//====================================================================
//	moveDiagonalFrontRight
//
//
//====================================================================
void moveDiagonalFrontRight( short speed, int ms)
{
	writeDebugStreamLine("moveDiagonalFrontRight speed=%d", speed );

	DIRECTION = DIRECTION_RIGHT_FRONT;

	resetMotorEncoders();

	// start the motors, using the designated amount of speed
	motor[motor_LF]  = speed;
	motor[motor_RR]  = speed;

	// if a nonzero time was passed in, then
	// wait appropriate time and then stop motors
	if( ms != 0 )
	{
		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		resetMotorEncoders();
		stopAllMotors();
	}

} // end moveDiagonalFrontRight

//====================================================================
//	moveDiagonalFrontLeft
//
//
//====================================================================
void moveDiagonalFrontLeft( short speed, int ms)
{
	writeDebugStreamLine("moveDiagonalFrontLeft speed=%d", speed );

	DIRECTION = DIRECTION_LEFT_FRONT;

	resetMotorEncoders();

	// start the motors, using the designated amount of speed
	motor[motor_RF]  = speed;
	motor[motor_LR]  = speed;

	// if a nonzero time was passed in, then
	// wait appropriate time and then stop motors
	if( ms != 0 )
	{
		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		resetMotorEncoders();
		stopAllMotors();
	}

} // end moveDiagonalFrontLeft

//====================================================================
//	moveDiagonalRearRight
//
//
//====================================================================
void moveDiagonalRearRight( short speed, int ms)
{
	writeDebugStreamLine("moveDiagonalRearRight speed=%d", speed );
	moveDiagonalFrontLeft(-speed, ms );
} // end moveDiagonalRearRight

//====================================================================
//	moveDiagonalRearLeft
//
//
//====================================================================
void moveDiagonalRearLeft( short speed, int ms)
{
	writeDebugStreamLine("moveDiagonalRearLeft speed=%d", speed );
	moveDiagonalFrontRight(-speed, ms );
} // end moveDiagonalRearLeft

//====================================================================
//	moveTraverseRight
//
//
//====================================================================
void moveTraverseRight(	short speed,
												int ms,
												float distance ) // inches
{
	writeDebugStreamLine("moveTraverseRight speed=%d", speed );

	resetMotorEncoders();

	DIRECTION = DIRECTION_RIGHT;

	if( ms != 0 )
	{
		// start the motors, using the designated amount of speed
		motor[motor_RF]  = -speed;
		motor[motor_RR]  = speed;

		motor[motor_LF]  = speed;
		motor[motor_LR]  =  -speed;


		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		stopAllMotors();
		resetMotorEncoders();
		return;
	} // end if condition

	// We want to move a specific distance
	// need to pass in a variable indicating that this is
	// traversing, and not going forward or backwards
	if( distance != 0 )
	{
		moveDistance(	speed,
									distance,  // desired distance inches
									-1,1,1,-1);
	}

} // end moveTraverseRight


//===============================================================
//	moveTraverseLeft
//
//
//===============================================================
void moveTraverseLeft(	short speed,
												int ms,
												float distance ) // inches
{
	writeDebugStreamLine("moveTraverseLeft speed=%d", speed );
	resetMotorEncoders();

	DIRECTION = DIRECTION_LEFT;

	if( ms != 0 )
	{
		// start the motors, using the designated amount of speed
		motor[motor_RF]  = speed;
		motor[motor_RR]  = -speed;

		motor[motor_LF]  = -speed;
		motor[motor_LR]  =  speed;

		// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		stopAllMotors();
		resetMotorEncoders();
		return;
	} // end if condition

	// We want to move a specific distance
	// need to pass in a variable indicating that this is
	// traversing, and not going forward or backwards
	if( distance != 0 )
	{
		moveDistance(	speed,
									distance,  // desired distance inches
									1,-1,-1,1);
	}
} // end moveTraverseLeft

//====================================================================
//	moveTraverseRightReact
//
//
//====================================================================
void moveTraverseRightReact(	short speed )
{
	writeDebugStreamLine("moveTraverseRightReact speed=%d", speed );
	resetMotorEncoders();
	// start the motors, using the designated amount of speed
		motor[motor_RF]  = -speed;
		motor[motor_RR]  = speed;

		motor[motor_LF]  = speed;
		motor[motor_LR]  =  -speed;

} // end moveTraverseRightReact

//====================================================================
//	moveTraverseLeftReact
//
//
//====================================================================
void moveTraverseLeftReact(	short speed )
{
	writeDebugStreamLine("moveTraverseLeftReact speed=%d", speed );
	resetMotorEncoders();
	// start the motors, using the designated amount of speed
		motor[motor_RF]  = speed;
		motor[motor_RR]  = -speed;

		motor[motor_LF]  = -speed;
		motor[motor_LR]  =  speed;

} // end moveTraverseLeftReact


//================================================================
//	moveRotateClockWise
//
//
//================================================================
void moveRotateClockWise( short speed, int ms)
{
	writeDebugStreamLine("moveRotateClockWise speed=%d", speed );

	// start the motors, using the designated amount of speed
	motor[motor_RF]  = -speed;
	motor[motor_RR]  = -speed;

	motor[motor_LF]  = speed;
	motor[motor_LR]  = speed;


	// if a nonzero time was passed in, then
	// wait appropriate time and then stop motors
	if( ms != 0 )
	{
	// use a timer, instead of using wait1Msec(...)
		// this will allow us to detect a collision.
		clearTimer(T1);
		while( time1[T1] < ms )
		{
			// keep looping until ms has elapsed
			// ..

			// check for collision
			// EMERGENCY!!! COLLISION DETECTED
			if( collisionDetected() )
			{
				stopAllMotors();
				resetMotorEncoders();

				// Write something to the LCD
				// to do ...

				// exit from function
				return;
			}
		} // end while

		stopAllMotors();
		resetMotorEncoders();
	}

} // end moveRotateClockWise

//====================================================================
//	moveRotateCounterClockWise
//
//
//====================================================================
void moveRotateCounterClockWise( short speed, int ms)
{
	writeDebugStreamLine("moveRotateCounterClockWise speed=%d", speed );

	// start the motors, using the designated amount of speed
	moveRotateClockWise(-speed, ms);
} // end moveRotateCounterClockWise

//================================================================
//	runDrivingTest
//
//
//================================================================
void runDrivingTest(short speed,
										int ms,
										int pauseMilliseconds,
										float distance)
{
	writeDebugStreamLine("runDrivingTest speed=%d", speed );

	runDrivingTestBasic(speed, ms, pauseMilliseconds,
											true, true, true, true,
											distance);

	wait1Msec(pauseMilliseconds);

	runDrivingTestTurns(speed, ms, pauseMilliseconds);

	wait1Msec(pauseMilliseconds);

	// TEST DIAGONAL MOVEMENT
	runDrivingTestDiagonal(speed, ms, pauseMilliseconds);

} // end runDrivingTest

//====================================================================
//	runDrivingTestBasic
//
//
//====================================================================
short runDrivingTestBasic(	short speed,
														int ms,
														int pauseMilliseconds,
														bool b_forward,
														bool b_backward,
														bool b_traverseRight,
														bool b_traverseLeft,
														float distance)
{
	writeDebugStreamLine("runDrivingTestBasic speed=%d", speed );

		// write to the sensor
	clearLCDLine(0);
	clearLCDLine(1);

	displayLCDCenteredString(0,"Driving Test:");
	displayLCDCenteredString(1,"Basic");

	wait1Msec(1000);

	if( b_forward )
	{
		clearLCDLine(1);
		displayLCDCenteredString(1, "Moving Forward");
		moveForward(speed, ms, distance);
		wait1Msec(pauseMilliseconds);
	}

	if( b_backward )
	{
		clearLCDLine(1);
 		displayLCDCenteredString(1, "Moving Backward");
		moveBackward(speed, ms, distance);
		wait1Msec(pauseMilliseconds);
	}

	if( b_traverseRight )
	{
		clearLCDLine(1);
		displayLCDCenteredString(1, "Traversing Right");
		moveTraverseRight(speed, ms, distance);
		wait1Msec(pauseMilliseconds);
	}

	if( b_traverseLeft )
	{
		clearLCDLine(1);
		displayLCDCenteredString(1, "Traversing Left");
		moveTraverseLeft(speed, ms, distance);
	}

	// done with the list of commands, return back to
	// state MODE_DECIDING
	return MODE_DECIDING;
} // end runDrivingTestBasic

//====================================================================
//	runDrivingTestDiagonal
//
//
//====================================================================
void runDrivingTestDiagonal(short speed,
														int ms,
														int pauseMilliseconds)
{
	writeDebugStreamLine("runDrivingTestDiagonal speed=%d", speed );

	// TEST DIAGONAL MOVEMENT
	moveDiagonalFrontRight(speed, ms);

	wait1Msec(pauseMilliseconds);

	moveDiagonalRearLeft(speed, ms);

	wait1Msec(pauseMilliseconds);

	moveDiagonalFrontLeft(speed, ms);

	wait1Msec(pauseMilliseconds);

	moveDiagonalRearRight(speed, ms);
} // end runDrivingTestDiagonal

//====================================================================
//	runDrivingTestTurns
//
//
//====================================================================
void runDrivingTestTurns(	short speed,
													int ms,
													int pauseMilliseconds)
{
	writeDebugStreamLine("runDrivingTestTurns speed=%d", speed );

	moveRotateClockWise(speed, ms);
	wait1Msec(pauseMilliseconds);
	moveRotateCounterClockWise(speed, ms);
} // end runDrivingTestTurns

//====================================================================
//	collisionDetected
//	call this function anytime we need to know
//  if robot has collided
//
//====================================================================
bool collisionDetected()
{
	bool collision = false;

	// check all bump sensors for a pressed state
	if( bFrontBumperPressed == 1 )
		collision = true;

  return collision;
}
