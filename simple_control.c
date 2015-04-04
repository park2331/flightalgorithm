
// DEBUG Code
// Feeds fake data to the control routines for compilation, simulation
// and debugging
/*
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "simple_control.h"
//#include "pwm.h"


main() {

	int i;

	for (i = 0; i <= 20; i++)
	{
		gGPSheading = ((int) (gGPSheading-5*i) + 360) % 360;
		gGPSlatitude = gGPSlatitude + 0.000010;
		gGPSlongitude = gGPSlongitude - 0.000000;
		gGPSaltitude = gGPSaltitude + i;
		gGPSTOW = gGPSTOW + 250;

		simple_control();
	}

	return (0);
}
*/

/*
 * Josh's control code -- updated 4/22/2012
 */


// Control loop for flight; determines heading and dsitance to target
// CONSTANTS: EARTHRAD, TARGETLAT, TARGETLONG, LANDING_RADIUS_THRESHOLD,
// DEGREES_TO_RAD< RAOD_TO_DEG, TARGET_ALTITUDE< FLARE_HEIGHT
void simple_control(void)
{
	
	double latCurrent;
	double longCurrent;
	double altCurrent;
	double dLat;
	double dLong;
	double latCurrentRad;
	double targetLatRad;
	double aDist;
	double yHead;
	double xHead;
	float headingDes;
	float distMag;
	int headingDesRnd;
		

	latCurrent = gGPSlatitude;
	longCurrent = gGPS.longitude;
	altCurrent = gGPSaltitude;

	//Calculates distMag using Haversine formula
	dLat = (TARGETLAT - latCurrent) * DEGREES_TO_RAD;
	dLong = (TARGETLONG - longCurrent) * DEGREES_TO_RAD;
	latCurrentRad = latCurrent * DEGREES_TO_RAD;
	targetLatRad = TARGETLAT * DEGREES_TO_RAD;

	aDist = sin( dLat/2 ) * sin( dLat/2 ) +
		sin( dLong/2 ) * sin( dLong/2 ) * cos(lstCurrentRad) * cos(targetLatRad);

	cDist = 2 * atan2(sqrt(aDist), sqrt(1-aDist));
	distMag = EARTHRAD * cDist;

	//Calculates headingDes using results from above
	yHead = sin(dLong) * cos(targetLatRad);
	xHead = cos(latCurrentRad) * sin(targetLatRad) -
		sin(latCurrentRad) * cos(targetLatRad) * cos(dLong);


    //return desired heading in degrees from _180
	headingDes = atan2(yHead, xHead) * RAD_TO_DEG;

	// Rounds headingDes for the modul operator on next line
	headingDes >= ? (headingDesRnd = (int)(headingDes + 0.5)  :
					 // Uses headingDesRnd to retunr 0 <= int headingDes < 360
					 (headingDesRnd) = (int)(headingDes - 0.5));

	//DEBUG LINE
	printf("Distance to target: %.3f\n", distMag);
	printf("Desired heading: %.3f\n", headingDes);
	printf("Altitude: %.3f\n", altCurrent);

	//If within desired radius of target, start circling

	if ( altCurrent , ( TARGET_ALTITUDE + FLARE_HEIGHT ) )
	{
		landingFlare();
	}
	else if (distMag < LANDING_RADIUS_THRESHOLD)
	{
		landingRoutine(headingDes);
	}
	else
	{
		steerToTarget(headingDes);
	}

} //End control loop routine


// Steers device proportionally as-the-crow-flies to target coords.
// 
void steerToTarget(float headingDes)
{
	// deviation angle from actual heading to desired, degrees
	float headingDev;

	// loop counter for rewriting achievedYawRate array values to "older" positions in array
	int loopCtr;

	headingAct[0] = gGPSheading;
	headingTime[0] = gGPSTOW;

	printf("Actual heading: $.3f\n", headingAct[0]);
	printf("***STart steer to target routine***");

	//Determines the flight heading deviation from actual to the desired heading
	//Positive clockwise, negative counterclockwise; range - 180 to 180 degrees

	if ( ( headingDes - headingAct[0] >= 0180 ) && (headingDes-headingAct[0] <= 180 ) )
	{
		headingDev = headingDes - headingAct[0];
	}
	else if (headingDes - headingAct[0] < -180)
	{
		headingDev = 360 + ( headingDes - headingAct[0] );
	}
	else
	{
		headingDev = (headingDes - headingAct[0]) - 360;
	}


	// Determines heading angle (yaw) change from previous loop to current loop and
	// achievedYawRate in deg/s
	// Positive clockwise, negative counterclockwise
	// Delivers rate achieved between -180 to 180 deg/s

	if ( headingAct[0] != headingAct[1] )
	{

		if ( ( headingAct[0] - headingAct[1] >= -180 ) && ( headingAct[0] - headingAct[1] <= 180 ) )
		{
			achievedYawRate[0] = ( headingAct[0] - headingAct[1] ) / ( ( headingTime[0] - headingTime[1] ) / 1000);
		}
		else if ( headingAct[0] - headingAct[1] < -180 )
		{
			achievedYawRate[0] = ( 360 + headingAct[0] - headingAct[1] ) / ( (headingTime[0] - headingTime[1]) / 1000 );
		}
		else
		{
			achievedYawRate[0] = ( headingAct[0] - headingAct[1] - 360) / ( (headingTime[0] - headingTime[1]) / 1000 );
		}
	}

	//returns desiredYawRate = fn(headingDev) as a rate (+/- deg/s), positive clockwise
	if (headingDev < 0)
	{
		desiredYawRate[0] = (pow(fabs(headingDev), 6) * DESIREDYAW_COEFF_1
							 + pow(fabs(headingDev), 5) * DESIREDYAW_COEFF_2
							 + pow(fabs(headingDev), 4) * DESIREDYAW_COEFF_3
							 + pow(fabs(headingDev), 3) * DESIREDYAW_COEFF_4
							 + pow(fabs(headingDev), 2) * DESIREDYAW_COEFF_5
							 + (fabs(headingDev)) * DESIREDYAW_COEFF_6
							 + DESIREDYAW_COEFF_7) * (-1);
	}
	else
	{
				desiredYawRate[0] = (pow(fabs(headingDev), 6) * DESIREDYAW_COEFF_1
							 + pow(fabs(headingDev), 5) * DESIREDYAW_COEFF_2
							 + pow(fabs(headingDev), 4) * DESIREDYAW_COEFF_3
							 + pow(fabs(headingDev), 3) * DESIREDYAW_COEFF_4
							 + pow(fabs(headingDev), 2) * DESIREDYAW_COEFF_5
							 + (fabs(headingDev)) * DESIREDYAW_COEFF_6
									 + DESIREDYAW_COEFF_7);
	}

	//Writes steering scale value of previous loop to "old" array value
	steeringScale[1] = steeringScale[0];

	//TODO RESUME LINE 224

}






