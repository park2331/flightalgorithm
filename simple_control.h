//Josh's targed control agorithm
//These are used by simple_control(), steerToTarget(), landingRoutine(), and landingFlare()

float headingAct[2] = {0,0}; //actual heading, initialize {0,0} here; updated in loop
float servoOutPrcnt[2] = {0,0}; //init {0,0}, corrective steering command to servo without amplification scaling, a fn of headingDev
float desiredYawRate[2] = {0,0};
int steeringActive = 0;
float steeringScale[2] = {0,0};
float achievedYawRate[2] = {0,0};
int steerDelayCounter = 0;
int turnedLeft[2] = {0,0};
	
int turnedRight[2] = {0,0};
double headingTime[2] = {0,0};

//Function prototypes:
void simple_control(void);
void steerToTarget(float headingDes);
void landingRoutine(float headingDes);
void landingFlare (void);

//Landing target parameters:
#define TARGETLAT 37.2862100  // target latitude in decimal degrees
#define TARGETLONG -121.8517000 //target longitude in decimal degrees
#define TARGEt_ALTITUDE 72.1550 //elevation of landing target coordinates above sea level, meters

//Distance and heading calculation constants:
#define EARTHRAD 6371000 //radius of the Earth, average, m

//Steering algorithm tuning parameters:

//deadband +/- degrees deviation between actual and desired; within this range no servo updates will occur this loop
#define HEADING_DEADBAND 2.5

// Yaw rate deadband +/- degrees/sec for no yaw rate adjustment in next loop (holds servo values constant)
#define DESIRED_YAW_DEADBAND

// Number of loops to hold steering value before stepping to a new servo pull value; value of 1 updates every loop, 2 every 2 loops, etc.
#define NUM_LOOPS_BEFORE_SCALING_TURN 1

// If yaw rate deviation from actual to desired +/- is less than this value, steering gain stepping uses fine increments
#define FINE_SCALING_CUTOFF_DEG_SEC 2.5

// Range 0 to 1; servo pull percent change each update (10 times/sec) to achieve target yaw rate when yaw rate deviation < FINE SCALING CUTOFF DEG SEC
#define FINE_SCALING_STEP_PERCENT 0.010

#define FINE_SCALING_UNWIND_GAIN 1

#define COARSE_SCALING_STEP_PERCENT 0.020

#define COARSE_SCALING_UNWIND_GAIN 1.5

// Desired yaw rate coefficients; controls desired yaw rate for a given heading deviation
#define DESIREDYAW_COEEFF_1 -0.00000000000937 //x^6
#define DESIREDYAW_COEEFF_2  0.00000000662971 //x^5
#define DESIREDYAW_COEEFF_3 -0.00000185521159 //x^4
#define DESIREDYAW_COEEFF_4  0.00026074766721 //x^3
#define DESIREDYAW_COEEFF_5 -0.01906582376881 //x^2
#define DESIREDYAW_COEEFF_6  0.76467370416140 //x
#define DESIREDYAW_COEEFF_7 -0.41502129438777 //constant term


// Landing routine constants:
#define NUM_LOOPS_BEFORE_SCALING_TURN_LANDING 1

#define FINE_SCALING_CUTOFF_DEG_SEC_LANDING 5

// range 0 to 1;
#define FINE_SCALING_STEP_PERCENT_LANDING 0.015

#define COARSE_SCALING_STEP_PERCENT_LANDING 0.035

#define LANDING_RADIUS_THRESHOLD 7.5

#define LANDING_RADIUS 6

#define FLARE_HEIGHT 4
#define FLARE_PRCNT 1.0

//***Debug Code Start -- comment all out for Moneky!!***

//Used to feed fake data to control routine for compilation on windows PC

/*
#define DEGREES_TO_RAD 0.01745329251994329576923690768486
#define RAD_TO_DEG 57.295779413082320876798154814105
#define SERVO_RIGHT_WINCH_3 2
#define SERVO_LEFT_WINCH_4 3
#define SERVO_R_WINCH_MAX_TRAVEL 5.75
#define SERVO_L_WINCH_MAX_TRAVEL -5.75
#define SERVO_R_WINCH_SCALE_FACOTR 1.75
#define SERVO_L_WINCH_SCALE_FACOTR -1.75

float gGPSheading = 284;
float gGPSlatitude = 37.2862000;
float gGPSlongitude = -121.8517000;
float gGPSgndspeed = 15;
float gGPSaltitude = 82.1550;
double gGPSTOW = 579857000;


*/
// End debugging code




