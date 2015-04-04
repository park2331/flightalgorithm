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
