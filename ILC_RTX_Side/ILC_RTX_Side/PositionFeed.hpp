#ifndef POSITIONFEED_HPP
#define POSITIONFEED_HPP

#include "windows.h"
#include <vector>
#include <fstream>
#include <string>
#include <iostream>

#include <tra40.h>
#include <dmd40.h>
#include <dsa40.h>
#include <etne40.h>

//Units
#define ITP_REF_MODE 1 //reference mode
#define ROTARY 0
#define USE_SLAVE 0

#define MAX_ROW_COUNT 340000 //use this to avoid looking in file how much line and how much memory to allocate at first
#define MAX_COL_COUNT 4

#define MAX_TIMESTAMPS MAX_ROW_COUNT

#define HOST_CONNEXION_PORT		1140						/* The port allowing a host connexion through UltimET*/
#define HOST_CONNEXION_NAME		"CUSTOMER DEBUG"			/* The name of the connexion which will be displayed in the ComET host list*/

using namespace std;

class CustomerTrajectory
{
public:
	//input values
	double* mPosition;
	double* mCurrent;

	//output values
	double* mML1;
	double* mML0;
	double* mTimeStamps; //for the moment we assume the callback is perfectly accurate
	int mLineNumber;

	CustomerTrajectory() :mPosition(NULL), mML0(NULL), mML1(NULL)
	{

	}

	~CustomerTrajectory()
	{
		clear();
	}

	void clear()
	{
		if (mPosition != NULL)
		{
			delete mPosition;
			delete mCurrent;
			delete mML0;
			delete mML1; //in c++11 can now delete a null pointer
			delete mTimeStamps;
		}
	}

	void create(double* position, double* current, int n)
	{
		mML0 = new double[n];
		mML1 = new double[n];
		mTimeStamps = new double[n];
		mPosition = new double[n];
		mCurrent = new double[n];
		memcpy(mPosition, position, sizeof(double)*n);
		memcpy(mCurrent, current, sizeof(double)*n);
		mLineNumber = n;
	}
};

class PositionFeed
{
public:
	PositionFeed();
	~PositionFeed();
	void writeToDrive(double* position, double* current, int n);//will start the trajectory
	void readFromDrive(double* timeStamps, double* ml0, double* ml1, int* n); //will block until the trajectory is completed;

	
	//drives
	/* copy/paste FROM OLD CODE developped from MEI */

	int gZAxisDriveNumber = 2;
	int gZAxisSlaveDriveNumber = 1;
	const char* connectString = "etb:ultimet";
	//const char* connectString = "etb:etn://localhost:1140";
	DSA_DRIVE *x = NULL;
	DSA_DRIVE *xprime = NULL;
	DSA_MASTER *ultimet = NULL;
	DSA_RTV_SLOT *posRefSlot = NULL;
	DSA_RTV_SLOT *posSlaveRefSlot = NULL;
	DSA_RTV_DATA *curFfwRefData = NULL;
	DSA_RTV_SLOT *curFfwRefSlot = NULL;
	DSA_RTV_DATA * gML1Slot = NULL;
	DSA_RTV_DATA * gML0Slot = NULL;
	int gSlotDestroyed = false;
	int posRefSlotNr;
	int posSlaveRefSlotNr;
	bool itpModeEnabled = false;
	int rtvInterruptNumber = 0;
	double* gTimes;
	double* gTimesAfter;
	int err;
	__int64 gT1 = 0, gT2;
	__int64 gFrequency;
	CustomerTrajectory gTrajectory;

	long long isoToInc = 0; //either degtoinc or mtoinc 
	double curIsoToInc = 0.0;
	double unitsConversionFactor = 1e-6; //we consider all trajectories are given in m or deg so we need an adjustement factor if it's given in mm or turn or something else to convert back to m or deg
	int gSampleRate = 1; // 1 = 100us, 2 = 200us etc

	//rtx event for feed done
	HANDLE mTrajectoryDone;
	
	void errHandler(int& err);
	void createDrives();
	void destroyDrives();
	void setupRTVSlots();
	void destroyRTVSlots();
	void switchToExternalPositionReferenceMode();
	void switchToAccurETReferenceMode();
	void switchToPCReferenceMode();
	void enableITPReferenceMode();
	void disableITPReferenceMode();
	void startCustomerTrajectory();
	void printTimeStampsToScreen();
	void moveToStartingPos();
	void powerOn();
	void powerOff();
	double timeStamp();
	void clearDriveErrors();
};

#endif
