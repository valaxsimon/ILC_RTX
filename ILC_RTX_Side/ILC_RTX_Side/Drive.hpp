#ifndef DRIVE_HPP
#define DRIVE_HPP

#include <windows.h>
#include <tchar.h>
#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS

#include <string>
#include <vector>
#include <unordered_map>

#include "../../Common/RtxLogging.hpp"

using namespace std;

#define ETEL_OO_API

#include <dex40.h>
#include <dmd40.h>
#include <etb40.h>
#include <dsa40.h>
#include <tra40.h>
#include <esc40.h>
#include <lib40.h>

//Axis are singleton for a definite axis number
class Axis
{
friend class AxisFactory;

private:
	Axis();
	~Axis();
	DsaDrive mDsaDrive; //the actual underlying physical drive

public:
};


class AxisFactory
{

private:
	static unordered_map<int, Axis*> mDsaDrivesMap; //vector of associated drive objects
	static string mDriverType;
	AxisFactory();
	~AxisFactory();

public:
	void setDriverType(const char* driverType);
	Axis* getAxis(int axisNumber);
};

#endif // "DRIVE_HPP"