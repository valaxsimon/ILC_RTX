#include "Drive.hpp"

Axis::Axis()
{

}

Axis::~Axis()
{

}


AxisFactory::AxisFactory()
{

}

AxisFactory::~AxisFactory()
{

}

void AxisFactory::setDriverType(const char* driverType)
{
	
}

Axis* AxisFactory::getAxis(int axisNumber)
{
	if (mDriverType.empty())
	{
		THROW("Call setDriverType first, example: \"etb:USB\" or \"etb:ULTIMET\" ");
	}
	else
	{
		unordered_map<int, Axis*>::iterator iter = mDsaDrivesMap.find(axisNumber);
		if (iter != mDsaDrivesMap.end())
			return iter->second;
		else //open the drive connection
		{
			char connectionString[256];
			sprintf(connectionString, "%s:%d", mDriverType.c_str(), axisNumber);
			Axis* newAxis = new Axis;

			newAxis->mDsaDrive.open(connectionString);
			//insert it in the drive map
			mDsaDrivesMap.insert(std::make_pair(axisNumber, newAxis));

			return newAxis;
		}
	}
}

unordered_map<int, Axis*> AxisFactory::mDsaDrivesMap;
string AxisFactory::mDriverType;

