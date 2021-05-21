#include "ILC_DLL.hpp"
#include "../../Common/RtxLogging.hpp"
#include "../../Common/RtxDataChannel.hpp"

#include <stdio.h>

int CExecuteTrajectory(double* timeStamps, double* ml0, double* mf231, double* ml1, int* n)
{
	try
	{
		printf("CExecuteTrajectory, write %d elements\n", *n);
		//send theoretical position to RTX
		MsgHeader h;
		h.mType = _START;
		h.mLen = sizeof(double) * (*n);
		WRITE_MSG(&h, (char*)ml0);
		// !!!!!!!!!! Try to add a second message to the set the current FFW values !!!!!!!!!
		//READ_HDR(&h);
		WRITE_MSG(&h, (char*)mf231);

		//read back from RTX

		//timestamps
		printf("CExecuteTrajectory, read timeStamps\n", n);
		READ_HDR(&h);
		READ_MSG(timeStamps);

		//ml0
		printf("CExecuteTrajectory, read ml0\n", n);
		READ_HDR(&h);
		READ_MSG(ml0);

		//ml1
		printf("CExecuteTrajectory, read ml1\n", n);
		READ_HDR(&h);
		printf("header type %d len %d\n", h.mType, h.mLen);
		READ_MSG(ml1);
	}
	catch (string& exString)
	{
		printf("%s\n", exString.c_str());
	}
	catch (...)
	{

	}
	
	return 0;
}