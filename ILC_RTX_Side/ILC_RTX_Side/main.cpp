#include "Drive.hpp"
#include "../../Common/RtxLogging.hpp"
#include "../../Common/RtxDataChannel.hpp"
#include "PositionFeed.hpp"

//=================================================================
//
// To compile and run this project map u: to EDI RTX64 path 
// subst U: C:\EDI\EDI-rtx64-VisualStudio2015-c-4.22A\EDI-4.22A
// subst U: C:\EDI\EDI-4.24A-rtx64_3-vs2015-c\EDI-4.24A
// 
//=================================================================

void main()
{
	try
	{
		RtxLogging* rtxLog = RtxLogging::getInstance(); //instantiate the rtx log buffer
		PositionFeed pFeeder; //instantiate the drive trajectory feeder
		RtPrintf("Drive initiated successfully\nYou can start the windows side application\n");
		int nElem;
		double* ML0 = NULL;
		double* ML0_read = NULL;
		double* MF231 = NULL;
		double* ML1 = NULL;
		double* timeStamps = NULL;
		MsgHeader hdr; //messages going through the RTX - Windows shared memory dual communication channel will have this header, it can be changed to tailor user needs
		do
		{
			READ_HDR(&hdr); //first read the header to find the message type and data length, blocks until a message is ready
			if (hdr.mType == _START)
			{
				//allocate memory for the data length (trajectory)
				int nNewElem = hdr.mLen / sizeof(double);
				if (ML0 == NULL)
				{
					nElem = nNewElem;
					ML0 = (double*) malloc(sizeof(double) * nElem);
					ML0_read = (double*)malloc(sizeof(double) * nElem);
					MF231 = (double*) malloc(sizeof(double) * nElem);
					ML1 = (double*)malloc(sizeof(double) * nElem);
					timeStamps = (double*)malloc(sizeof(double) * nElem);
				}
				else
				{
					if (nNewElem > nElem) //reallocate some memory if the user do not sent trajectory of the same length every time
					{
						nElem = nNewElem;
						ML0 = (double*)realloc(ML0, sizeof(double) * nElem);
						ML0_read = (double*)realloc(ML0_read, sizeof(double) * nElem);
						MF231 = (double*)realloc(MF231, sizeof(double) * nElem);
						ML1 = (double*)realloc(ML1, sizeof(double) * nElem);
						timeStamps = (double*)realloc(timeStamps, sizeof(double) * nElem);
					}
					else
					{
						nElem = nNewElem;
					}
				}

				READ_MSG(ML0); //now we allocated the memory for ML0 => read the trajectory
				/*MsgHeader h;
				READ_HDR(&h);*/
				READ_HDR(&hdr);
				READ_MSG(MF231); //now we allocated the memory for MF231 => read the trajectory
				pFeeder.writeToDrive(ML0, MF231, nElem); //start feeding the trajectory to the drive, blocks until the trajectory has finished executing
				pFeeder.readFromDrive(timeStamps, ML0_read, ML1, &nElem); //copy back RTV of interest that were saved during each callback during trajectory execution

				//send the values back to Windows
				MsgHeader w;
				w.mType = _TIMESTAMPS;
				w.mLen = sizeof(double) * nElem;
				WRITE_MSG(&w, (char*)timeStamps); //write values to the dual-channel communication object, will block until space is valid in the buffer
				w.mType = _ML0;
				WRITE_MSG(&w, (char*)ML0_read);
				w.mType = _ML1;
				WRITE_MSG(&w, (char*)ML1);
			}
			else if (hdr.mType == _STOP) //exit the loop and close this program when the _STOP message is received
				break;
		} while (1);

		DEBUG("Trajectory Completed");
		rtxLog->flushtoScreenAndDestroy(); //flush all log messages to screen
	}
	catch (string& exString) //very basic error reporting with strings
	{
		printf("Error: %s\n", exString.c_str());
	}
	catch (...)
	{

	}
}