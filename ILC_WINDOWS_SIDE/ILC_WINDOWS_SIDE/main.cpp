#include "../../Common/RtxLogging.hpp"
#include "../../Common/RtxDataChannel.hpp"

void main()
{
	try
	{
		RtxLogging* rtxLog = RtxLogging::getInstance(); //instantiate the rtx log buffer

		//MsgHeader h;
		//h.mType = ML0;
		//h.mLen = 8;
		//long int ml0 = 1254;
		//char data[4096];
		//for (int i = 0; i < 400; ++i)
		//{
		//	ml0 = i;
		//	h.mLen = i;
		//	memcpy(data, &i, sizeof(i));
		//	WRITE_MSG(&h, data);
		//}

		//++ml0;
		//h.mType = DONE;
		//WRITE_MSG(&h, (char*)&ml0);

		//h.mType = MF31;
		//h.mLen = 4;
		//ml0 = 1255;
		//WRITE_MSG(&h, (char*)&ml0);

		//MsgHeader r;
		//int ml0_r;
		//READ_MSG(&r, &ml0_r);
		//READ_MSG(&r, &ml0_r);

		//RECEIVE_MSG(ML0_RTX_CHANNEL, (char*) &r, &n);

		int n = 1000;
		double* forward = new double[n];
		for (int i = 0; i <n; ++i)
		{
			forward[i] = (double)i / (double)n;
		}

		double* backward = new double[n];
		for (int i = n - 1; i >= 0; --i)
		{
			backward[n - i - 1] = (double)i / (double)n;
		}

		
		int nbLoop = 10;
		MsgHeader h;
		h.mType = _START;
		h.mLen =  sizeof(double) * n;
		MsgHeader r;
		double* timeStamps = new double[n];
		double* ml0 = new double[n];
		double* ml1 = new double[n];
		for (int i = 0; i < nbLoop; ++i)
		{
			if (i % 2 == 0)
			{
				WRITE_MSG(&h, (char*)forward);
				//timestamps
				READ_HDR(&h);
				READ_MSG(timeStamps);

				//ml0
				READ_HDR(&h);
				READ_MSG(ml0);

				//ml1
				READ_HDR(&h);
				READ_MSG(ml1);
			}
			else
			{
				WRITE_MSG(&h, (char*)backward);
				//timestamps
				READ_HDR(&h);
				READ_MSG(timeStamps);

				//ml0
				READ_HDR(&h);
				READ_MSG(ml0);

				//ml1
				READ_HDR(&h);
				READ_MSG(ml1);
			}
		}

		h.mType = _STOP;
		h.mLen = 4;
		WRITE_MSG(&h, (char*)&nbLoop);
		

		DEBUG("done");
		rtxLog->flushtoScreenAndDestroy(); //flush all log messages to screen
	}
	catch (string& exString)
	{
		printf("%s\n", exString.c_str());
	}
	catch (...)
	{

	}
}