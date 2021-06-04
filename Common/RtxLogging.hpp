#ifndef RTXLOGGING_HPP
#define RTXLOGGING_HPP

#include <string>
#include <vector>

using namespace std;

#define __FILENAME__ (strrchr(__FILE__,'/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define MAXIMUM_MSG_SIZE 4096
#define NUMBER_OF_MESSAGES_SLOTS 10 //initial number of messages slots of 4096 char

#ifdef _DEBUG
#define DEBUG(...) RtxLogging::getInstance()->printf(__FILENAME__,__LINE__,__VA_ARGS__)
#endif

#define THROW(...) {\
char buffer[255]; \
sprintf(buffer, "ERROR: %s:%d - %s", __FILENAME__, __LINE__, __VA_ARGS__);\
throw string(buffer);\
}

//#ifdef _RELEASE
//#define RELEASE(...) RtxLogging::getInstance()->printf(__FILENAME__,__LINE__,__VA_ARGS__)
//#endif

//RtxLogging collects logging data strings in a memory buffer and print to screen at program exit or crash
class RtxLogging
{
private:
	RtxLogging();
	~RtxLogging();
	static RtxLogging* mRtxLogging;
	char* mMessagesBuffer;
	vector<int> mMessagesOffsets; //offset in buffer for each message
	int mMessagesNumber; //total number of messages
	int mMessagesBufferSize;
	int mTotalNumberBytes; //number of bytes already written in the buffer
	const int mBufferIncrementSize;

public:
	static RtxLogging* getInstance();
	void printf(const char* file, const int line, const char* str,...);
	void flushtoScreenAndDestroy();
};




#endif // "RTXLOGGING_HPP"