#include "RtxLogging.hpp"

#include <stdio.h>
#include <stdarg.h>
#include <windows.h>
#include <tchar.h>
#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications.

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS

RtxLogging::RtxLogging():
	mBufferIncrementSize(MAXIMUM_MSG_SIZE * NUMBER_OF_MESSAGES_SLOTS)
{
	mMessagesBuffer = (char*)malloc(mBufferIncrementSize);
	if (mMessagesBuffer == NULL)
		throw string("Can't allocate initial logging memory buffer");

	mMessagesOffsets.resize(NUMBER_OF_MESSAGES_SLOTS);
	mTotalNumberBytes = 0;
	mMessagesBufferSize = mBufferIncrementSize;
	mMessagesNumber = 0;
}

RtxLogging::~RtxLogging()
{
	//flush the buffer to screen // todo implement a file flush possibility later
	for (int i = 0; i < mMessagesNumber; ++i)
	{
		RtPrintf("%s\n", mMessagesBuffer + mMessagesOffsets[i]);
	}
}

void RtxLogging::flushtoScreenAndDestroy()
{
	if (mRtxLogging != NULL)
		delete mRtxLogging;
}


RtxLogging* RtxLogging::getInstance()
{
	if (mRtxLogging != NULL)
		return mRtxLogging;
	else
	{
		mRtxLogging = new RtxLogging;
		return mRtxLogging;
	}
}

void RtxLogging::printf(const char* file, const int line, const char* format, ...)
{
	if (mRtxLogging == NULL)
	{
		RtxLogging* rtxLog = RtxLogging::getInstance();
	}

	char buffer[MAXIMUM_MSG_SIZE];
	va_list args;
	va_start(args, format);
	int lenPrivate = sprintf(buffer, "%s:%d - ", file, line);
	int lenPublic = vsnprintf(buffer + lenPrivate, MAXIMUM_MSG_SIZE - lenPrivate, format, args);
	++lenPublic; //add \0 char
	int len = lenPublic + lenPrivate;
	if (len >= MAXIMUM_MSG_SIZE)
		throw string("Logging message too big, increase MAXIMUM_MSG_SIZE");

	//save this message offset
	mMessagesOffsets[mMessagesNumber] = mTotalNumberBytes;

	//transfer the local buffer to the messages buffer, slow, to change after if needed
	memcpy(mMessagesBuffer + mMessagesOffsets[mMessagesNumber], buffer, len);

	//increase buffer space if necessary
	mTotalNumberBytes += len;
	if (mTotalNumberBytes >= mMessagesBufferSize - MAXIMUM_MSG_SIZE)
	{
		mMessagesBuffer = (char*)realloc(mMessagesBuffer, mBufferIncrementSize);
		if (mMessagesBuffer == NULL)
			throw string("Can't reallocate initial logging memory buffer");

		mMessagesBufferSize += mBufferIncrementSize;
	}

	//add more slots to the vector offset if necessary
	++mMessagesNumber;
	if (mMessagesNumber >= mMessagesOffsets.size())
		mMessagesOffsets.resize(mMessagesOffsets.size() + NUMBER_OF_MESSAGES_SLOTS);

	va_end(args);
}

RtxLogging* RtxLogging::mRtxLogging = NULL;