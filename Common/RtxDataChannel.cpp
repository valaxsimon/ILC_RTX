#include "RtxDataChannel.hpp"
#include "RtxLogging.hpp"

OneWayChannel::OneWayChannel(int number):
	mMsgMemorySize(MAXIMUM_NUMBER_MSG * sizeof(Message)),
	mChannelNumber(number),
	mHeaderRead(false)
{
	wchar_t fullName[255];

	LPSECURITY_ATTRIBUTES lpMutexAttributes = NULL; //ignored by rtCreateMutex
	//swprintf(fullName, L"RtxDataChannel_RTXTOWINDOWS_Mutex%d", number);
	//m_RTXTOWINDOWS_Mutex = RtCreateMutex(lpMutexAttributes, false, fullName);
	//if (m_RTXTOWINDOWS_Mutex == NULL)
	//	THROW("Can't create mutex");

	//swprintf(fullName, L"RtxDataChannel_WINDOWSTORTX_Mutex%d", number);
	//m_WINDOWSTORTX_Mutex = RtCreateMutex(lpMutexAttributes, false, fullName);
	//if (m_WINDOWSTORTX_Mutex == NULL)
	//	THROW("Can't create  mutex");

	swprintf(fullName, L"RtxDataChannelMsgListReadDoneEvent%d", number);
	mMsgListReadDoneEvent = RtCreateEvent(lpMutexAttributes, false, false, fullName);
	if (mMsgListReadDoneEvent == NULL)
		THROW("Can't create event");

	swprintf(fullName, L"RtxDataChannelMsgListWriteDoneEvent%d", number);
	mMsgListWriteDoneEvent = RtCreateEvent(lpMutexAttributes, false, false, fullName);
	if (mMsgListWriteDoneEvent == NULL)
		THROW("Can't create event");

	//create or open the named data memory area
	void* location;
	swprintf(fullName, L"RtxDataChannelDataMemory%d", number);
	mDataMemorySegment = RtCreateSharedMemory(PAGE_READWRITE, 0, SHARED_MEMORY_SIZE, fullName, &location);
	mStartOfDataMemory = (char*)location;
	if (mDataMemorySegment == NULL)
		THROW("Can't open memory segment");

	//create or open the named msg memory area
	swprintf(fullName, L"RtxDataChannelMsgMemory%d", number);
	mMsgMemorySegment = RtCreateSharedMemory(PAGE_READWRITE, 0, mMsgMemorySize, fullName, &location);
	mStartOfMsgMemory = (char*)location;
	if (mMsgMemorySegment == NULL)
		THROW("Can't open memory segment");

	//create or open the named info memory area
	swprintf(fullName, L"RtxDataChannelInfoMemory%d", number);
	mInfoMemorySegment = RtCreateSharedMemory(PAGE_READWRITE, 0, sizeof(RtxDataChannelInfoHeader), fullName, &location);
	mStartOfInfoMemory = (char*)location;
	if (mInfoMemorySegment == NULL)
		THROW("Can't open memory segment");

	mInfoHdr = (RtxDataChannelInfoHeader*)mStartOfInfoMemory;

	if (RtGetLastError() != ERROR_ALREADY_EXISTS) //no other process has already started the shared memory
	{
		//initialize the read/write values
		mInfoHdr->mWriteDataMemory = 0;
		mInfoHdr->mReadDataMemory = 0;
		mInfoHdr->mWriteMsgMemory = 0;
		mInfoHdr->mReadMsgMemory = 0;
	}
}

OneWayChannel::~OneWayChannel()
{
	//free all RTX synchronisation objects
	RtCloseHandle(mDataMemorySegment);
	RtCloseHandle(mMsgMemorySegment);
	RtCloseHandle(mInfoMemorySegment);
	RtCloseHandle(mMsgListWriteDoneEvent);
	RtCloseHandle(mMsgListReadDoneEvent);
}

void OneWayChannel::pushMsg(Message* msg)
{
	uint64_t mWriteMsgMemoryModulo = mInfoHdr->mWriteMsgMemory % mMsgMemorySize;
	memcpy(mStartOfMsgMemory + mWriteMsgMemoryModulo, msg, sizeof(Message));
	mInfoHdr->mWriteMsgMemory += sizeof(Message);
}

void OneWayChannel::popMsg()
{
	mInfoHdr->mReadMsgMemory += sizeof(Message);
}

void OneWayChannel::calculateFreeBytesInDataBuffer(uint64_t& totalBytesFreeInBuffer)
{
	uint64_t mWriteDataMemoryModulo = mInfoHdr->mWriteDataMemory % SHARED_MEMORY_SIZE;
	uint64_t mReadDataMemoryModulo = mInfoHdr->mReadDataMemory % SHARED_MEMORY_SIZE;

	if (mWriteDataMemoryModulo < mReadDataMemoryModulo)
	{
		totalBytesFreeInBuffer = mReadDataMemoryModulo - mWriteDataMemoryModulo; //difference between read and write pointers
	}
	else
	{
		uint64_t bytesLeftUntilBufferEnd = SHARED_MEMORY_SIZE - mWriteDataMemoryModulo;
		uint64_t bytesFreeUntilReadPointer = (mInfoHdr->mReadDataMemory % SHARED_MEMORY_SIZE);
		totalBytesFreeInBuffer = bytesLeftUntilBufferEnd + bytesFreeUntilReadPointer;
	}
}

void OneWayChannel::sendMsg(MsgHeader* header, void* data)
{
	//check  enough space is present for header + data
	uint64_t totalBytesFreeInBuffer;
	calculateFreeBytesInDataBuffer(totalBytesFreeInBuffer);

	const int hdrSize = sizeof(MsgHeader);
	int totalDataSize = header->mLen + hdrSize;

	if (totalDataSize > SHARED_MEMORY_SIZE) //check data actually fits in an empty buffer
		THROW("Not enough space increase: SHARED_MEMORY_SIZE");

	uint64_t totalNbMessageInMemory = (mInfoHdr->mWriteMsgMemory - mInfoHdr->mReadMsgMemory) / sizeof(Message);
	while ((totalDataSize > totalBytesFreeInBuffer) || (totalNbMessageInMemory > MAXIMUM_NUMBER_MSG)) //not enough space wait for a pulse event from the reader thread
	{
		RtWaitForSingleObject(mMsgListReadDoneEvent, TIMEOUT);

		//recalculate the amount of free space in the buffer
		calculateFreeBytesInDataBuffer(totalBytesFreeInBuffer);

		//at least one slot will have been freed
		totalNbMessageInMemory = (mInfoHdr->mWriteMsgMemory - mInfoHdr->mReadMsgMemory) / sizeof(Message);
	}

	//add the message start/stop limits to the message list
	Message m;
	m.mHdrStart = mInfoHdr->mWriteDataMemory;

	//split the data between end and start of buffer when needed
	uint64_t mWriteDataMemoryModulo = mInfoHdr->mWriteDataMemory % SHARED_MEMORY_SIZE;
	uint64_t bytesLeftUntilBufferEnd = SHARED_MEMORY_SIZE - mWriteDataMemoryModulo;

	//copy the header
	char* pHeader = (char*)header;
	uint64_t bytesToWriteUntilEndOfBuffer = min(hdrSize, bytesLeftUntilBufferEnd);
	memcpy(mStartOfDataMemory + mWriteDataMemoryModulo, pHeader, bytesToWriteUntilEndOfBuffer);
	pHeader += bytesToWriteUntilEndOfBuffer;
	uint64_t leftOverBytes = hdrSize - bytesToWriteUntilEndOfBuffer;
	if (leftOverBytes >0)
		memcpy(mStartOfDataMemory, pHeader, leftOverBytes); //any bytes left to be written

	mInfoHdr->mWriteDataMemory += hdrSize;

	//copy the data
	mWriteDataMemoryModulo = mInfoHdr->mWriteDataMemory % SHARED_MEMORY_SIZE;
	bytesLeftUntilBufferEnd = SHARED_MEMORY_SIZE - mWriteDataMemoryModulo;
	bytesToWriteUntilEndOfBuffer = min(header->mLen, bytesLeftUntilBufferEnd);
	memcpy(mStartOfDataMemory + mWriteDataMemoryModulo, data, bytesToWriteUntilEndOfBuffer);
	leftOverBytes = header->mLen - bytesToWriteUntilEndOfBuffer;
	if (leftOverBytes >0)
		memcpy(mStartOfDataMemory, (char*)data + bytesToWriteUntilEndOfBuffer, leftOverBytes); //any bytes left to be written

	mInfoHdr->mWriteDataMemory += header->mLen;

	//push msg to msg queue
	pushMsg(&m);

	//notify receiver a message is present
	RtPulseEvent(mMsgListWriteDoneEvent);
}

bool OneWayChannel::isEmpty()
{
	return (mInfoHdr->mWriteMsgMemory == mInfoHdr->mReadMsgMemory);
}

void OneWayChannel::receiveHdr(MsgHeader* header)
{
	while (isEmpty()) //wait for a pulsed event from the sender function, block until at least one message is present in the queue
	{
		RtWaitForSingleObject(mMsgListWriteDoneEvent, TIMEOUT);
	}

	//read the header
	const int hdrSize = sizeof(MsgHeader);
	uint64_t mReadDataMemoryModulo = mInfoHdr->mReadDataMemory % SHARED_MEMORY_SIZE;
	uint64_t bytesLeftUntilBufferEnd = SHARED_MEMORY_SIZE - mReadDataMemoryModulo;
	uint64_t bytesToReadUntilBufferEnd = min(hdrSize, bytesLeftUntilBufferEnd);
	memcpy(header, mStartOfDataMemory + mReadDataMemoryModulo, bytesToReadUntilBufferEnd);
	uint64_t leftOverBytes = hdrSize - bytesToReadUntilBufferEnd;
	char* pHdr = (char*)header;
	pHdr += bytesToReadUntilBufferEnd;
	if (leftOverBytes > 0)
		memcpy(pHdr, mStartOfDataMemory, leftOverBytes);

	mInfoHdr->mReadDataMemory += hdrSize;
	memcpy(&mReceiveHdr, header, sizeof(MsgHeader)); //save internally the header to be used when the customer read the message with receiveMsg
	mHeaderRead = true;
}

void OneWayChannel::receiveMsg(void* data)
{
	if (mHeaderRead == false)
		THROW("Call function receiveHdr first before calling receiveHdr");

	//read the data
	uint64_t mReadDataMemoryModulo = mInfoHdr->mReadDataMemory % SHARED_MEMORY_SIZE;
	uint64_t bytesLeftUntilBufferEnd = SHARED_MEMORY_SIZE - mReadDataMemoryModulo;
	uint64_t bytesToReadUntilBufferEnd = min(mReceiveHdr.mLen, bytesLeftUntilBufferEnd);
	memcpy(data, mStartOfDataMemory + mReadDataMemoryModulo, bytesToReadUntilBufferEnd);
	uint64_t leftOverBytes = mReceiveHdr.mLen - bytesToReadUntilBufferEnd;
	char* pData = (char*)data;
	pData += bytesToReadUntilBufferEnd;
	if (leftOverBytes > 0)
		memcpy(pData, mStartOfDataMemory, leftOverBytes);

	mInfoHdr->mReadDataMemory += mReceiveHdr.mLen;

	popMsg();

	RtPulseEvent(mMsgListReadDoneEvent); //in case the sender was waiting on the reader to write more data
	mHeaderRead = false; //reset header read flag to false for next message
}

RtxDataChannel::RtxDataChannel()
{
	mChannels[WINDOWSTORTX] = new OneWayChannel(WINDOWSTORTX);
	mChannels[RTXTOWINDOWS] = new OneWayChannel(RTXTOWINDOWS);
}

RtxDataChannel::~RtxDataChannel()
{
	delete mChannels[WINDOWSTORTX];
	delete mChannels[RTXTOWINDOWS];
}

RtxDataChannel* RtxDataChannel::getInstance()
{
	if (mRtxDataChannel != NULL)
		return mRtxDataChannel;
	else
	{
		//TODO should be protected against other user thread, not done as only one user thread calls READ and WRITE macros for now
		mRtxDataChannel = new RtxDataChannel;
		mGarbageCollector.reset(mRtxDataChannel);
		return mRtxDataChannel;
	}
}

void RtxDataChannel::sendMsg(int channel, MsgHeader* header, void* data)
{
	if (mRtxDataChannel == NULL)
		RtxDataChannel::getInstance();

	mChannels[channel]->sendMsg(header, data);
}

void RtxDataChannel::receiveHdr(int channel, MsgHeader* header)
{
	if (mRtxDataChannel == NULL)
		RtxDataChannel::getInstance();

	mChannels[channel]->receiveHdr(header);
}

void RtxDataChannel::receiveMsg(int channel, void* data)
{
	if (mRtxDataChannel == NULL)
		THROW("Call function receiveHdr first");

	mChannels[channel]->receiveMsg(data);
}

RtxDataChannel* RtxDataChannel::mRtxDataChannel = NULL;
auto_ptr<RtxDataChannel> RtxDataChannel::mGarbageCollector(0);