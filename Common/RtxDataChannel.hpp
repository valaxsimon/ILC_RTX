#ifndef RTXDATACHANNEL_HPP
#define RTXDATACHANNEL_HPP

#include <stdio.h>
#include <stdarg.h>
#include <windows.h>
#include <memory>
#include <tchar.h>
#include <rtapi.h>    // RTX64 APIs that can be used in real-time and Windows applications
#include <unordered_map>
#include <list>
#include <atomic>

#ifdef UNDER_RTSS
#include <rtssapi.h>  // RTX64 APIs that can only be used in real-time applications.
#endif // UNDER_RTSS

#include <string>
#include <vector>


using namespace std;

//user defined values
#define TIMEOUT 3000 //maximum number of millisecond to wait for RTX events to be siganlled otherwise check the queue again
#define SHARED_MEMORY_SIZE 100000 //size in bytes of the full shared memory (all the data channel will share this memory)
#define MAXIMUM_NUMBER_MSG 150 //maximum number of independent messages that can be in the shared memory

//component defined macros
#define WINDOWSTORTX 0
#define RTXTOWINDOWS 1

//depending of which code call those macro we use one channel or the other
#ifdef UNDER_RTSS
#define WRITE_MSG(...) RtxDataChannel::getInstance()->sendMsg(RTXTOWINDOWS,__VA_ARGS__);
#define READ_HDR(...) RtxDataChannel::getInstance()->receiveHdr(WINDOWSTORTX,__VA_ARGS__);
#define READ_MSG(...) RtxDataChannel::getInstance()->receiveMsg(WINDOWSTORTX,__VA_ARGS__);
#else
#define WRITE_MSG(...) RtxDataChannel::getInstance()->sendMsg(WINDOWSTORTX,__VA_ARGS__);
#define READ_HDR(...) RtxDataChannel::getInstance()->receiveHdr(RTXTOWINDOWS,__VA_ARGS__);
#define READ_MSG(...) RtxDataChannel::getInstance()->receiveMsg(RTXTOWINDOWS,__VA_ARGS__);
#endif

enum MsgType
{
	_START = 0,
	_STOP,
	_TIMESTAMPS,
	_ML0,
	_ML1,
};

#pragma pack(push, 1)
struct RtxDataChannelInfoHeader //shared information about the RtxDataChannel structure so accessible from windows / rtx
{
	uint64_t mWriteDataMemory;
	uint64_t mReadDataMemory;
	uint64_t mWriteMsgMemory;
	uint64_t mReadMsgMemory;
};

struct MsgHeader
{
	MsgType mType;
	int mLen;
};

struct Message //info for the Msg shared memory queue
{
	uint64_t mHdrStart;
};
#pragma pack(pop)

class OneWayChannel
{
public:

	OneWayChannel(int number);
	~OneWayChannel();
	void pushMsg(Message* msg);
	void popMsg();
	bool isEmpty(); //check if any message is present
	void calculateFreeBytesInDataBuffer(uint64_t& totalBytesFreeInBuffer);
	void sendMsg(MsgHeader* header, void* data); //block if not enough space in shared memory until free space is available
	void receiveHdr(MsgHeader* header);
	void receiveMsg(void* data); //if no message is present, block until the next message is available

	const int mChannelNumber;
	//HANDLE m_RTXTOWINDOWS_Mutex; //will ensure that multiple processes do not update the shared memory area at the same time
	//HANDLE m_WINDOWSTORTX_Mutex;
	HANDLE mMsgListWriteDoneEvent;
	HANDLE mMsgListReadDoneEvent;
	HANDLE mDataMemorySegment;
	HANDLE mMsgMemorySegment;
	HANDLE mInfoMemorySegment;
	char* mStartOfDataMemory; //pointer to start of data memory segment
	char* mStartOfMsgMemory; //pointer to start of msg memory segment
	char* mStartOfInfoMemory; //pointer to shared info area about pointers and memory filled
	RtxDataChannelInfoHeader* mInfoHdr;
	const DWORD mMsgMemorySize;
	MsgHeader mReceiveHdr; //last received header
	bool mHeaderRead; //if true the user has read a header with READ_HDR macro and is allowed to read the data with READ_MSG
};

/*Manage shared memory between RTX and Windows
  Set up a dual communication channel*/
class RtxDataChannel
{
private:
	RtxDataChannel();
	static RtxDataChannel* mRtxDataChannel;
	static auto_ptr<RtxDataChannel> mGarbageCollector; //destroy singleton automatically

	OneWayChannel* mChannels[2]; //to/from RTX/Windows channels

public:
	~RtxDataChannel();
	static RtxDataChannel* getInstance();
	
	void sendMsg(int channel, MsgHeader* header, void* data); //block if not enough space in shared memory until free space is available
	void receiveHdr(int channel, MsgHeader* header);
	void receiveMsg(int channel, void* data); //if no message is present, block until the next message is available
};

#endif //RTXDATACHANNEL_HPP
