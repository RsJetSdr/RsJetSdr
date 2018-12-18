/*
*/
#ifndef _STREAMINGH
#define _STREAMINGH
#include <algorithm>	//min
#include <climits>		//SHRT_MAX
#include <atomic>
#include <mutex>
#include <cstdio>
#include <vector>
#include <iostream>
#include <iomanip>

#include "Rsjet\RsJetSuite.h"
#include "Rs_FX3_USB.h"
#include "liquid/liquid.h"
#include "RsSDRDefs.h"
#ifdef _MSC_VER
#include <windows.h>
#define usleep(t) Sleep((t)/1000)
#else
#include <unistd.h>
#endif




#define BYTES_PER_SAMPLE		   4
#define ELEMS_PER_SAMPLE		(2)
#define DEFAULT_BUFFER_LENGTH     (2048)
#define DEFAULT_NUM_BUFFERS       (8)




class Streaming { 
/*	typedef struct {
		double freq;
		AttnStr att_str;
	}hwReceiver;
*/
public:
	Streaming();
	~Streaming();

//	void setAttn(AttnStr data);
	AttnStr getAttn(void);


//	void setFrequency(const double frequency);
	double getFrequency(void) const;

	void setAttnFreq(FreqAttnStr m_FreqAttnStr);

	void setSampleRate(const long long rate);
	long getSampleRate(void) const;


//Stream Data 
	unsigned int getStreamMTU() const;
	void*  SetupStream(RxFormat format);
	void  CloseStream(void);
	int activateStream(const size_t mode = 0,const long long timeNs = 0,const size_t numElems = 0);
	int deactivateStream(const long long timeNs = 0);
	
	int readStream(void * const *buffs, const size_t numElems,
	int &flags, long long &timeNs, const long timeoutUs = 100000);
	std::atomic_bool streamActive;

private:
	const ULONG numBuffers		= DEFAULT_NUM_BUFFERS;
	const ULONG const_BufferElems = DEFAULT_BUFFER_LENGTH;
		
	std::atomic_bool rxFormat;
	CCyUSBEndPoint *eptInStream;

	std::atomic<size_t> modeStream; //burst/raw

	double	centerFrequency;
	mutable std::mutex Reg_mutex;
	std::mutex	Buf_mutex;
	
	std::atomic_uint shortsPerWord;
	size_t bufferLength;
   
	//flags
	std::atomic_bool _overflowEvent;
	std::atomic_bool resetBuffer;

	
	std::condition_variable _buf_cond;
	std::atomic<size_t>	_buf_count;
	
	size_t	_buf_head;
	size_t	_buf_tail;
	std::vector<std::vector<short> > __buffs;
	std::atomic_size_t bufferedElems;
	uint8_t *currentBuff;

	std::thread _rx_async_thread;
	void rx_async_operation(void);
	void rx_callback(void *IQdata, uint32_t len);
	int acquireReadBuffer(size_t &handle, const void **buffs, int &flags,
	long long &timeNs, const long timeoutUs = 100000);
	void releaseReadBuffer(const size_t handle);


    #define h_len 21
	float hFir[h_len];
	firfilt_crcf firfilt;

	FreqAttnStr receiverHW;
	void  SetHardware(void);
};
#endif