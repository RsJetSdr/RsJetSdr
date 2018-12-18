#include "Rs_FX3_USB.h"
#include "SoapyRsJet.hpp"
#include <omp.h>


//=========================================================
std::vector<std::string> RsJet::getStreamFormats(const int direction, const size_t channel) const
{
	std::vector<std::string> formats;
	formats.push_back("CS16");
	formats.push_back("CF32");
	return formats;
}
//=========================================================
std::string RsJet::getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const
{
	fullScale = 32768.0;
//	return "CS16";

	fullScale = 1.0;
	return "CF32";
}

SoapySDR::ArgInfoList RsJet::getStreamArgsInfo(const int, const size_t) const
{
	SoapySDR::ArgInfoList streamArgs;

	SoapySDR::ArgInfo buffersArg;
	buffersArg.key = "buffers";
	buffersArg.value = STRINGIFY(DEFAULT_NUM_BUFFERS);
	buffersArg.name = "Buffer Count";
	buffersArg.description = "Number of async USB buffers.";
	buffersArg.units = "buffers";
	buffersArg.type = SoapySDR::ArgInfo::INT;
	streamArgs.push_back(buffersArg);

	SoapySDR::ArgInfo lengthArg;
	lengthArg.key = "buflen";
	lengthArg.value = STRINGIFY(131072);
	lengthArg.name = "Buffer Length";
	lengthArg.description = "Number of bytes per USB buffer, the number must be a multiple of 1024.";
	lengthArg.units = "bytes";
	lengthArg.type = SoapySDR::ArgInfo::INT;
	streamArgs.push_back(lengthArg);

	SoapySDR_log(SOAPY_SDR_DEBUG, "getStreamArgsInfo");

	return streamArgs;
}
//=======================================================================

/*******************************************************************
* Stream API
******************************************************************/
SoapySDR::Stream *RsJet::setupStream(const int direction,
	const std::string &format,
	const std::vector<size_t> &channels,
	const SoapySDR::Kwargs &args)
{
	// check the channel configuration
	if (direction != SOAPY_SDR_RX) {
		throw std::runtime_error("RsJetSDR is RX only, use SOAPY_SDR_RX");
	}

	if (channels.size() > 1 or (channels.size() > 0 and channels.at(0) != 0))
	{
		throw std::runtime_error("setupStream invalid channel selection");
	}
	char *pchar;
	// check the format
	if (format == "CS16")
	{
		rxFormat = RX_FORMAT_CS16;
		shortsPerWord = 1;
		bufferLength = bufferElems * elementsPerSample * shortsPerWord;
		pchar = "Using format CS16.";
	}
	else if (format == "CF32")
	{
		rxFormat = RX_FORMAT_CF32;
		shortsPerWord = sizeof(float) / sizeof(short);
		bufferLength = bufferElems * elementsPerSample * shortsPerWord;  // allocate enough space for floats instead of shorts
		pchar = "Using format CF32.";
	}
	else
	{
		throw std::runtime_error("setupStream invalid format '" + format +
			"' -- Only CS16 or CF32 are supported by the SoapySDRPlay module.");
	}

	SoapySDR_logf(mDEBUG, "Set %d buffers : buffer length %d : %s ", numBuffers, bufferLength, pchar);

	std::lock_guard<std::mutex> lock(_buf_mutex);

	// clear async fifo counts
	_buf_tail = 0;
	_buf_head = 0;
	_buf_count = 0;

	// allocate buffers
	__buffs.resize(numBuffers);
	for (auto &buff : __buffs) buff.reserve(bufferLength);
	for (auto &buff : __buffs) buff.clear();

	return (SoapySDR::Stream *) this;
}





//==================================================
void RsJet::closeStream(SoapySDR::Stream *stream)
{
	SoapySDR_log(mDEBUG, "closeStream");
	std::lock_guard <std::mutex> lock(_general_state_mutex);
	//if (streamActive)
	{
		if (_rx_async_thread.joinable()) {
			streamActive = false;
			_rx_async_thread.join();
			SoapySDR_log(mDEBUG, "closeStream");
		}

	}
}

size_t RsJet::getStreamMTU(SoapySDR::Stream *stream) const {
	// is a constant in practice
	return bufferElems;
}
//=============================================================================
int RsJet::activateStream(
	SoapySDR::Stream *stream,
	const int flags,
	const long long timeNs,
	const size_t numElems)
{
	if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;
	resetBuffer = true;
	bufferedElems = 0;

	eptInStream = pUSBDev->StreameptIn;
	if (eptInStream == NULL) 	return -1;


	//eptIn->Reset();
	//eptIn->Abort();
	//pUSBDev->StopCommand();

	//start the async thread
	if (not _rx_async_thread.joinable())
	{
		SoapySDR_log(mDEBUG, "activateStream");
		_rx_async_thread = std::thread(&RsJet::rx_async_operation, this);
		while (!streamActive.load());

	}

	return 0;
}

//=============================================================
int RsJet::deactivateStream(SoapySDR::Stream *stream, const int flags, const long long timeNs)
{
	if (flags != 0) return SOAPY_SDR_NOT_SUPPORTED;

	if (_rx_async_thread.joinable()) {
		streamActive = false;
		_rx_async_thread.join();
		SoapySDR_log(mDEBUG, "deactivateStream");

	}
	return 0;
}
//===============================================================
int RsJet::readStream(SoapySDR::Stream *stream,
	void * const *buffs,
	const size_t numElems,
	int &flags,
	long long &timeNs,
	const long timeoutUs)
{
	if (!streamActive)
	{
		return 0;
	}

	// this is the user's buffer for channel 0
	void *buff0 = buffs[0];

	// are elements left in the buffer? if not, do a new read.
	if (bufferedElems == 0)
	{
		int ret = this->acquireReadBuffer(stream, _currentHandle, (const void **)&_currentBuff, flags, timeNs, timeoutUs);
		if (ret < 0)
		{
			return ret;
		}
		bufferedElems = ret;
	}

	size_t returnedElems = std::min(bufferedElems.load(), numElems);

	// copy into user's buff0
	if (RX_FORMAT_CS16 == rxFormat)
	{
		std::memcpy(buff0, _currentBuff, returnedElems * 2 * sizeof(short));
	}
	else
	{
		std::memcpy(buff0, (float *)_currentBuff, returnedElems * 2 * sizeof(float));
	}

	// bump variables for next call into readStream
	bufferedElems -= returnedElems;

	// scope lock here to update _currentBuff position
	{
		std::lock_guard <std::mutex> lock(_buf_mutex);
		_currentBuff += returnedElems * elementsPerSample * shortsPerWord;
	}

	// return number of elements written to buff0
	if (bufferedElems != 0)
	{
		flags |= SOAPY_SDR_MORE_FRAGMENTS;
	}
	else
	{
		this->releaseReadBuffer(stream, _currentHandle);
	}
	return (int)returnedElems;
}

/*******************************************************************
* Direct buffer access API
******************************************************************/
size_t RsJet::getNumDirectAccessBuffers(SoapySDR::Stream *stream)
{
	std::lock_guard <std::mutex> lock(_buf_mutex);

	return __buffs.size();
}
int RsJet::getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs)
{
	std::lock_guard <std::mutex> lock(_buf_mutex);

	buffs[0] = (void *)__buffs[handle].data();
	return 0;
}
//========================================================================================================
int RsJet::acquireReadBuffer(SoapySDR::Stream *stream,
	size_t &handle,
	const void **buffs,
	int &flags,
	long long &timeNs,
	const long timeoutUs)
{
	std::unique_lock <std::mutex> lock(_buf_mutex);

	// reset is issued by various settings
	// overflow set in the rx callback thread
	if (resetBuffer || _overflowEvent)
	{
		// drain all buffers from the fifo
		_buf_tail = 0;
		_buf_head = 0;
		_buf_count = 0;
		for (auto &buff : __buffs) buff.clear();
		_overflowEvent = false;
		if (resetBuffer)
		{
			resetBuffer = false;
		}
		else
		{
			SoapySDR_log(SOAPY_SDR_SSI, "O");
			return SOAPY_SDR_OVERFLOW;
		}
	}

	// wait for a buffer to become available
	if (_buf_count == 0)
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_buf_count == 0)
		{
			SoapySDR_log(SOAPY_SDR_SSI, "T");
			return SOAPY_SDR_TIMEOUT;
		}
	}

	// extract handle and buffer
	handle = _buf_head;
	buffs[0] = (void *)__buffs[handle].data();
	flags = 0;

	_buf_head = (_buf_head + 1) % numBuffers;

	// return number available
	return (int)(__buffs[handle].size() / (elementsPerSample * shortsPerWord));
}
//==============================================================
void RsJet::releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle)
{
	std::lock_guard <std::mutex> lock(_buf_mutex);
	__buffs[handle].clear();
	_buf_count--;
}
//=============================================================
/*******************************************************************
* Async thread work
******************************************************************/
void RsJet::rx_callback(void *IQdata, uint32_t numSamples)
{
	std::lock_guard<std::mutex> lock(_buf_mutex);

	if (_buf_count == numBuffers)
	{
		_overflowEvent = true;
		return;
	}

	int spaceReqd = numSamples * elementsPerSample * shortsPerWord;


	if ((__buffs[_buf_tail].size() + spaceReqd) >= (bufferLength / 1))
	{
		// increment the tail pointer and buffer count
		_buf_tail = (_buf_tail + 1) % numBuffers;
		_buf_count++;

		// notify readStream()
		_buf_cond.notify_one();
	}

	// get current fill buffer
	auto &buff = __buffs[_buf_tail];

	buff.resize(buff.size() + spaceReqd);

	// copy into the buffer queue
	unsigned int i = 0;

	if (rxFormat == RX_FORMAT_CS16)
	{
		short *dptr = buff.data();
		dptr += (buff.size() - spaceReqd);
		std::memcpy(dptr, IQdata, numSamples * BYTES_PER_SAMPLE);
	}
	else
	{
		/*
		Todo
		add cast 
		*/
		float *dptr = (float *)buff.data();
		//liquid_float_complex *dptr = (liquid_float_complex *)buff.data();
		dptr += ((buff.size() - spaceReqd) / shortsPerWord);
		const float factor = 1.0f / 32768.f;
		int16_t *source = (int16_t *)IQdata;
		liquid_float_complex Y;

		//#pragma omp parallel for 
		if ((centerFrequency/1000000) < 1444)
	
			for (i = 0; i < numSamples; i++)
			{

				Y.imag = *source++ * factor;
				Y.real = *source++ * factor;

				firfilt_crcf_push(firfilt, Y); // push input sample
				firfilt_crcf_execute(firfilt, &Y); // compute output
				*dptr++ = Y.imag;
				*dptr++ = Y.real;
			}
		
		else 
			for (i = 0; i < numSamples; i++)
			{

				Y.real = *source++ * factor;
				Y.imag = *source++ * factor;

				firfilt_crcf_push(firfilt, Y); // push input sample
				firfilt_crcf_execute(firfilt, &Y); // compute output
				*dptr++ = Y.real;
				*dptr++ = Y.imag;
				

				/*
				*dptr++ = *source++ * factor;
				*dptr++ = *source++ * factor;
				*/
			}
		





	}
	return;
}
//=============================================================
void RsJet::rx_async_operation(void)
{

	long		readLength;
	int			nCount;
	size_t      _bufferLength = bufferElems * BYTES_PER_SAMPLE;

	char Life[] = { '|','/','-','\\' };
	int p = 0;
	pUSBDev->StopCommand();
	eptInStream->Abort();
	eptInStream->SetXferSize((ULONG)_bufferLength);

	PUCHAR buffers[MAX_TRANSFER];
	PUCHAR contexts[MAX_TRANSFER];

	/*
	PUCHAR			*buffers = new PUCHAR[MAX_TRANSFER];
	PUCHAR			*contexts = new PUCHAR[MAX_TRANSFER];
	*/

	OVERLAPPED		inOvLap[MAX_TRANSFER];
	unsigned long totalBytesReceived = 0; //for data rate calculation


										  // Allocate all the buffers for the queues
	for (nCount = 0; nCount < MAX_TRANSFER; nCount++)
	{
		buffers[nCount] = new UCHAR[_bufferLength];
		memset(&inOvLap[nCount], 0, sizeof(OVERLAPPED)); //Clear
		inOvLap[nCount].hEvent = CreateEvent(NULL, false, false, NULL);
		contexts[nCount] = nullptr;
	}



	// Queue-up the first batch of transfer requests
	for (nCount = 0; nCount < MAX_TRANSFER; nCount++) {
		//BeginDataXFer will kick start the IN transactions.................
		contexts[nCount] = eptInStream->BeginDataXfer(buffers[nCount], (long)_bufferLength, &inOvLap[nCount]);
	}

	nCount = 0;
	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = t1;
	
//	SoapySDR::log(SOAPY_SDR_SSI, "\n>>>>>> Start >>>>>>>\n");

	/////// Send start  to the device //////////////////////////////
	pUSBDev->RawCommand();// OutCmdReg(MB_RAW_CTRL, MODE_RAW | (2048));

	streamActive = true;

	//>>>>>>>>>>>>>>
	while (streamActive)
	{
		//// Wait till the transfer completion.. //
		if (eptInStream->WaitForXfer(&inOvLap[nCount], 100))
		{
			readLength = (long)_bufferLength;
			//== Read the trasnferred data from the device ==//
			eptInStream->FinishDataXfer(buffers[nCount], readLength,
				&inOvLap[nCount],
				contexts[nCount]);
			totalBytesReceived += readLength;

			if (readLength == _bufferLength)
				rx_callback(buffers[nCount], readLength / BYTES_PER_SAMPLE);

			// Re-submit this queue element to keep the queue full
			contexts[nCount] = eptInStream->BeginDataXfer(buffers[nCount], (long)_bufferLength, &inOvLap[nCount]);
			nCount = (nCount + 1) % MAX_TRANSFER;
			//
		}
		else {
			totalBytesReceived = 0;

		}

		t2 = std::chrono::high_resolution_clock::now();
		auto timePeriod = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		if (timePeriod >= 1000)
		{
			t1 = t2;
			//total number of bytes sent per second
			double dataRate = 1000.0*totalBytesReceived / timePeriod;
#ifdef NDEBUG
			SoapySDR::logf(SOAPY_SDR_SSI, "\r Rx: %.3f MB/s %c", dataRate / 1000000.0, Life[++p & 3]);

#endif
			totalBytesReceived = 0;
			//	rxDataRate_Bps.store((uint32_t)dataRate);
		}

	}
	
	//<<<<<<<<<<<<<<<<<<<<<<<<
	pUSBDev->StopCommand();
	eptInStream->Abort();
	Sleep(1);
	SoapySDR::log(SOAPY_SDR_TRACE, "\n");
	for (nCount = 0; nCount< MAX_TRANSFER; nCount++)
	{
		eptInStream->WaitForXfer(&inOvLap[nCount], 100);
		eptInStream->FinishDataXfer(buffers[nCount], readLength,
			&inOvLap[nCount],
			contexts[nCount]);
		CloseHandle((HANDLE)(inOvLap[nCount].hEvent));
		delete[] buffers[nCount];
	}
	// do the needed cleanup.
	//delete[]buffers;
	//delete[]contexts;

//	SoapySDR::log(SOAPY_SDR_SSI, "\n>>>>>> End >>>>>>>\n");
}