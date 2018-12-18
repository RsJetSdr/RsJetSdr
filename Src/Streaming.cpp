/*
*/
#include "Streaming.h"
//=======================================================================================
Streaming::Streaming() :
	resetBuffer(true),
	bufferedElems(0),
	streamActive(false) 
{

	shortsPerWord = 1;
	bufferLength = 0;
	
	liquid_firdes_kaiser(h_len, 0.41f, 40.0f, 0.0f, hFir);
	firfilt = firfilt_crcf_create(hFir, h_len);
	firfilt_crcf_set_scale(firfilt, 2.0f*0.43f);
	std::cout << std::setprecision(8);

}
//=======================================================================================
Streaming::~Streaming() {
	CloseStream();
	firfilt_crcf_destroy(firfilt);
}
unsigned int Streaming::getStreamMTU() const {
	// is a constant in practice
	return const_BufferElems;
}
//=================================================================
void  Streaming::SetHardware(void)
{
	
	std::cout << "Streaming::Set Freq :" << receiverHW.freq / 1000000.0 << " MHz";// << std::endl;
	

	//Check band High/Low
	if (receiverHW.freq > LOW_FREQ)
	{
		std::cout << "\tSet Attn High :" << receiverHW.attnStr.att1 << " "\
			<< receiverHW.attnStr.att2 << " "\
			<< receiverHW.attnStr.att3 << " " << std::endl;

		//Convert to Hz
		uint32_t mod = uint64_t(receiverHW.freq) % 1000000;
		
		//Convert to MHz
		uint16_t adf_freq = (uint16_t)(receiverHW.freq / 1000000.0);  //Точность синтезатора 1 MHz

		//Check inverse spectrum
		double  FreqDDC = (adf_freq < 1444) ?
			CENTER_REC_140MHZ + mod : CENTER_FFT_140MHZ - mod; 
		//=====================================================
		pUSBDev->SetFreqDDC(FreqDDC);
		{
			uint8_t cmdbuf[16];
			cmdbuf[0] = 0x05;
			cmdbuf[1] = 10;   // Size 10 * 8 uS = 80 uS
			cmdbuf[2] = 0x20;   // command
			
			cmdbuf[3] = adf_freq & 0xFF;;
			cmdbuf[4] = (adf_freq >> 8) & 0xFF;
			/*
			modeAttn
			0: Set All Struct
			1: Set Attn1 Struct, Attn2 & Attn3 predefined (eeprom)
			2: Set All  predefined (eeprom)
			*/
			cmdbuf[5] = receiverHW.attnStr.mode; //External
			cmdbuf[6] = (63 - (receiverHW.attnStr.att1 * 2));
			cmdbuf[7] = (63 - (receiverHW.attnStr.att2 * 2));
			cmdbuf[8] = (63 - (receiverHW.attnStr.att3 * 2));
			DWORD Snt = 0;
			pUSBDev->OutSerial(cmdbuf, &Snt, 4); //Ser Freq & Attns
		}
		pUSBDev->SelectBand(true);
	}
	else {
		pUSBDev->SetFreqDDC(receiverHW.freq); //Ser Freq
		pUSBDev->SelectBand(false);
		pUSBDev->OutCmdReg(MB_ATTN_REG, (63 - (receiverHW.attnStr.att_low * 2) & 0x3F));
		std::cout << "\tSet Attn Low:" << receiverHW.attnStr.att_low << std::endl;

	}
	centerFrequency = receiverHW.freq;
}

void Streaming::setAttnFreq(FreqAttnStr m_FreqAttnStr) {
	receiverHW.attnStr = m_FreqAttnStr.attnStr;
	receiverHW.freq = m_FreqAttnStr.freq;
	SetHardware();
	resetBuffer = true;

}
/*
void Streaming::setAttn(AttnStr data)
{
	receiverHW.attnStr = data;
	if (pUSBDev) 
		SetHardware();
	resetBuffer = true;

}
*/
//=====================================
AttnStr Streaming::getAttn(void)
{
	return receiverHW.attnStr;
}
//=======================================================================================
/*
void Streaming::setFrequency(const double frequency)
{
	receiverHW.freq = frequency;
	SetHardware();
	resetBuffer = true;
}
*/
//=======================================================================================
double Streaming::getFrequency(void) const
{
	return centerFrequency;
}
//=======================================================================================
void Streaming::setSampleRate(const long long rate)
{
	if (pUSBDev) 
			pUSBDev->SetSampleRate((double)rate);
	
	resetBuffer = true;
}
//=======================================================================================
long Streaming::getSampleRate(void) const
{
	return (long)pUSBDev->GetSampleRate();
}
//================================================================================================
void *Streaming::SetupStream(RxFormat format)
{
//	SoapySDR_logf(mDEBUG, "Set %d buffers : buffer length %d : %s ", numBuffers, bufferLength, pchar);
	if (format == RX_FORMAT_CS16)
	{
		rxFormat = RX_FORMAT_CS16;
		shortsPerWord = sizeof(short)/sizeof(short); //Math 1
		bufferLength = const_BufferElems/*2048*/ * ELEMS_PER_SAMPLE * shortsPerWord;
		
	}
	else if (format == RX_FORMAT_CF32)
	{
		rxFormat = RX_FORMAT_CF32;
		shortsPerWord = sizeof(float)/sizeof(short); //Math 2
		bufferLength = const_BufferElems/*2048*/ * ELEMS_PER_SAMPLE * shortsPerWord;  // allocate enough space for floats instead of shorts
		
	}
	else
	{
		throw std::runtime_error("SetupStream invalid format ");
	}

	std::lock_guard<std::mutex> lock(Buf_mutex);
// clear async fifo counts
	_buf_tail = 0;
	_buf_head = 0;
	_buf_count = 0;

// allocate buffers base short
	__buffs.resize(numBuffers); 

	for (auto &buff : __buffs) buff.reserve(bufferLength);
	for (auto &buff : __buffs) buff.clear();

	return (void *) this;
}
//==================================================
void Streaming::CloseStream(void)
{
	std::lock_guard <std::mutex> lock(Reg_mutex);
	{
		if (_rx_async_thread.joinable()) {
			streamActive = false;
			_rx_async_thread.join();
		}
	}
	for (auto &buff : __buffs) buff.clear();
	__buffs.clear();
}
//=============================================================================
int Streaming::activateStream(const size_t mode,const long long timeNs,const size_t numElems)
{
	resetBuffer = true;
	bufferedElems = 0;
 	
	if (pUSBDev->StreameptIn == NULL) //Not open Device
					return SDR_STREAM_ERROR; 

	eptInStream = pUSBDev->StreameptIn;
	//if active stream, deactive
	if (_rx_async_thread.joinable())
									deactivateStream();
	//Set mode
	modeStream = mode;

	//start the async thread
		_rx_async_thread = std::thread(&Streaming::rx_async_operation, this);
		while (!streamActive);
	return 0;
}
//=============================================================
int Streaming::deactivateStream(const long long timeNs)
{
	if (_rx_async_thread.joinable()) {
		streamActive = false;
		_rx_async_thread.join();
	}
	return 0;
}
//========================================================================================================
int Streaming::readStream(void * const *buffs,
	const size_t numElems,
	int &flags,
	long long &timeNs,
	const long timeoutUs)
{
	if (!streamActive)
					return 0;
	
	size_t  handle;

	// this is the user's buffer for channel 0
	void *buff0 = buffs[0];

	// are elements left in the buffer? if not, do a new read.
	if (bufferedElems == 0)
	{
		int ret = this->acquireReadBuffer(handle, (const void **)&currentBuff, flags, timeNs, timeoutUs);
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
		std::memcpy(buff0, currentBuff, returnedElems * 2 * sizeof(short));
	}
	else
	{
		std::memcpy(buff0, (float *)currentBuff, returnedElems * 2 * sizeof(float));
	}
	

	// bump variables for next call into readStream
	bufferedElems -= returnedElems;

	// scope lock here to update _currentBuff position
	{
		std::lock_guard <std::mutex> lock(Buf_mutex);
		currentBuff += returnedElems * ELEMS_PER_SAMPLE * shortsPerWord;
	}

	// return number of elements written to buff0
	if (bufferedElems != 0) 
		flags |= SDR_MORE_FRAGMENTS;
	else		//Clear current buffer
		this->releaseReadBuffer(handle);

	return (int)returnedElems;
}

//========================================================================================================
int  Streaming::acquireReadBuffer(size_t &handle,const void **buffs,
	int &flags,	long long &timeNs,const long timeoutUs)
{
	std::unique_lock <std::mutex> lock(Buf_mutex);

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
			//SoapySDR_log(SOAPY_SDR_SSI, "O");
			return SDR_OVERFLOW;
		}
	}

	// wait for a buffer to become available
	if (_buf_count == 0)
	{
		_buf_cond.wait_for(lock, std::chrono::microseconds(timeoutUs));
		if (_buf_count == 0)
		{
			return SDR_TIMEOUT;
		}
	}

	// extract handle and buffer
	handle = _buf_head;
	buffs[0] = (void *)__buffs[handle].data();
	flags = 0;

	_buf_head = (_buf_head + 1) % numBuffers;

	// return number available
	return (int)(__buffs[handle].size() / (ELEMS_PER_SAMPLE * shortsPerWord));
}
//==============================================================
void Streaming::releaseReadBuffer(const size_t handle)
{
	std::lock_guard <std::mutex> lock(Buf_mutex);
	__buffs[handle].clear();
	_buf_count--;
}

//=============================================================
void Streaming::rx_callback(void *IQdata, uint32_t numSamples)
{
	std::lock_guard<std::mutex> lock(Buf_mutex);

	if (_buf_count == numBuffers)
	{
		_overflowEvent = true;
		return;
	}

	int spaceReqd = numSamples * ELEMS_PER_SAMPLE * shortsPerWord;
	
	if ((__buffs[_buf_tail].size() + spaceReqd) >= bufferLength)
	{
		// increment the tail pointer and buffer count
		_buf_tail = (_buf_tail + 1) % numBuffers;
		_buf_count++;

		// notify readStream()
		_buf_cond.notify_one();
	}

	// get current fill buffer
	auto &buff = __buffs[_buf_tail];
	//allocate mem
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
		float *dptr = (float *)buff.data();
		//liquid_float_complex *dptr = (liquid_float_complex *)buff.data();
		dptr += ((buff.size() - spaceReqd) / shortsPerWord);
		float factor = 1.0f / 32768.f;
		int16_t *source = (int16_t *)IQdata;
		liquid_float_complex Y;
//	#pragma omp parallel for 
		for (i = 0; i < numSamples; i++)
		{
#if 1

			Y.imag = *source++ * factor;
			Y.real = *source++ * factor;

			firfilt_crcf_push(firfilt, Y); // push input sample
			firfilt_crcf_execute(firfilt, &Y); // compute output
											   //firfilt_crcf_execute(firfilt, &dptr[i]); // compute output
			*dptr++ = Y.imag;
			*dptr++ = Y.real;
#else
			
			*dptr++ = *source++ * factor;
			*dptr++ = *source++ * factor;
#endif
			
		}

	}
	return;
}
//=============================================================
void Streaming::rx_async_operation(void)
{

	long		readLength;
	int			nCount;
	ULONG      _bufferLength = const_BufferElems * BYTES_PER_SAMPLE;

	char Life[] = { '|','/','-','\\' };
	int p = 0;

	pUSBDev->StopCommand();
	eptInStream->Abort();
	eptInStream->SetXferSize(_bufferLength);

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
		contexts[nCount] = eptInStream->BeginDataXfer(buffers[nCount],
			(long)_bufferLength,
			&inOvLap[nCount]);
	}

	nCount = 0;
	auto t1 = std::chrono::high_resolution_clock::now();
	auto t2 = t1;

	//SoapySDR::log(SOAPY_SDR_SSI, "\n>>>>>> Start >>>>>>>\n");

	/////// Send start  to the device /////
	if(modeStream) //Burst
		pUSBDev->BurstCommand();
	else
		pUSBDev->RawCommand();

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
/*
		t2 = std::chrono::high_resolution_clock::now();
		auto timePeriod = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
		if (timePeriod >= 1000)
		{
			t1 = t2;
			//total number of bytes sent per second
			double dataRate = 1000.0*totalBytesReceived / timePeriod;
			char buf[128];
			std::sprintf(buf, "\r Rx: %.3f MB/s %c", dataRate / 1000000.0, Life[++p & 3]);
			std::cout << buf;
#ifdef NDEBUG
		//	SoapySDR::logf(SOAPY_SDR_SSI, "\r Rx: %.3f MB/s %c", dataRate / 1000000.0, Life[++p & 3]);

#endif
			totalBytesReceived = 0;
			//	rxDataRate_Bps.store((uint32_t)dataRate);
		}
 */
	}

	//<<<<<<<<<<<<<<<<<<<<<<<<
	pUSBDev->StopCommand();
	eptInStream->Abort();
	Sleep(1);

	//SoapySDR::log(SOAPY_SDR_TRACE, "\n");
	for (nCount = 0; nCount< MAX_TRANSFER; nCount++)
	{
		eptInStream->WaitForXfer(&inOvLap[nCount], 100);
		
		eptInStream->FinishDataXfer(buffers[nCount], readLength,
			&inOvLap[nCount],
			contexts[nCount]);
		
		CloseHandle((HANDLE)(inOvLap[nCount].hEvent));
		
		delete[] buffers[nCount];
	}
	
	//SoapySDR::log(SOAPY_SDR_SSI, "\n>>>>>> End >>>>>>>\n");
}
//=============================================================================
