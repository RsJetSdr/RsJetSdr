// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+



#include "SDRThread.h"
#include "RsSDRDefs.h"
#include "RsDemod.h"
//#include <SoapySDR/Logger.h>



SDRThread::SDRThread() : IOThread(), buffers("SDRThreadBuffers") {

	deviceHw = nullptr;
	sampleRate.store(DEFAULT_SAMPLE_RATE);
	frequency.store(DEFAULT_FREQ);


	
	freq_attn.attnStr.mode= ManualALL;
	freq_attn.attnStr.att1 = 31;
	freq_attn.attnStr.att2 = 31;
	freq_attn.attnStr.att3 = 31;
	freq_attn.attnStr.att_low = 31;

	strGain = freq_attn.attnStr;

	freq_attn.freq = (double)frequency;


	offset.store(0);
	ppm.store(0);
	numElems.store(0);


//flag changed
	rate_changed.store(true);
	freq_changed.store(true);
	gain_value_changed.store(true);

	setting_value_changed.store(false);
	offset_changed.store(false);
	antenna_changed.store(false);
	ppm_changed.store(false);
	device_changed.store(false);
//set variable
	numChannels.store(8);
	hasPPM.store(false);
	hasHardwareDC.store(false);
	frequency_lock_init.store(false);
	frequency_locked.store(false);
	lock_freq.store(0);
	iq_swap.store(false);
	buffs[0] = nullptr;
}

SDRThread::~SDRThread() {
	Deinit();
}
//=====================================================================
#if 0
Kwargs SDRThread::combineArgs(Kwargs a, Kwargs b) {
    Kwargs c;
    Kwargs::iterator i;
    for (i = a.begin(); i != a.end(); i++) {
        c[i->first] = i->second;
    }
    for (i = b.begin(); i != b.end(); i++) {
        c[i->first] = i->second;
    }
    return c;
}
#endif
//====================================================================
bool SDRThread::Init()
{
	//#warning Debug On
	//    SoapySDR_setLogLevel(SOAPY_SDR_DEBUG);
	
	if(!deviceHw)
		deviceHw = new Streaming();

	ppm.store(0);
	ppm_changed.store(true);
	offset.store(0); 
	rsGetApp().sdrEnumThreadNotify(/*SDREnumerator::*/SDR_ENUM_MESSAGE, std::string("Initializing device."));

	std::string streamExceptionStr("");

	try {
		stream =(HANDLE *) deviceHw->SetupStream(RX_FORMAT_CF32 /*SOAPY_SDR_RX, "CF32", std::vector<size_t>(), currentStreamArgs*/);
	}
	catch (std::exception e) {
		streamExceptionStr = e.what();
	}

	if (!stream) {
		rsGetApp().sdrThreadNotify(SDRThread::SDR_THREAD_FAILED, std::string("Stream setup failed, stream is null. ") + streamExceptionStr);
		std::cout << "SDRThread::Stream setup failed, stream is null. " << streamExceptionStr << std::endl;
		return false;
	}

	int streamMTU = deviceHw->getStreamMTU(/*stream*/);
	mtuElems.store(streamMTU);
	std::cout << "SDRThread::Device Stream MTU: " << mtuElems.load() << std::endl << std::flush;

	rsGetApp().sdrEnumThreadNotify(/*SDREnumerastor::*/SDR_ENUM_MESSAGE, std::string("Activating stream."));
	deviceHw->setSampleRate(/*SOAPY_SDR_RX, 0,*/ sampleRate.load());

	freq_attn.freq = (double)(frequency - offset.load());
	freq_attn.attnStr = strGain.load();
	deviceHw->setAttnFreq(freq_attn);


	numChannels.store(getOptimalChannelCount(sampleRate.load()));
	numElems.store(getOptimalElementCount(sampleRate.load(), TARGET_DISPLAY_FPS));

	//fallback if  mtuElems was wrong.
	if (!mtuElems.load()) {
		mtuElems.store(numElems.load());
	}
	overflowBuffer.data.resize(mtuElems.load());
	buffs[0] = malloc(mtuElems.load() * 4 * sizeof(float));
	numOverflow = 0;
	updateSettings();

	rsGetApp().sdrThreadNotify(SDRThread::SDR_THREAD_INITIALIZED, std::string("Device Initialized."));
	//rebuild menu now that settings are really been applied.
	rsGetApp().notifyMainUIOfDeviceChange(true);

	deviceHw->activateStream(/*stream*/);

	return true;
}
//=============================================================================
void SDRThread::Deinit() {

	if (deviceHw) {
		deviceHw->deactivateStream();
		delete deviceHw;
		deviceHw = nullptr;
	}

   // deviceHw->CloseStream();
	if (buffs[0]) {
		free(buffs[0]);
		buffs[0] = nullptr;
	}
	

}
//=============================================================================
void SDRThread::assureBufferMinSize(SDRThreadIQData * dataOut, size_t minSize) {
    
    if (dataOut->data.size() < minSize) {
        dataOut->data.resize(minSize);
    }
}
//Called in an infinite loop, read SaopySDR device to build 
// a 'this.numElems' sized batch of samples (SDRThreadIQData) and push it into  iqDataOutQueue.
//this batch of samples is built to represent 1 frame / TARGET_DISPLAY_FPS.
int SDRThread::readStream(SDRThreadIQDataQueuePtr iqDataOutQueue) {
	int flags;
	long long timeNs;

	int n_read = 0;
	int nElems = numElems.load();
	int mtElems = mtuElems.load();

	// Warning: if MTU > numElems, i.e if device MTU is too big w.r.t the sample rate, the TARGET_DISPLAY_FPS cannot
	//be reached and the CubicSDR displays "slows down". 
	//To get back a TARGET_DISPLAY_FPS, the user need to adapt 
	//the SoapySDR Device to use smaller buffer sizes, because  
	// readStream() is suited to device MTU and cannot be really adapted dynamically.
	//TODO: Add in doc the need to reduce SoapySDR device buffer length (if available) to restore higher fps.

	//0. Retreive a new batch 
	SDRThreadIQDataPtr dataOut = buffers.getBuffer();

	//resize to the target size immedialetly, to minimize later reallocs:
	assureBufferMinSize(dataOut.get(), nElems);

	//1.If overflow occured on the previous readStream(), transfer it in dataOut directly. 
	if (numOverflow > 0) {
		int n_overflow = std::min(numOverflow, nElems);

		//safety
		assureBufferMinSize(dataOut.get(), n_overflow);

		::memcpy(&dataOut->data[0], &overflowBuffer.data[0], n_overflow * sizeof(liquid_float_complex));
		n_read = n_overflow;

		//is still > 0 if MTU > nElements (low sample rate w.r.t the MTU !)
		numOverflow -= n_overflow;

		// std::cout << "SDRThread::readStream() 1.1 overflowBuffer not empty, collect the remaining " << n_overflow << " samples in it..." << std::endl;

		if (numOverflow > 0) { // still some left, shift the remaining samples to the begining..
			::memmove(&overflowBuffer.data[0], &overflowBuffer.data[n_overflow], numOverflow * sizeof(liquid_float_complex));

			//    std::cout << "SDRThread::readStream() 1.2 overflowBuffer still not empty, compact the remaining " << numOverflow << " samples in it..." << std::endl;
		}
	} //end if numOverflow > 0

	int readStreamCode = 0;

	//2. attempt readStream() at most nElems, by mtElems-sized chunks, append in dataOut->data directly.
	while (n_read < nElems && !stopping) {

		//Whatever the number of remaining samples needed to reach nElems,  we always try to read a mtElems-size chunk,
		//from which SoapySDR effectively returns n_stream_read.
		int n_stream_read = deviceHw->readStream(/*stream,*/ buffs, mtElems, flags, timeNs);

		readStreamCode = n_stream_read;

		//if the n_stream_read <= 0, bail out from reading. 
		if (n_stream_read == 0) {
			std::cout << "SDRThread::readStream(): 2. SDR read blocking..." << std::endl;
			break;
		}
		else if (n_stream_read < 0) {
			std::cout << "SDRThread::readStream(): 2. SDR read failed with code: " << n_stream_read << std::endl;
			rsGetApp().sdrThreadNotify(SDRThread::SDR_THREAD_FAILED, std::string("SDR read failed"));
			break;
		}

		//sucess read beyond nElems, so with overflow:
		if ((n_read + n_stream_read) > nElems) {

			//n_requested is the exact number to reach nElems.
			int n_requested = nElems - n_read;

			//Copy at most n_requested CF32 into .data liquid_float_complex,
			//starting at n_read position.
			//inspired from SoapyRTLSDR code, this mysterious void** is indeed an array of CF32(real/imag) samples, indeed an array of 
			//float with the following layout [sample 1 real part , sample 1 imag part,  sample 2 real part , sample 2 imag part,sample 3 real part , sample 3 imag part,...etc]
			//Since there is indeed no garantee that sizeof(liquid_float_complex) = 2 * sizeof (float)
			//nor that the Re/Im layout of fields matches the float array order, assign liquid_float_complex field by field.
			float *pp = (float *)buffs[0];

			//safety
			assureBufferMinSize(dataOut.get(), n_read + n_requested);

			if (iq_swap.load()) {
				for (int i = 0; i < n_requested; i++) {
					dataOut->data[n_read + i].imag = pp[2 * i];
					dataOut->data[n_read + i].real = pp[2 * i + 1];
				}
			}
			else {
				for (int i = 0; i < n_requested; i++) {
					dataOut->data[n_read + i].real = pp[2 * i];
					dataOut->data[n_read + i].imag = pp[2 * i + 1];
				}
			}

			//shift of n_requested samples, each one made of 2 floats...
			pp += n_requested * 2;

			//numNewOverflow are in exess, they have to be added in the existing overflowBuffer.
			int numNewOverflow = n_stream_read - n_requested;

			//so push the remainder samples to overflowBuffer:
			if (numNewOverflow > 0) {
				//	std::cout << "SDRThread::readStream(): 2. SoapySDR read make nElems overflow by " << numNewOverflow << " samples..." << std::endl;
			}

			//safety
			assureBufferMinSize(&overflowBuffer, numOverflow + numNewOverflow);

			if (iq_swap.load()) {

				for (int i = 0; i < numNewOverflow; i++) {
					overflowBuffer.data[numOverflow + i].imag = pp[2 * i];
					overflowBuffer.data[numOverflow + i].real = pp[2 * i + 1];
				}
			}
			else {
				for (int i = 0; i < numNewOverflow; i++) {
					overflowBuffer.data[numOverflow + i].real = pp[2 * i];
					overflowBuffer.data[numOverflow + i].imag = pp[2 * i + 1];
				}
			}
			numOverflow += numNewOverflow;

			n_read += n_requested;
		}
		else if (n_stream_read > 0) { // no overflow, read the whole n_stream_read.

			float *pp = (float *)buffs[0];

			//safety
			assureBufferMinSize(dataOut.get(), n_read + n_stream_read);

			if (iq_swap.load()) {
				for (int i = 0; i < n_stream_read; i++) {
					dataOut->data[n_read + i].imag = pp[2 * i];
					dataOut->data[n_read + i].real = pp[2 * i + 1];
				}
			}
			else {
				for (int i = 0; i < n_stream_read; i++) {
					dataOut->data[n_read + i].real = pp[2 * i];
					dataOut->data[n_read + i].imag = pp[2 * i + 1];
				}
			}

			n_read += n_stream_read;
		}
		else {
			break;
		}
	} //end while

	  //3. At that point, dataOut contains nElems (or less if a read has return an error), try to post in queue, else discard.
	if (n_read > 0 && !stopping && !iqDataOutQueue->full()) {

		//clamp result to the actual read size:
		dataOut->data.resize(n_read);
		dataOut->frequency = frequency.load();
		dataOut->sampleRate = sampleRate.load();
		dataOut->dcCorrected = hasHardwareDC.load();
		dataOut->numChannels = numChannels.load();

		// ================ Data Out =========================
		if (!iqDataOutQueue->try_push(dataOut)) {
			//The rest of the system saturates,
			//finally the push didn't suceeded.
			readStreamCode = -32;
			std::cout << "SDRThread::readStream(): 3.2 iqDataOutQueue output queue is full, discard processing of the batch..." << std::endl;
			rsGetApp().sdrThreadNotify(SDRThread::SDR_THREAD_FAILED, std::string("iqDataOutQueue output queue is full"));

			//saturation, let a chance to the other threads to consume the existing samples
			std::this_thread::yield();
		}
	}
	else {
		readStreamCode = -31;
		//std::cout << "SDRThread::readStream(): 3.1 iqDataOutQueue output queue is full, discard processing of the batch..." << std::endl;
		//saturation, let a chance to the other threads to consume the existing samples
		std::this_thread::yield();
	}

	return readStreamCode;
}

void SDRThread::readLoop() {

	SDRThreadIQDataQueuePtr iqDataOutQueue = std::static_pointer_cast<SDRThreadIQDataQueue>(getOutputQueue("IQDataOutput"));

	if (iqDataOutQueue == nullptr) {
		return;
	}

	updateGains();

	while (!stopping.load()) {

		updateSettings();

		readStream(iqDataOutQueue);

	} //End while

	iqDataOutQueue->flush();
}

void SDRThread::updateGains() {
	std::lock_guard < std::mutex > lock(gain_busy);
	freq_attn.attnStr = strGain.load();
	deviceHw->setAttnFreq(freq_attn);
	gain_value_changed.store(false);
}

void SDRThread::updateSettings() {
	bool doUpdate = false;

	if (!stream) {
		return;
	}

	if (antenna_changed.load()) {
		antenna_changed.store(false);
	}

	if (offset_changed.load()) {
		if (!freq_changed.load()) {
			frequency.store(frequency.load());
			freq_changed.store(true);
		}
		offset_changed.store(false);
	}

	if (rate_changed.load()) {

		deviceHw->setSampleRate(/*SOAPY_SDR_RX, 0,*/ sampleRate.load());
		sampleRate.store(deviceHw->getSampleRate(/*SOAPY_SDR_RX, 0*/));
		
	 numChannels.store(getOptimalChannelCount(sampleRate.load()));
		numElems.store(getOptimalElementCount(sampleRate.load(), TARGET_DISPLAY_FPS));
		int streamMTU = deviceHw->getStreamMTU(/*stream*/);
		mtuElems.store(streamMTU);
		//fallback if  mtuElems was wrong
		if (!mtuElems.load()) {
			mtuElems.store(numElems.load());
		}

		overflowBuffer.data.resize(mtuElems.load());
		
		if(buffs[0])
				free(buffs[0]);

		buffs[0] = malloc(mtuElems.load() * 4 * sizeof(float));
		//clear overflow buffer
		numOverflow = 0;

		rate_changed.store(false);
		doUpdate = true;
	}

	if (ppm_changed.load() && hasPPM.load()) {
	//	device->setFrequency(SOAPY_SDR_RX, 0, "CORR", ppm.load());
		ppm_changed.store(false);
	}


	if (gain_value_changed.load() /*&& !agc_mode.load()*/) {
		std::lock_guard < std::mutex > lock(gain_busy);
		freq_attn.attnStr = strGain.load();
	}


	//Change Freq
	if (freq_changed.load()) {
		if (frequency_locked.load() && !frequency_lock_init.load()) {
			freq_attn.freq = (double)lock_freq.load();
			frequency_lock_init.store(true);
		}
		else if (!frequency_locked.load()) {
			freq_attn.freq =(double)(frequency.load() - offset.load());
		}
	}

	if (freq_changed.load() || gain_value_changed.load())
	{
		gain_value_changed.store(false);
		freq_changed.store(false);
		deviceHw->setAttnFreq(freq_attn);
		doUpdate = true;
	}



	//    double devFreq = device->getFrequency(SOAPY_SDR_RX,0);
	//    if (((long long)devFreq + offset.load()) != frequency.load()) {
	//        wxGetApp().setFrequency((long long)devFreq + offset.load());
	//    }


	if (doUpdate) {
		rsGetApp().sdrThreadNotify(SDRThread::SDR_THREAD_INITIALIZED, std::string("Settings updated."));
	}
}
//============================================================================
void SDRThread::run() {
	std::cout << "SDRThread::thread starting................>" << std::endl;

	if (deviceHw == nullptr) {
		std::cout << "SDRThread::Device init()" << std::endl;
		if (!Init()) {
			std::cout << "SDRThread::SDR Thread stream init error." << std::endl;
			return;
		}
	//	std::cout << "SDRThread::Starting readLoop()" << std::endl;
	//	activeDev->setActive(true);
		readLoop();
	//	activeDev->setActive(false);
	//	std::cout << "SDRThread::LeadLoop() ended." << std::endl;
		Deinit();
		std::cout << "SDRThread::Device deinit()" << std::endl;
	}
	else {
		std::cout << "SDRThread::Thread started with null device?" << std::endl;
	}

	std::cout << "SDRThread::thread done...................<" << std::endl;
}

void SDRThread::terminate() {
	IOThread::terminate();

	SDRThreadIQDataQueuePtr iqDataOutQueue = std::static_pointer_cast<SDRThreadIQDataQueue>(getOutputQueue("IQDataOutput"));

	if (iqDataOutQueue != nullptr) {
		iqDataOutQueue->flush();
	}
}

int SDRThread::getOptimalElementCount(long long sampleRate, int fps) {
    int elemCount = (int)floor((double)sampleRate/(double)fps);
    int nch = numChannels.load();
    elemCount = int(ceil((double)elemCount/(double)nch))*nch;
    std::cout << "SDRThread::Calculated optimal " << numChannels.load() << " channel element count of " << elemCount << std::endl;
    return elemCount;
}

int SDRThread::getOptimalChannelCount(long long sampleRate) {
    if (sampleRate <= CHANNELIZER_RATE_MAX) {
        return 1;
    }
    
    int optimal_rate = CHANNELIZER_RATE_MAX;
    int optimal_count = int(ceil(double(sampleRate)/double(optimal_rate)));
    
    if (optimal_count % 2 == 1) {
        optimal_count--;
    }
    
    if (optimal_count < 2) {
        optimal_count = 2;
    }

    return optimal_count;
}



long long SDRThread::getFrequency() {
    return frequency.load();
}

void SDRThread::lockFrequency(long long freq) {
    lock_freq.store(freq);
    frequency_locked.store(true);
    frequency_lock_init.store(false);
    setFrequency(freq);
}

bool SDRThread::isFrequencyLocked() {
    return frequency_locked.load();
}

void SDRThread::unlockFrequency() {
    frequency_locked.store(false);
    frequency_lock_init.store(false);
    freq_changed.store(true);
}

void SDRThread::setOffset(long long ofs) {
    offset.store(ofs);
    offset_changed.store(true);
    std::cout << "SDRThread::Set offset: " << offset.load() << std::endl;
}

long long SDRThread::getOffset() {
    return offset.load();
}

void SDRThread::setAntenna(const std::string& name) {
    antennaName = name;
    antenna_changed.store(true);
}

std::string SDRThread::getAntenna() {
    return antennaName;
}

void SDRThread::setSampleRate(long rate) {
    sampleRate.store(rate);
    rate_changed = true;
    std::cout << "SDRThread::Set sample rate: " << sampleRate.load() << std::endl;
}
long SDRThread::getSampleRate() {
    return sampleRate.load();
}

void SDRThread::setPPM(int ppm) {
    this->ppm.store(ppm);
    ppm_changed.store(true);
     std::cout << "SDRThread::SDRThread::Set PPM: " << this->ppm.load() << std::endl;
}

int SDRThread::getPPM() {
    return ppm.load();
}

void SDRThread::setIQSwap(bool swap) {
	std::cout << "SDRThread::IQSwap: " << swap << std::endl;
	iq_swap.store(swap);
}

bool SDRThread::getIQSwap() {
	  return iq_swap.load();
}

void SDRThread::setFrequency(long long freq) {
	std::lock_guard < std::mutex > lock(gain_busy);
	if (freq < sampleRate.load() / 2) {
		freq = sampleRate.load() / 2;
	}
	frequency.store(freq);
	freq_changed.store(true);
}

void SDRThread::setGain(AttnStr value) {
    std::lock_guard < std::mutex > lock(gain_busy);
    strGain.store(value);
    gain_value_changed.store(true);
}

void SDRThread::setGainFreq(FreqAttnStr value) {
	std::lock_guard < std::mutex > lock(gain_busy);
	strGain.store(value.attnStr);
	frequency.store((long long)value.freq);

	gain_value_changed.store(true);
	freq_changed.store(true);
}

AttnStr SDRThread::getGain() {
    std::lock_guard < std::mutex > lock(gain_busy);
    return strGain;
}

std::vector<long> SDRThread::getSampleRates(int direction, size_t channel)
{
	size_t nbMaxDifferentRates = DEVICE_SAMPLE_RATES_MAX_NB;
	std::vector<long> result;
	std::vector<double> sampleRates = {
		0.16e6,
		0.32e6,
		0.64e6,
		1.28e6,
		2.56e6,
		5.12e6,
		10.24e6,
		20.48e6
	};

	//be paranoid, sort by increasing rates...
	std::sort(sampleRates.begin(), sampleRates.end(), [](double a, double b) -> bool { return a < b; });

	//if sampleRates.size() > nbMaxDifferentRates, decimate this number to only return nbMaxDifferentRates sample 
	//rates values.
	size_t sampleRateSelectionStep = 1;
	if (sampleRates.size() / nbMaxDifferentRates >= 2) {
		sampleRateSelectionStep = sampleRates.size() / nbMaxDifferentRates;
	}
	for (size_t i = 0; sampleRateSelectionStep * i < sampleRates.size(); i++) {
		//convert to longs...
		result.push_back((long)sampleRates[sampleRateSelectionStep * i]);
	}

	//always include the biggest value:
	if ((long)sampleRates.back() > result.back()) {
		result.push_back((long)sampleRates.back());
	}
	return result;
}
//====================================================
long SDRThread::getSampleRateNear(int direction, size_t channel, long sampleRate_in)
{
	std::vector<long> sampleRates = getSampleRates(direction, channel);
	long returnRate = sampleRates[0];
	long sDelta = (long)sampleRate_in - sampleRates[0];
	long minDelta = std::abs(sDelta);
	for (long i : sampleRates) {
		long thisDelta = std::abs(sampleRate_in - i);
		if (thisDelta < minDelta) {
			minDelta = thisDelta;
			returnRate = i;
		}
	}
	return returnRate;
}


