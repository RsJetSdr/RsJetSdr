// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#pragma once

#include <atomic>
#include <memory>
#include <vector>
#include <chrono>
#include <string>
#include <algorithm>
#include <stddef.h>
#include <iostream>

#include "DemodulatorMgr.h"
#include "SDRDeviceInfo.h"
#include "AppConfig.h"



enum SDREnumState { SDR_ENUM_DEVICES_READY, SDR_ENUM_MESSAGE, SDR_ENUM_TERMINATED, SDR_ENUM_FAILED };

class SDRThreadIQData {
public:
    long long frequency;
    long sampleRate;
    bool dcCorrected;
    int numChannels;
    std::vector<liquid_float_complex> data;

    SDRThreadIQData() :
            frequency(0),
		    sampleRate(DEFAULT_SAMPLE_RATE),
		    dcCorrected(true),
		    numChannels(0){

    }

    SDRThreadIQData(long bandwidth, long long frequency, std::vector<signed char> * /* data */) :
            frequency(frequency), sampleRate(bandwidth) {

    }

    virtual ~SDRThreadIQData() {

    }
};
typedef std::shared_ptr<SDRThreadIQData> SDRThreadIQDataPtr;
typedef ThreadBlockingQueue<SDRThreadIQDataPtr> SDRThreadIQDataQueue;
typedef std::shared_ptr<SDRThreadIQDataQueue> SDRThreadIQDataQueuePtr;

class SDRThread : public IOThread {
private:
    bool Init();
    void Deinit();
    
    //returns the Device readStream return value,
    //i.e if >= 0 the numbre of samples read, else if < 0 an error code.
    int readStream(SDRThreadIQDataQueuePtr iqDataOutQueue);
    void readLoop();

public:
    SDRThread();
    ~SDRThread();

	enum SDRThreadState { SDR_THREAD_MESSAGE, SDR_THREAD_INITIALIZED, SDR_THREAD_FAILED};
    
    virtual void run();
    virtual void terminate();
		
    int getOptimalElementCount(long long sampleRate, int fps);
    int getOptimalChannelCount(long long sampleRate);
    
   
    
    void lockFrequency(long long freq);
    bool isFrequencyLocked();
    void unlockFrequency();
    
    void setOffset(long long ofs);
    long long getOffset();

    void setAntenna(const std::string& name);
    std::string getAntenna();
    
    void setSampleRate(long rate);
    long getSampleRate();

    void setPPM(int ppm);
    int getPPM();
    
    void setIQSwap(bool swap);
    bool getIQSwap();

	void setFrequency(long long freq);
	long long getFrequency();

    void setGain(AttnStr value);
	AttnStr getGain();

	void setGainFreq(FreqAttnStr value);

	Streaming * GetDevice() { return deviceHw;}
	std::vector<long> getSampleRates(int direction, size_t channel);
	long getSampleRateNear(int direction, size_t channel, long sampleRate_in);

protected:
 
    HANDLE *stream;
	Streaming *deviceHw;

	std::mutex setting_busy;
	void updateSettings();
	void updateGains();
    void *buffs[1];

	ReBuffer<SDRThreadIQData> buffers;
    SDRThreadIQData overflowBuffer;
    int numOverflow;
 
    std::atomic_long sampleRate;
	std::atomic_bool  rate_changed;

    std::atomic_llong frequency, offset, lock_freq;
	std::atomic_bool freq_changed, offset_changed, frequency_locked, frequency_lock_init;

	std::atomic_int ppm, numElems, mtuElems, numChannels;
    std::atomic_bool hasPPM, hasHardwareDC;
    std::string antennaName;

	std::atomic_bool 
		antenna_changed,
        ppm_changed,
		device_changed,
		gain_value_changed,
		setting_value_changed,
		iq_swap;

    std::mutex gain_busy;
	std::atomic<AttnStr> strGain;
	FreqAttnStr freq_attn;

    
 private:
	void assureBufferMinSize(SDRThreadIQData * dataOut, size_t minSize);
};
