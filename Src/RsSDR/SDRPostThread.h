// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#pragma once

#include "SDRThread.h"

/*
PFBCH polyphase filterbank channelizer
https://pdfs.semanticscholar.org/28f3/8afa6199fb7c950b59924833ec1f34144ecf.pdf
https://github.com/cjcliffe/CubicSDR/issues/150
https://github.com/cjcliffe/CubicSDR/issues/150#issuecomment-148681357
I haven't looked at the code, but looking at your callstack I think I see what you're doing,
so you might consider using the firpfbch_crcf object rather than the firpfbch2_crcf.
The firpfbch2 effectively runs the output at twice the rate of the firpfbch.
So if you have an input sample rate of 16 MHz and are trying to break it into 16 channels,
the firpfbch_crcf object will result in 16 channels each at 1 MHz while the firpfbch2 object
will result in 16 channels each at 2 MHz. I can go into the details as to why later.
*/
enum SDRPostThreadChannelizerType {
						//Example    Sample rate 16 Mhz & 16 channels
    SDRPostPFBCH = 1,   //firpfbch_crcf_analyzer_execute (16 channels each at 1 MHz)
    SDRPostPFBCH2 = 2   //firpfbch2_crcf_execute		 (16 channels each at 2 MHz)
};
//=================================================
class SDRPostThread : public IOThread {
public:

	SDRPostThread();
    ~SDRPostThread();

	void notifyDemodulatorsChanged() { doRefresh.store(true); }
    virtual void run();
    virtual void terminate();
    void resetAllDemodulators();

	void setChannelizerType(SDRPostThreadChannelizerType chType) {
		chanMode.store((int)chType);
	}
	SDRPostThreadChannelizerType getChannelizerType() {
		return (SDRPostThreadChannelizerType)chanMode.load();
	}
protected:
    SDRThreadIQDataQueuePtr iqDataInQueue;
    DemodulatorThreadInputQueuePtr iqDataOutQueue;
    DemodulatorThreadInputQueuePtr iqVisualQueue;
    DemodulatorThreadInputQueuePtr iqActiveDemodVisualQueue;

private:
    // Copy the full samplerate into a new DemodulatorThreadIQDataPtr.
    DemodulatorThreadIQDataPtr getFullSampleRateIqData(SDRThreadIQData *data_in);
    void pushVisualData(DemodulatorThreadIQDataPtr iqDataOut);
    void runSingleCH(SDRThreadIQData *data_in);
    void runDemodChannels(int channelBandwidth);
    void initPFBCH();
    void runPFBCH(SDRThreadIQData *data_in);
    void initPFBCH2();
    void runPFBCH2(SDRThreadIQData *data_in);
    void updateActiveDemodulators();
    void updateChannels();    
    int getChannelAt(long long frequency);

    ReBuffer<DemodulatorThreadIQData> buffers;
    std::vector<liquid_float_complex> dataOut;
    std::vector<long long> chanCenters;
    long long chanBw = 0;
    
    std::vector<DemodulatorInstancePtr> runDemods;
    std::vector<int> demodChannel;
    std::vector<int> demodChannelActive;

    ReBuffer<DemodulatorThreadIQData> visualDataBuffers;
	std::atomic_bool doRefresh;
	std::atomic_int chanMode;

    int numChannels, sampleRate, lastChanMode;
    long long frequency;
    firpfbch_crcf channelizer;
    firpfbch2_crcf channelizer2;
    iirfilt_crcf dcFilter;
    std::vector<liquid_float_complex> dcBuf;
};
