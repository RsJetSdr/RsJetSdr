/*
*/
#include <thread>
#include <exception>

#include "Modem.h"
#include "DemodulatorMgr.h"
#include "SDRThread.h"
#include "SDRPostThread.h"
//======== Analog =====
#include "ModemFM.h"
#include "ModemNBFM.h"
#include "ModemFMStereo.h"
#include "ModemAM.h"
#include "ModemUSB.h"
#include "ModemLSB.h"
#include "ModemDSB.h"
#include "ModemIQ.h"


//===========================================================================
class Demodulator {
public:

	Demodulator();
//	~Demodulator();

	static Demodulator* Demodulator::getInstance(){
	/*	if (instance == nullptr) {
			instance = new Demodulator();
		}
    */
		return instance;
	}

	virtual bool OnInit();
	virtual int OnExit();

	void setFrequency(long long freq);
	long long getFrequency() { return frequency; }

	void setGain(AttnStr value);
	AttnStr getGain() { return attn_str; }

	void setFreqAttn(FreqAttnStr m_FreqAttnStr);

	void lockFrequency(long long freq);
	bool isFrequencyLocked() { return frequency_locked.load(); }
	void unlockFrequency();

	void setSampleRate(long rate_in);
	long long getSampleRate() { return sampleRate; }

	DemodulatorMgr &getDemodMgr() { return demodMgr;}
	void notifyDemodulatorsChanged() { sdrPostThread->notifyDemodulatorsChanged();}


	void setSoloMode(bool solo) { soloMode.store(solo); }
	bool getSoloMode() { return soloMode.load(); }

	void sdrEnumThreadNotify(SDREnumState state, std::string message) { return; }
	void sdrThreadNotify(SDRThread::SDRThreadState state, std::string message) { return; }
	void notifyMainUIOfDeviceChange(bool forceRefreshOfGains = false) { return; }

//	AppConfig *getConfig() { return &config; }
//	std::vector<SDRDeviceInfo *> *getDevices(){return devs;}
//	SDRDeviceInfo * getDevice();

	void startDevice(int Samplerate);
	void stopDevice( int waitMsForTermination);


	void setPPM(int ppm_in) {
		ppm = ppm_in;
		if (sdrThread && !sdrThread->isTerminated()) {
			sdrThread->setPPM(ppm);
		}
	}

	int getPPM() {
		ppm = 0;
		return ppm;
	}
	
	void setOffset(long long ofs) {
		offset = ofs;
		if (sdrThread && !sdrThread->isTerminated()) {
			sdrThread->setOffset(offset);
		}
	}
	long long getOffset() {return offset;}

	DemodulatorThreadOutputQueuePtr getAudioVisualQueue() { return pipeAudioVisualData; }
	DemodulatorThreadInputQueuePtr getIQVisualQueue(){ return pipeIQVisualData; }
	DemodulatorThreadInputQueuePtr getWaterfallVisualQueue() {return pipeWaterfallIQVisualData;};

	SDRPostThread *getSDRPostThread() {	
		return sdrPostThread;
	}
	SDRThread *getSDRThread() { 
		return sdrThread;
	}

private:
	
	static Demodulator *instance;
	//Static
	DemodulatorMgr demodMgr;


//	Kwargs streamArgs;
//	Kwargs settingArgs;
//	AppConfig config;

	//SDRDeviceInfo *stoppedDev;

	
	SDRThread *sdrThread = nullptr;
	std::thread *t_SDR = nullptr;

	SDRPostThread *sdrPostThread = nullptr;
	std::thread *t_PostSDR = nullptr;


	SDRThreadIQDataQueuePtr		   pipeSDRIQData;
	DemodulatorThreadInputQueuePtr pipeIQVisualData;
	DemodulatorThreadInputQueuePtr pipeWaterfallIQVisualData;



	std::atomic_long sampleRate;
	std::atomic_llong frequency;
	std::atomic_bool frequency_locked;
    std::atomic_llong lock_freq;
	
	std::atomic_llong offset;
	std::atomic_int ppm, snap;
	std::atomic_bool agcMode;
	std::atomic_bool shuttingDown;


	DemodulatorThreadOutputQueuePtr pipeAudioVisualData;
	std::atomic_bool soloMode;
	std::atomic_bool sampleRateInitialized;

	AttnStr attn_str;
	std::vector<RtAudio::DeviceInfo> devices;
	std::map<int, RtAudio::DeviceInfo> inputDevices;
	std::map<int, RtAudio::DeviceInfo> outputDevices;
};

//#define DECLARE_APP(appname) extern appname& wxGetApp();
//DECLARE_APP(Demodulator)


extern Demodulator&  rsGetApp();