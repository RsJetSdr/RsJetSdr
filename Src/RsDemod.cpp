#pragma once
#include "RsSDRDefs.h"
#include "RsDemod.h"


Demodulator *Demodulator::instance = nullptr;
Demodulator& rsGetApp() { return *static_cast<Demodulator*>(Demodulator::getInstance()); }

//========================================================================================================
Demodulator::Demodulator() : 
	frequency(100000000), 
	offset(0), 
	ppm(0), 
	snap(1), 
	sampleRate(DEFAULT_SAMPLE_RATE) 

{
	sampleRateInitialized.store(false);
	agcMode.store(true);
	soloMode.store(false);
	shuttingDown.store(false);
//	stoppedDev = nullptr;

	attn_str.mode = ManualALL;
	attn_str.att1 = 0;
	attn_str.att2 = 0;
	attn_str.att3 = 0;
	attn_str.att_low = 0;

	instance = this;


/*
	fdlgTarget = FrequencyDialog::FDIALOG_TARGET_DEFAULT;
	//set OpenGL configuration:
	m_glContextAttributes = new wxGLContextAttrs();
	wxGLContextAttrs glSettings;
	glSettings.PlatformDefaults().EndList();
	*m_glContextAttributes = glSettings;
*/


/*
	    AppConfig *cfg = wxGetApp().getConfig();
		DeviceConfig *devConfig = cfg->getDevice(dev->getDeviceId());
		devConfig->setSettings(settingArgs);
		devConfig->setStreamOpts(streamArgs);
		wxGetApp().setDeviceArgs(settingArgs);
		wxGetApp().setStreamArgs(streamArgs);
		wxGetApp().setDevice(dev,0);
*/
}
//===========================================================
void Demodulator::setSampleRate(long rate_in) {
	sampleRate = rate_in;

	if (sdrThread && !sdrThread->isTerminated()) {
		sdrThread->setSampleRate(sampleRate);
	}
	setFrequency(frequency);
}
//===========================================================
void Demodulator::setGain(AttnStr value) {
	attn_str = value;
  if (sdrThread)
	   sdrThread->setGain(value);
}
//===========================================================
void Demodulator::setFrequency(long long freq) {
	if (freq < sampleRate / 2){
		freq = sampleRate / 2;
	}
	frequency = freq;
	if (sdrThread)
	      sdrThread->setFrequency(freq);
}

void Demodulator::setFreqAttn(FreqAttnStr m_FreqAttnStr) {
	long long freq = m_FreqAttnStr.freq;
	if (freq < sampleRate / 2) {
		freq = sampleRate / 2;
	}
	frequency = freq;
	attn_str = m_FreqAttnStr.attnStr;
	if (sdrThread)
		sdrThread->setGainFreq(m_FreqAttnStr);


}

//===========================================================
void Demodulator::lockFrequency(long long freq) {
	frequency_locked.store(true);
	lock_freq.store(freq);
	if (sdrThread && !sdrThread->isTerminated()) {
		sdrThread->lockFrequency(freq);
	}
}
//===========================================================
void Demodulator::unlockFrequency() {
	frequency_locked.store(false);

	if (sdrThread && !sdrThread->isTerminated()) {
		sdrThread->unlockFrequency();
	}

}
//====================================================================
void Demodulator::stopDevice(int waitMsForTermination) {

	//Firt we must stop the threads
	sdrThread->terminate();
	sdrThread->isTerminated(waitMsForTermination);

	if (t_SDR) {
		t_SDR->join();
		delete t_SDR;
		t_SDR = nullptr;
	}
//	stoppedDev = nullptr;
}
//===================  Start sdrTHread    ======================
void Demodulator::startDevice(int SampleRate) {

	sdrThread->terminate();
	sdrThread->isTerminated(1000);

	if (t_SDR) {
		t_SDR->join();
		delete t_SDR;
		t_SDR = nullptr;
	}

	{
		if(SampleRate){
			sampleRate = sdrThread->getSampleRateNear(0, 0, SampleRate);
			sampleRateInitialized.store(true);
		}

		if (!sampleRateInitialized.load()) {
			sampleRate = sdrThread->getSampleRateNear(0, 0, DEFAULT_SAMPLE_RATE);
			sampleRateInitialized.store(true);
		}
		else {
			sampleRate = sdrThread->getSampleRateNear(0, 0, sampleRate);
		}


		setFrequency(frequency);
		setGain(attn_str);
		setSampleRate(sampleRate);

		setPPM(0);
		setOffset(0);

		t_SDR = new std::thread(&SDRThread::threadMain, sdrThread);
	}
//	stoppedDev = nullptr;
}
//===================================================================
bool Demodulator::OnInit()
{

	Modem::addModemFactory(ModemFM::factory, "FM", 200000);
	Modem::addModemFactory(ModemNBFM::factory, "NBFM", 12500);
	Modem::addModemFactory(ModemFMStereo::factory, "FMS", 200000);
	Modem::addModemFactory(ModemAM::factory, "AM", 6000);
	Modem::addModemFactory(ModemLSB::factory, "LSB", 5400);
	Modem::addModemFactory(ModemUSB::factory, "USB", 5400);
	Modem::addModemFactory(ModemDSB::factory, "DSB", 5400);
	Modem::addModemFactory(ModemIQ::factory, "I/Q", 32000);

#ifdef ENABLE_DIGITAL_LAB
	Modem::addModemFactory(ModemAPSK::factory, "APSK", 200000);
	Modem::addModemFactory(ModemASK::factory, "ASK", 200000);
	Modem::addModemFactory(ModemBPSK::factory, "BPSK", 200000);
	Modem::addModemFactory(ModemDPSK::factory, "DPSK", 200000);
	Modem::addModemFactory(ModemFSK::factory, "FSK", 19200);
	Modem::addModemFactory(ModemGMSK::factory, "GMSK", 19200);
	Modem::addModemFactory(ModemOOK::factory, "OOK", 200000);
	Modem::addModemFactory(ModemPSK::factory, "PSK", 200000);
	Modem::addModemFactory(ModemQAM::factory, "QAM", 200000);
	Modem::addModemFactory(ModemQPSK::factory, "QPSK", 200000);
	Modem::addModemFactory(ModemSQAM::factory, "SQAM", 200000);
	Modem::addModemFactory(ModemST::factory, "ST", 200000);
#endif

// Visual Data
//	spectrumVisualThread = new SpectrumVisualDataThread();

//Create PIPE
	pipeIQVisualData = std::make_shared<DemodulatorThreadInputQueue>();
	pipeIQVisualData->set_max_num_items(1);
//Create PIPE

	pipeWaterfallIQVisualData = std::make_shared<DemodulatorThreadInputQueue>();
	pipeWaterfallIQVisualData->set_max_num_items(128);

//	getSpectrumProcessor()->setInput(pipeIQVisualData);
//	getSpectrumProcessor()->setHideDC(true);

	// I/Q Data
	//Create Pipe
	pipeSDRIQData = std::make_shared<SDRThreadIQDataQueue>();
	pipeSDRIQData->set_max_num_items(100);

	//Create SDRThread & DRPostThread, Not Start Thread
	sdrThread = new SDRThread();		
	sdrPostThread = new SDRPostThread(); //Not Start


	//Create Output pipe sdrThread
		sdrThread->setOutputQueue("IQDataOutput", pipeSDRIQData);
 
 //Create Input pipe sdrPostThread	
 	sdrPostThread->setInputQueue("IQDataInput", pipeSDRIQData);
//Create Output pipe sdrPostThread
	sdrPostThread->setOutputQueue("IQVisualDataOutput", pipeIQVisualData);
	sdrPostThread->setOutputQueue("IQDataOutput", pipeWaterfallIQVisualData);


#if ENABLE_VIEW_SCOPE
//Create PIPE
	pipeAudioVisualData = std::make_shared<DemodulatorThreadOutputQueue>();
	pipeAudioVisualData->set_max_num_items(1);
	scopeProcessor.setInput(pipeAudioVisualData);
#else
	pipeAudioVisualData = nullptr;
#endif

	//Set Audio Out
	std::vector<RtAudio::DeviceInfo>::iterator devices_i;
	std::map<int, RtAudio::DeviceInfo>::iterator mdevices_i;
	AudioThread::enumerateDevices(devices);
	int i = 0;

	for (devices_i = devices.begin(); devices_i != devices.end(); devices_i++) {
		if (devices_i->inputChannels) {
			inputDevices[i] = *devices_i;
		}
		if (devices_i->outputChannels) {
			outputDevices[i] = *devices_i;
		}
		i++;
	}
	
	rsGetApp().getDemodMgr().setOutputDevices(outputDevices);


#define NUM_RATES_DEFAULT 4
	unsigned int desired_rates[NUM_RATES_DEFAULT] = {32000, 48000, 44100, 96000 /*, 192000*/ };

	for (mdevices_i = outputDevices.begin();
		mdevices_i != outputDevices.end();
		mdevices_i++) 
	{
		unsigned int desired_rate = 0;
		unsigned int desired_rank = NUM_RATES_DEFAULT + 1;

		for (std::vector<unsigned int>::iterator srate = mdevices_i->second.sampleRates.begin(); srate != mdevices_i->second.sampleRates.end();
			srate++) {
			for (unsigned int i = 0; i < NUM_RATES_DEFAULT; i++) {
				if (desired_rates[i] == (*srate)) {
					if (desired_rank > i) {
						desired_rank = i;
						desired_rate = (*srate);
					}
				}
			}
		}

		if (desired_rank > NUM_RATES_DEFAULT) {
			desired_rate = mdevices_i->second.sampleRates.back();
		}
		AudioThread::deviceSampleRate[mdevices_i->first] = desired_rate;
	}//for all device

	//Start SDRPostThread last.
	t_PostSDR = new std::thread(&SDRPostThread::threadMain, sdrPostThread);

	
//	startDevice(5120000);

	return false;
}
//===============================================
int Demodulator::OnExit() {


	if (!sdrThread) return -1;
	
	shuttingDown.store(true);

	bool terminationSequenceOK = true;

	//The thread feeding them all should be terminated first, so: 
	std::cout << "Terminating SDR thread.." << std::endl << std::flush;
	sdrThread->terminate();
	int wait = t_SDR ? 3000 : 0;
	terminationSequenceOK = terminationSequenceOK && sdrThread->isTerminated(wait);

	//in case termination sequence goes wrong, kill App brutally now because it can get stuck. 
	if (!terminationSequenceOK) {
		//no trace here because it could occur if the device is not started.  
		::exit(11);
	}

	std::cout << "Terminating SDR post-processing thread.." << std::endl << std::flush;
	sdrPostThread->terminate();

	//Wait for termination for sdrPostThread second:: since it is doing
	//mostly blocking push() to the other threads, they must stay alive
	//so that sdrPostThread can complete a processing loop and die.
	wait = t_PostSDR ? 3000 : 0;
	terminationSequenceOK = terminationSequenceOK && sdrPostThread->isTerminated(wait);

	//in case termination sequence goes wrong, kill App brutally now because it can get stuck. 
	if (!terminationSequenceOK) {
		std::cout << "Cannot terminate application properly, calling exit() now." << std::endl << std::flush;
		::exit(12);
	}

	std::cout << "Terminating All Demodulators.." << std::endl << std::flush;
	demodMgr.terminateAll();

	std::cout << "Terminating Visual Processor threads.." << std::endl << std::flush;
	//spectrumVisualThread->terminate();
	//if (demodVisualThread) {demodVisualThread->terminate();	}

	//Wait nicely
//	terminationSequenceOK = terminationSequenceOK && spectrumVisualThread->isTerminated(1000);
//	if (demodVisualThread) {terminationSequenceOK = terminationSequenceOK && demodVisualThread->isTerminated(1000);}

	//in case termination sequence goes wrong, kill App brutally because it can get stuck. 
	if (!terminationSequenceOK) {
		std::cout << "Cannot terminate application properly, calling exit() now." << std::endl << std::flush;
		::exit(13);
	}

	//Then join the thread themselves:
	if (t_SDR) {
		t_SDR->join();
	}
	if (t_PostSDR) {
		t_PostSDR->join();
	}

//	if (t_DemodVisual) {  t_DemodVisual->join(); }
//	t_SpectrumVisual->join();

	//Now only we can delete:
	delete t_SDR; 	
	t_SDR = nullptr;
	delete sdrThread; 
	sdrThread = nullptr;

	delete t_PostSDR; 
	t_PostSDR = nullptr;
	delete sdrPostThread; 
	sdrPostThread = nullptr;
     

    //delete t_SpectrumVisual; t_SpectrumVisual = nullptr;
	//delete spectrumVisualThread; spectrumVisualThread = nullptr;
	//delete t_DemodVisual; t_DemodVisual = nullptr;
	//delete demodVisualThread; demodVisualThread = nullptr;
    
	
	AudioThread::deviceCleanup();
	std::cout << "Application termination complete." << std::endl << std::flush;
	delete this;
	instance = nullptr;
	return 0;     //wxApp::OnExit();

}
