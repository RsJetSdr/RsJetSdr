#pragma once
//cross platform usleep()
#ifdef _MSC_VER
#include <windows.h>
#define usleep(t) Sleep((t)/1000)
#else
#include <unistd.h>
#endif

#include <mutex>
#include <cstdio>
#include <algorithm>	//min
#include <climits>		//SHRT_MAX
#include <cstring>		// memcpy
#include <atomic>
#include <condition_variable>

#include <SoapySDR/Device.hpp>
#include <SoapySDR/Registry.hpp>
#include <SoapySDR/Logger.hpp>
#include <SoapySDR/Time.hpp>
#include "liquid/liquid.h"


#define RSJET_FREQUENCY_MIN      9e3
#define RSJET_FREQUENCY_MAX    3000e6
#define GAIN_VGA1              15
#define GAIN_VGA2              15
#define GAIN_VGA3              15
#define ATTN_MIN			   0
#define ATTN_MAX			   -31
#define ATTN_STEP			   1

#define DEFAULT_BUFFER_LENGTH     (8192)
#define DEFAULT_NUM_BUFFERS       (8)
#define DEFAULT_ELEMS_PER_SAMPLE  (2)
#define BYTES_PER_SAMPLE		   4



#define STRINGIFY_(x) #x
#define STRINGIFY(x) STRINGIFY_(x)

#define	mDEBUG SOAPY_SDR_INFO


typedef enum rsjetRXFormat {
	RX_FORMAT_CF32,
	RX_FORMAT_CS16
} rsjetRXFormat;



// D:\PotHos\SoapyRsJet\Build\RelWithDebInfo\
//$(TargetPath)
class RsJet : public SoapySDR::Device
{
public:
	
	RsJet(const SoapySDR::Kwargs & args);
	
	~RsJet(void);
	
	/*******************************************************************
	* Identification API
	******************************************************************/
	std::string getDriverKey(void) const;

	std::string getHardwareKey(void) const;
	
	/*******************************************************************
	* Frequency API
	******************************************************************/
	void setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args = SoapySDR::Kwargs());
	
	double getFrequency(const int direction, const size_t channel, const std::string &name) const;
	
	std::vector<std::string> listFrequencies(const int direction, const size_t channel) const;
	
	SoapySDR::RangeList getFrequencyRange(const int direction, const size_t channel, const std::string &name) const;

	/*******************************************************************
	* Sample Rate API
	******************************************************************/

	void setSampleRate(const int direction, const size_t channel, const double rate);
	double getSampleRate(const int direction, const size_t channel) const;
	std::vector<double> listSampleRates(const int direction, const size_t channel) const;

	/*******************************************************************
	* Gain API
	******************************************************************/
	std::vector<std::string> listGains(const int direction, const size_t) const;
	SoapySDR::Range getGainRange(const int direction, const size_t, const std::string &name) const;
//	void  setGain(const int direction, const size_t channel, const double value);
	void  setGain(const int direction, const size_t channel, const std::string &name, const double value);
	double getGain(const int direction, const size_t, const std::string &name) const;

	/*******************************************************************
	* Frontend corrections API
	******************************************************************/
	bool hasDCOffset(const int direction, const size_t channel) const;

	/*******************************************************************
	* Antenna API
	******************************************************************/

	std::vector<std::string> listAntennas(const int direction, const size_t channel) const;
	std::string getAntenna(const int direction, const size_t channel) const;
	void RsJet::setAntenna(const int, const size_t, const std::string &);

	/*******************************************************************
	* Channels API
	******************************************************************/
	size_t getNumChannels(const int) const;
	bool getFullDuplex(const int, const size_t) const;

	/*******************************************************************
	* Stream API
	******************************************************************/
	SoapySDR::ArgInfoList RsJet::getStreamArgsInfo(const int, const size_t) const;

	std::vector<std::string> getStreamFormats(const int direction, const size_t channel) const;

	std::string getNativeStreamFormat(const int direction, const size_t channel, double &fullScale) const;

	SoapySDR::Stream *setupStream(
		const int direction,
		const std::string &format,
		const std::vector<size_t> &channels = std::vector<size_t>(),
		const SoapySDR::Kwargs &args = SoapySDR::Kwargs());

	SoapySDR::Stream *_setupStream(const int direction,
		const std::string &format,
		const std::vector<size_t> &channels,
		const SoapySDR::Kwargs &args);



	void closeStream(SoapySDR::Stream *stream);

	int activateStream(
		SoapySDR::Stream *stream,
		const int flags = 0,
		const long long timeNs = 0,
		const size_t numElems = 0);

	int deactivateStream(SoapySDR::Stream *stream,
		const int flags = 0,const long long timeNs = 0);
	
	int readStream(SoapySDR::Stream *stream,void * const *buffs,const size_t numElems,
		int &flags,long long &timeNs,const long timeoutUs = 100000);
	

	/*******************************************************************
	* Direct buffer access API
	******************************************************************/
	size_t getNumDirectAccessBuffers(SoapySDR::Stream *stream);

	int getDirectAccessBufferAddrs(SoapySDR::Stream *stream, const size_t handle, void **buffs);

	void releaseReadBuffer(SoapySDR::Stream *stream, const size_t handle);
	
	int acquireReadBuffer(SoapySDR::Stream *stream,	size_t &handle,	const void **buffs,	int &flags,
										long long &timeNs,const long timeoutUs = 100000);
	
	size_t getStreamMTU(SoapySDR::Stream *stream) const;

	
	
	void  SetFreqAttn(double freq);
private:
	mutable std::mutex _general_state_mutex;
	std::atomic_bool streamActive;


	RsFx3_USB  *pUSBDev;


	double	centerFrequency;
	double	sampleRate;
	rsjetRXFormat rxFormat;
	uint8_t Attn[3];


	//async api usage
	//bool stop_thread;

	std::thread _rx_async_thread;
	void rx_async_operation(void);
	void rx_callback(void *IQdata, uint32_t len);
	CCyUSBEndPoint *eptInStream;

	


	
	
	std::mutex	_buf_mutex;
	std::condition_variable _buf_cond;
	std::atomic<size_t>	_buf_count;
	size_t	_buf_head;
	size_t	_buf_tail;
	std::vector<std::vector<short> > __buffs;

	size_t bufferLength;
	std::atomic_size_t bufferedElems; 
	
	size_t _currentHandle;

	uint8_t *_currentBuff;

	std::atomic<bool> _overflowEvent;
	std::atomic_bool resetBuffer; 

	int16_t *_rxConvBuff;
	


	const size_t numBuffers = DEFAULT_NUM_BUFFERS;
	const unsigned int bufferElems = DEFAULT_BUFFER_LENGTH;
	const int elementsPerSample = DEFAULT_ELEMS_PER_SAMPLE;

	std::atomic_uint shortsPerWord;


	char name[255];
	char model[255];

#define h_len 21
	float hFir[h_len];
	float dF;
	firfilt_crcf firfilt;


};