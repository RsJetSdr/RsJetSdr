#include "Rs_FX3_USB.h"
#include "SoapyRsJet.hpp"

/***********************************************************************
* Device interface
**********************************************************************/

//=======================================
RsJet::RsJet(const SoapySDR::Kwargs & args) :
	sampleRate(160e3),
	centerFrequency(140000000),
	resetBuffer(true),
	numBuffers(DEFAULT_NUM_BUFFERS),
	streamActive(false),
	rxFormat(RX_FORMAT_CS16),
	pUSBDev(NULL)
{
	shortsPerWord = 1;
	bufferLength = bufferElems * elementsPerSample * shortsPerWord;

	SoapySDR::logf(mDEBUG, "RsJet_open");
	pUSBDev = new RsFx3_USB(NULL);
	BOOL ret = pUSBDev->OpenDevice(model);
	if (!ret)
	{
		SoapySDR::logf(SOAPY_SDR_ERROR, "RsJet_OpenDevice() returned %s", pUSBDev->lastError());
		throw std::runtime_error("RsJet_OpenDevice() failed ");
	}
	/*
	pUSBDev->OutCmdReg(MB_RX_CONTRL0, pUSBDev->MB_CONTROL0_BR);
	pUSBDev->OutCmdReg(MB_RAW_CTRL, MODE_STOP | 4096);
	pUSBDev->OutCmdReg(MB_ADC_REG, pUSBDev->REG_ADC);
	pUSBDev->OutCmdReg(MB_RX_MUX, FLAG_DSP_RX_MUX_REAL_MODE);
	*/
	Attn[0]=0;
	Attn[1]=0;
	Attn[2]=0;
	SetFreqAttn(centerFrequency);
	
	dF = estimate_req_filter_df(40.0f, h_len);

	liquid_firdes_kaiser(h_len, 0.41f, 40.0f, 0.0f, hFir);
	firfilt = firfilt_crcf_create(hFir, h_len);
	firfilt_crcf_set_scale(firfilt, 2.0f*0.43f);
	
}
//=====================================
RsJet::~RsJet()
{
	SoapySDR::logf(mDEBUG, "RsJet_close()");
	if (pUSBDev != NULL) {
	
		if (_rx_async_thread.joinable()) {
			streamActive = false;
			_rx_async_thread.join();
		}
		delete pUSBDev;


	}

}

/*******************************************************************
* Identification API
******************************************************************/
std::string RsJet::getDriverKey(void) const {
		return "rsjet";
}
/*
template< typename TCharType, typename TCharTraits, typename TStringAllocator >
inline void Convert(const std::basic_string<TCharType, TCharTraits, TStringAllocator>& source_string, std::string& dest_string)
{
	std::wstring_convert<std::codecvt_utf8_utf16<TCharType>, TCharType> converter;
	dest_string = converter.to_bytes(source_string);
}
*/


std::string RsJet::getHardwareKey(void) const {

	char pMBBuffer[256];
	const wchar_t* val = pUSBDev->getSerialNumber();
	wcstombs(pMBBuffer, pUSBDev->getSerialNumber(),256);
	return std::string(pMBBuffer);


}

//Kwargs getHardwareInfo(void) const;


/*******************************************************************
* Frequency API
******************************************************************/
void RsJet::setFrequency(const int direction, const size_t channel, const std::string &name, const double frequency, const SoapySDR::Kwargs &args) {
	if (name == "RF")
	{
		SoapySDR::logf(mDEBUG, "setFrequency");
		resetBuffer = true;
		SetFreqAttn(frequency);
	}
}
//========================================================
double RsJet::getFrequency(const int direction, const size_t channel, const std::string &name) const
{
	if (name == "BB") return 0; //for compatibility
	   if(name != "RF") 
		throw std::runtime_error("setFrequency(" + name + ") unknown name");
	return centerFrequency;
}

SoapySDR::RangeList RsJet::getFrequencyRange(const int direction, const size_t channel, const std::string &name) const
{
	if (name == "BB") return SoapySDR::RangeList(1, SoapySDR::Range(0.0, 0.0)); //for compatibility
	if (name != "RF") 
		throw std::runtime_error("getFrequencyRange(" + name + ") unknown name");
	return (SoapySDR::RangeList(1, SoapySDR::Range(RSJET_FREQUENCY_MIN, RSJET_FREQUENCY_MAX)));

}
//========================================================
std::vector<std::string> RsJet::listFrequencies(const int direction, const size_t channel) const
{
	std::vector<std::string> names;
	names.push_back("RF");
	return(names);
}
/*******************************************************************
* Sample Rate API
******************************************************************/
void RsJet::setSampleRate(const int direction, const size_t channel, const double rate)
{
	sampleRate = rate;
	resetBuffer = true;
	SoapySDR_logf(mDEBUG, "Setting sample rate: %f", float(rate));
	pUSBDev->SetSampleRate(sampleRate);
}
//================================================================================================
double RsJet::getSampleRate(const int direction, const size_t channel) const {
	SoapySDR::logf(mDEBUG, "getSampleRate: %f",float(sampleRate));
	return sampleRate;
	
}

std::vector<double> RsJet::listSampleRates(const int direction, const size_t channel) const
{
	std::vector<double> options;
	SoapySDR::logf(mDEBUG, "flistSampleRates....");

	  options.push_back(16e4);
	  options.push_back(32e4);
	  options.push_back(64e4);
	 options.push_back(128e4);
	 options.push_back(256e4);
	 options.push_back(512e4);
	options.push_back(1024e4);
	if (pUSBDev->bSuperSpeed)
	options.push_back(2048e4);
	return(options);
}

/*******************************************************************
* Antenna API
******************************************************************/

std::vector<std::string>RsJet::listAntennas(const int direction, const size_t channel) const {

	std::vector<std::string> options;
	options.push_back("RXH");
	options.push_back("RXL");
	return(options);
}
void RsJet::setAntenna(const int, const size_t, const std::string &)
{
	return; //nothing to set, ignore it
}


std::string RsJet::getAntenna(const int direction, const size_t channel) const
{
	if (direction == SOAPY_SDR_RX) {
		if(centerFrequency < 30) return "RXL";
		else return "RXH";
	}
		
	return SoapySDR::Device::getAntenna(direction, channel);
}
/*******************************************************************
* Gain API
******************************************************************/
std::vector<std::string> RsJet::listGains(const int direction, const size_t) const
{
	std::vector<std::string> options;
	options.push_back("ATT");
	options.push_back("RF");
	options.push_back("IF");
	return options;
}

SoapySDR::Range RsJet::getGainRange(const int direction, const size_t, const std::string &name) const
{
	if (direction == SOAPY_SDR_RX and name == "ATT")return SoapySDR::Range(ATTN_MAX+GAIN_VGA1,GAIN_VGA1);
	if (direction == SOAPY_SDR_RX and name == "RF") return SoapySDR::Range(ATTN_MAX+GAIN_VGA2,GAIN_VGA2);
	if (direction == SOAPY_SDR_RX and name == "IF") return SoapySDR::Range(ATTN_MAX+GAIN_VGA3,GAIN_VGA3);
	else 
		throw std::runtime_error("getGainRange(" + name + ") -- unknown name");
}
/*
void  RsJet::setGain(const int direction, const size_t channel, const double value)
{
	//set the overall gain by distributing it across available gain elements
	//OR delete this function to use SoapySDR's default gain distribution algorithm...
	SoapySDR::Device::setGain(direction, channel, value);
	SoapySDR_logf(mDEBUG, "Setting Gain default: gain = %f", float(value));
}
*/
void RsJet::setGain(const int direction, const size_t channel, const std::string &name, const double value)
{
	int ret = 0;
	uint32_t attn = (uint32_t)(abs(value - GAIN_VGA1));
	if (attn > 31) 	attn = 31;
	
	if (direction == SOAPY_SDR_RX and name == "ATT") Attn[0] = attn;
	else if (direction == SOAPY_SDR_RX and name == "RF") Attn[1] = attn;
	else if (direction == SOAPY_SDR_RX and name == "IF") Attn[2] = attn;
	else 
		throw std::runtime_error("setGain(" + name + ") -- unknown name");

	SetFreqAttn(centerFrequency); //default

}

double RsJet::getGain(const int direction, const size_t, const std::string &name) const
{
	int ret = 0;
	double gain = 0;
	if (direction == SOAPY_SDR_RX and name == "ATT") ret = Attn[0];
	else if (direction == SOAPY_SDR_RX and name == "RF") ret = Attn[1];
	else if (direction == SOAPY_SDR_RX and name == "IF") ret = Attn[2];
	else 
		throw std::runtime_error("getGain(" + name + ") -- unknown name");
	gain = GAIN_VGA1 - ret;
//	SoapySDR_logf(mDEBUG, "Get Gain: %s %f %d", name, float(gain),ret);
	return gain;
}

/*******************************************************************
* Channels API
******************************************************************/
size_t RsJet::getNumChannels(const int dir) const
{
	return (dir == SOAPY_SDR_RX) ? 1 : 0;
}
//======================================================
bool RsJet::getFullDuplex(const int, const size_t) const
{
	return false;
}
/*******************************************************************
* Frontend corrections API
******************************************************************/
bool RsJet::hasDCOffset(const int direction, const size_t channel) const
{
	SoapySDR_log(mDEBUG, "hasDCOffset false");
	return false;
}
//=================================================================
void  RsJet::SetFreqAttn( double freq)
{
	

	if (freq > 22000000.0) {

		uint32_t mod = uint32_t(freq) % 1000000;
		//Convert to MHz
		double  FreqDDC = ((freq / 1000000) < 1444) ? 
			 CENTER_REC_140MHZ + mod :
			 CENTER_FFT_140MHZ - mod; //

		pUSBDev->SetFreqDDC(FreqDDC);

		uint16_t adf_freq = (uint16_t)(freq/1000000);
		{
			uint8_t cmdbuf[16];
			cmdbuf[0] = 0x05;
			cmdbuf[1] = 10;   // Size 10 * 8 uS = 80 uS
			cmdbuf[2] = 0x20;   // command

			adf_freq =(uint16_t)( freq / 1000000);  //Точность синтезатора 1 MHz

			cmdbuf[3] = adf_freq & 0xFF;;
			cmdbuf[4] = (adf_freq >> 8) & 0xFF;
			/*
			modeAttn
			0: Set All Struct
			1: Set Attn1 Struct, Attn2 & Attn3 predefined (eeprom)
			2: Set All  predefined (eeprom)
			*/
			cmdbuf[5] = 0; //External
			cmdbuf[6] = (63 - (Attn[0] * 2));
			cmdbuf[7] = (63 - (Attn[1] * 2));
			cmdbuf[8] = (63 - (Attn[2] * 2));
			DWORD Snt = 0;
			pUSBDev->OutSerial(cmdbuf,&Snt,4); //Ser Freq & Attns

		}
		pUSBDev->SelectBand(true);
		SoapySDR_logf(mDEBUG, "Setting center freq: %f Att %d:%d:%d", float(freq), Attn[0], Attn[1], Attn[2]);
		}
	else {
		pUSBDev->SetFreqDDC(freq); //Ser Freq
		pUSBDev->SelectBand(false);
		pUSBDev->OutCmdReg(MB_ATTN_REG, (63 - (Attn[0] * 2) & 0x3F));
		SoapySDR_logf(mDEBUG, "Setting center freq: %f Att %d", float(freq), Attn[0]);

	}
	   centerFrequency = freq;
}