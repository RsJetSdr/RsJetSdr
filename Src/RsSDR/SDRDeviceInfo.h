// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#pragma once

#include <string>
#include <vector>
#include <atomic>
#include "Streaming.h"


typedef struct _SDRManualDef {
    std::string factory;
    std::string params;
} SDRManualDef;


/*!
 * A simple min/max numeric range pair
 */


class Range {
private:
	double _min, _max, _step;
public:
	//! Create an empty range (0.0, 0.0)
	Range(void) : _min(0.0),_max(0.0),_step(0.0){}
	//! Create a min/max range
	Range(const double minimum, const double maximum, const double step =0.0)
	{
		_min = minimum;
		_max = maximum;
		_step = step;
	}
	//! Get the range minimum
	double minimum(void) const {return _min;}
	//! Get the range maximum
	double maximum(void) const { return _max; }
	//! Get the range step size
	double step(void) const { return _step; }
};


typedef std::map<std::string, Range> SDRRangeMap;
//! Typedef for a dictionary of key-value string arguments
typedef std::map<std::string, std::string> Kwargs;
/*
class SDRDeviceInfo 
{
public:
//=======================================================================
	SDRDeviceInfo():name(""),serial(""),available(false),remote(false),manual(false)
		, RsJetDevice(nullptr)
	{
		active.store(false);
	};
 //=====================================================================
	~SDRDeviceInfo() {
		if (RsJetDevice != nullptr) {
			delete RsJetDevice;
			RsJetDevice = nullptr;

		}
	}
 //=====================================================================   
	std::string getDeviceId() {
		std::string deviceId;
		deviceId.append(getName());
		return deviceId;
	}
    
	int getIndex() const { return index; }
	void setIndex(const int index) { this->index = index; }
    
	bool isAvailable() const { return available; }
    void setAvailable(bool available) { this->available = available; }

    bool isActive() const { return active.load(); }
    void setActive(bool active) { this->active.store(active); }


    const std::string& getName() const { return name; }
    void setName(const std::string& name) { this->name = name;}
    
    const std::string& getSerial() const { return serial; }
    void setSerial(const std::string& serial) { this->serial = serial;}
    
    const std::string& getTuner() const  { return tuner; }
    void setTuner(const std::string& tuner) { this->tuner = tuner; }
    
    const std::string& getManufacturer() const { return manufacturer; }
    void setManufacturer(const std::string& manufacturer) { this->manufacturer = manufacturer; }
    
    const std::string& getProduct() const { return product; }
    void setProduct(const std::string& product) { this->product = product; }

    const std::string& getDriver() const { return driver; }
    void setDriver(const std::string& driver) { this->driver = driver; }
    
    const std::string& getHardware() const { return hardware; }
    void setHardware(const std::string& hardware) { this->hardware = hardware; }
    
    bool hasTimestamps() const  { return timestamps; }
    void setTimestamps(bool timestamps) { this->timestamps = timestamps; }

    bool isRemote() const { return remote; }
    void setRemote(bool remote){ this->remote = remote; }

    bool isManual() const { return manual; }
    void setManual(bool manual) { this->manual = manual; }
    
    void setManualParams(std::string manualParams) { ; }
    std::string getManualParams() { return manual_params; }
    
    void setDeviceArgs(Kwargs deviceArgs) { this->deviceArgs = deviceArgs; }
    Kwargs getDeviceArgs() { return deviceArgs; }

	void setStreamArgs(Kwargs deviceArgs){ this->streamArgs = streamArgs; }
	Kwargs getStreamArgs() { return streamArgs; }

//===========================================================    
	void setSoapyDevice(Streaming *dev) {
		
		if (RsJetDevice) {
			//SoapySDR::Device::unmake(soapyDevice);
		}
		RsJetDevice = dev;
	}
 //===========================================================
	Streaming *getRsJetDevice() 
	{
		if (RsJetDevice == nullptr) {
			RsJetDevice = new Streaming();
		}
		return RsJetDevice;
	}
 //===========================================================================
	SDRRangeMap getGains(int direction, size_t channel) {
		std::map<std::string, Range> gainMap;
		return gainMap;
	}
 //===========================================================================   
    bool hasCORR(int direction, size_t channel)  { return false; }
 //===========================================================================   
	std::vector<long> getSampleRates(int direction, size_t channel)
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
 //===========================================================================
	std::vector<std::string> getAntennaNames(int direction, size_t channel)
	{
		return std::vector<std::string>();  
	}
 //===========================================================================
 std::string getAntennaName(int direction, size_t channel) { return std::string(""); }
 //===========================================================================
 long getSampleRateNear(int direction, size_t channel, long sampleRate_in) 
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
 //===========================================================================
	//read the current gain of name gainName (must exist in getGains(), else return 0)
	//in the device.
	double getCurrentGain(int direction, size_t channel, const std::string& gainName) { return 0.0; }

private:
    int index = 0;
    std::string name, serial, product, manufacturer, tuner;
    std::string driver, hardware, manual_params;
    bool timestamps, available, remote, manual;
    std::atomic_bool active;

    Kwargs deviceArgs, streamArgs;
    Streaming *RsJetDevice;
};
*/