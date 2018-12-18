// Copyright (c) Charles J. Cliffe
// SPDX-License-Identifier: GPL-2.0+

#pragma once


#include <atomic>
#include <mutex>

//#include "DataTree.h"
#include "RsSDRDefs.h"
#include "SDRDeviceInfo.h"

typedef std::map<std::string, std::string> ConfigSettings;
typedef std::map<std::string, float> ConfigGains;

class DeviceConfig {
public:
    DeviceConfig() : deviceId(""){
		ppm.store(0);
		offset.store(0);
		agcMode.store(true);
		sampleRate.store(0);
	}

    DeviceConfig(std::string deviceId) : DeviceConfig() {
		this->deviceId = deviceId;
	}

    void setPPM(int ppm) {this->ppm.store(ppm);}
    int  getPPM() {	return ppm.load();}

    
    void setOffset(long long offset) {this->offset.store(offset);}
    long long getOffset() {return offset.load();}

    void setSampleRate(long srate){sampleRate.store(srate);}
    long getSampleRate() {return sampleRate.load();}


    void setAntennaName(const std::string& name) {antennaName = name;}
    const std::string& getAntennaName() {return antennaName;}

	void setAGCMode(bool agcMode) {	this->agcMode.store(agcMode);}
	bool getAGCMode() {return agcMode.load();}
    
	void setDeviceId(std::string deviceId) {
		std::lock_guard < std::mutex > lock(busy_lock);
		this->deviceId = deviceId;
	}

	std::string getDeviceId() {
		std::string tmp;
		std::lock_guard < std::mutex > lock(busy_lock);
		tmp = deviceId;
		return tmp;
	}

	void setDeviceName(std::string deviceName) {
		std::lock_guard < std::mutex > lock(busy_lock);
		this->deviceName = deviceName;
	}

	std::string getDeviceName() {
		std::string tmp;
		std::lock_guard < std::mutex > lock(busy_lock);
		tmp = (deviceName == "") ? deviceId : deviceName;
		return tmp;
	}

	void setStreamOpts(ConfigSettings opts) {streamOpts = opts;}
	ConfigSettings getStreamOpts() {return streamOpts;}


    void setStreamOpt(std::string key, std::string value) {
		streamOpts[key] = value;
	}
    std::string getStreamOpt(std::string key, std::string defaultValue) {
		if (streamOpts.find(key) == streamOpts.end()) {
			return defaultValue;
		}
		return streamOpts[key];
	}
    
    void setSettings(ConfigSettings settings) {this->settings = settings;}
	ConfigSettings getSettings() { return settings; }

	void setSetting(std::string key, std::string value) {this->settings[key] = value;}
	std::string getSetting(std::string key, std::string defaultValue) {
		if (settings.find(key) == settings.end()) {
			return defaultValue;
		}
		return settings[key];
	}


	void DeviceConfig::setGains(ConfigGains gains) { this->gains = gains; }
	ConfigGains DeviceConfig::getGains() { return gains; }

	void setGain(std::string key, float value) {gains[key] = value;}

	float DeviceConfig::getGain(std::string key, float defaultValue) {
		if (gains.find(key) != gains.end()) {
			return gains[key];
		}
		return defaultValue;
	}

	void setRigIF(int rigType, long long freq) {
		rigIF[rigType] = freq;
	}

	long long getRigIF(int rigType) {
		if (rigIF.find(rigType) != rigIF.end()) {
			return rigIF[rigType];
		}
		return 0;
	}

    void save(DataNode *node);
    void load(DataNode *node);

private:
    std::string deviceId;
    std::string deviceName;

    std::mutex busy_lock;

    std::atomic_int ppm;
    std::atomic_llong offset;
    std::atomic_bool agcMode;
    std::atomic_long sampleRate;
    std::string antennaName;
    ConfigSettings streamOpts;
    ConfigGains gains;
    std::map<std::string, std::string> settings;
    std::map<int, long long> rigIF;
};


class AppConfig {
public:

    enum PerfModeEnum {
        PERF_LOW = 0,
        PERF_NORMAL = 1,
        PERF_HIGH = 2
    };


	AppConfig() : configName("") {
		winX.store(0);
		winY.store(0);
		winW.store(0);
		winH.store(0);
		winMax.store(false);
		showTips.store(true);
		perfMode.store(PERF_NORMAL);
		themeId.store(0);
		fontScale.store(0);
		snap.store(1);
		centerFreq.store(100000000);
		waterfallLinesPerSec.store(DEFAULT_WATERFALL_LPS);
		spectrumAvgSpeed.store(0.65f);
		dbOffset.store(0);
		modemPropsCollapsed.store(false);
		mainSplit = -1;
		visSplit = -1;
		bookmarkSplit = 200;
	}

    std::string getConfigDir();
	DeviceConfig *getDevice(std::string deviceId) {
		{
			if (deviceConfig.find(deviceId) == deviceConfig.end()) {
				deviceConfig[deviceId] = new DeviceConfig();
			}
			DeviceConfig *conf = deviceConfig[deviceId];
			conf->setDeviceId(deviceId);
			return conf;
		}
	}
	/*
    void setWindow(wxPoint winXY, wxSize winWH);
    wxRect *getWindow();
    */
    void setWindowMaximized(bool max);
    bool getWindowMaximized();

    void setModemPropsCollapsed(bool collapse);
    bool getModemPropsCollapsed();

    void setShowTips(bool show);
    bool getShowTips();

    void setPerfMode(PerfModeEnum mode);
    PerfModeEnum getPerfMode();
    
    void setTheme(int themeId);
    int getTheme();

    void setFontScale(int scaleValue);
    int getFontScale();

    void setSnap(long long snapVal);
    long long getSnap();
    
    void setCenterFreq(long long freqVal);
    long long getCenterFreq();
    
    void setWaterfallLinesPerSec(int lps);
    int getWaterfallLinesPerSec();
    
    void setSpectrumAvgSpeed(float avgSpeed);
    float getSpectrumAvgSpeed();
    
    void setDBOffset(int offset);
    int getDBOffset();
    
    void setManualDevices(std::vector<SDRManualDef> manuals);
    std::vector<SDRManualDef> getManualDevices();
    
    void setMainSplit(float value);
    float getMainSplit();
    
    void setVisSplit(float value);
    float getVisSplit();
    
    void setBookmarkSplit(float value);
    float getBookmarkSplit();
    
    void setBookmarksVisible(bool state);
    bool getBookmarksVisible();
    
	//Recording settings:
    void setRecordingPath(std::string recPath);
    std::string getRecordingPath();
	bool verifyRecordingPath();

	void setRecordingSquelchOption(int enumChoice);
	int getRecordingSquelchOption();
    
	void setRecordingFileTimeLimit(int nbSeconds);
	int getRecordingFileTimeLimit();

	void setConfigName(std::string configName);
    std::string getConfigFileName(bool ignoreName=false);
   
	bool save();
    bool load();
    bool reset();
private:
    std::string configName;
    std::map<std::string, DeviceConfig *> deviceConfig;
    std::atomic_int winX,winY,winW,winH;
    std::atomic_bool winMax, showTips, modemPropsCollapsed;
    std::atomic_int themeId;
    std::atomic_int fontScale;
    std::atomic_llong snap;
    std::atomic_llong centerFreq;
    std::atomic_int waterfallLinesPerSec;
    std::atomic<float> spectrumAvgSpeed, mainSplit, visSplit, bookmarkSplit;
    std::atomic_int dbOffset;
    std::vector<SDRManualDef> manualDevices;
    std::atomic_bool bookmarksVisible;

    std::atomic<PerfModeEnum> perfMode;

    std::string recordingPath = "";
	int recordingSquelchOption = 0;
	int recordingFileTimeLimitSeconds = 0;
};
