#include "Rs_FX3_USB.h"
#include "SoapyRsJet.hpp"
//D:\PotHos\SoapyRsJet\Build\RelWithDebInfo\

/***********************************************************************
 * Find available devices
 **********************************************************************/
SoapySDR::KwargsList findRsJet(const SoapySDR::Kwargs &args)
{
    //locate the device on the system...
    //return a list of 0, 1, or more argument maps that each identify a device
	std::vector<SoapySDR::Kwargs> results;
	RsFx3_USB  *pUSBDev = new RsFx3_USB(NULL);
	char model[255];
	BOOL ret = pUSBDev->OpenDevice(model);
	if (ret) {
		SoapySDR::Kwargs options;
		SoapySDR::logf(SOAPY_SDR_INFO, "findRsJet Ok.");
		options["device"] = "RsJet";
		options["hardware"] = "RsJet v1.0";
		options["instance"] = "0";
		std::wstring ws(pUSBDev->getSerialNumber());
		std::string shortSerial(std::string(ws.begin(),ws.end()));
		options["serial"] = shortSerial;
		shortSerial.replace(8, 16, "..");
		options["label"] = "RsJet #" + options["instance"] + " [" + shortSerial + "]";
		results.push_back(options);
	}
	delete pUSBDev;
	return results;
}

/***********************************************************************
 * Make device instance
 **********************************************************************/
SoapySDR::Device *makeRsJet(const SoapySDR::Kwargs &args)
{
    //create an instance of the device object given the args
    //here we will translate args into something used in the constructor
//	SoapySDR::logf(SOAPY_SDR_INFO, "makeRsJet....");
    return new RsJet(args);
}

/***********************************************************************
 * Registration
 **********************************************************************/
static SoapySDR::Registry RegistryRsJet("rsjet", &findRsJet, &makeRsJet, SOAPY_SDR_ABI_VERSION);

/*
Debug 
c:\Program Files\PothosSDR\bin\SoapySDRutil.exe
--make="driver=rsjet"
*/