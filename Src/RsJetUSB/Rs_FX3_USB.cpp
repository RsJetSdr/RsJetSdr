//---------------------------------------------------------------------------
#include <math.h>
#include <algorithm>
#include <iostream>
#include <iomanip>
#pragma hdrstop
#include "Rs_FX3_USB.h"
#include <dbt.h>

//#include "logger.h"
// extern  GUID CYUSBDRV_GUID
//#define RsFx3_USB_GUID L"{36FC9E60-C465-11CF-8056-444553540000}"
//---------------------------------------------------------------------------
//#pragma package(smart_init)

 
//================
template <class T> T ceil_log2(T num){
	return std::ceil(std::log(num)/std::log(T(2)));
}
//====================
template <class Value>
int Sign(Value Val) {
  if (Val == 0.)  return 0;
  if (Val > 0.)   return 1;
  else            return -1;
}

//==============================================================================
RsFx3_USB::RsFx3_USB(HANDLE extWin) :
	FextWin(extWin),
	mUSBDevice(NULL),
	bFirmvare(false), bFpga(false),
	FsamleAdc(81920000.0),			//Constant
	MB_CONTROL0_BR(0x80000000),      
	REG_ADC(0x00000401),
	SampleRate(0),
	hMutex(nullptr),
	StreameptIn(NULL),
	pSerialNum(nullptr),
	ep_count(-1),
	FreqDDC(CENTER_REC_140MHZ),
	bSuperSpeed(false),
	last_error(nullptr)	 
{
	last_error_str = "None";
	memset(Str_Id, 0, sizeof(Str_Id));
}
//==============================================================================
RsFx3_USB::~RsFx3_USB()
{
	CloseDevice();
		ReleaseMutex(hMutex);
		CloseHandle(hMutex);
	//	SoapySDR::logf(SOAPY_SDR_INFO, "RsJet_close()");
	
}
//==============================================================================
void RsFx3_USB::CloseDevice()
{

	if (mUSBDevice) { delete mUSBDevice; }
	mUSBDevice = NULL;
}
//==============================================================================
BOOL RsFx3_USB::OpenDevice(char* model )
{
  *model = 0;

  hMutex = OpenMutex(MUTEX_ALL_ACCESS, 0, "Global\\72A17822-6ED4-4FE5-A216-F4F0A0F073BC");
  if (!hMutex)
	  hMutex = CreateMutex(0, 0, "Global\\72A17822-6ED4-4FE5-A216-F4F0A0F073BC");
  else
  {
	  last_error_str = "Driver is running.";
	  return FALSE;
  }
  
  last_error_str="Could not RsJetFX3! Make sure board is connected and powered up.";

  if(findRsDevice()!=0)
		return FALSE;

   last_error_str="";

	 // log(mDEBUG,"Found RsJetFX3...");
	   bSuperSpeed = mUSBDevice->bSuperSpeed;
	   bSuperSpeed ?
		   std::sprintf(model/*,32*/, "RsJet USB3.0") :
		   std::sprintf(model/*,32*/, "RsJet USB2.0");

	   Sleep(100);

	//	SoapySDR::log(mDEBUG,"Loading RsJetFX3 FPGA... Please wait...");
	 //Load FPGA
	  

	   if (!Get_Id(Str_Id)){
		    last_error_str="Get_Id";
		   return FALSE;
		}
	   
	 
	  if(loadFpga(RsCyFx3_FPGA)!=0){
				last_error_str="Error Load FPGA";
				return FALSE;
		}

	for (int nCount = 0; nCount < mUSBDevice->EndPointCount(); nCount++ )
	{
		StreameptIn = NULL;
		CCyUSBEndPoint *ept = mUSBDevice->EndPoints[nCount];

		if(ept->Address==0x81)
		{
	//		SoapySDR::log(mDEBUG,"RsJetFX3 FPGA is loaded...");
	
			StreameptIn = ept;
			StreameptIn->Abort();
//Reset FPGA
		MB_CONTROL0_BR = 0x80000000;
		OutCmdReg(MB_RX_CONTRL0, MB_CONTROL0_BR);
		Sleep(10);
		MB_CONTROL0_BR = 0;
//Set default 
		MB_CONTROL0_BR |= RANDOM | BAND;   //RANDOM ON  FPGA
		OutCmdReg(MB_RX_CONTRL0 ,MB_CONTROL0_BR);
		Sleep(1);
//Init ADC
		//out adc_spi
		REG_ADC = REG_ADC | 0x2;    //RANDOM ON  ADC
		OutCmdReg(MB_ADC_REG,REG_ADC);
		Sleep(1);
		OutCmdReg(MB_RX_MUX, FLAG_DSP_RX_MUX_REAL_MODE | FLAG_DSP_RX_MUX_SWAP_IQ );
		Sleep(1);
//Set Mode
		StopCommand();  //outcmdreg(MB_RAW_CTRL,MODE_STOP | 2048);
		Sleep(1);
		SetSampleRate(2.56e6); //2.56 MHz   default
		Sleep(1);
		SetFreqDDC(CENTER_FFT_140MHZ);
  	return TRUE;

		}
	}
	last_error_str="Not Find Hardware";
	return FALSE;
}
//=============================================================================
int RsFx3_USB::findRsDevice()
  {
	
   USHORT  VendorID, ProductID;
   bFirmvare = false;

   if (mUSBDevice) {
	   delete mUSBDevice;
	   mUSBDevice=nullptr;
   }

   CCyFX3Device *mUSBFx3 = new CCyFX3Device();
   UCHAR DeviceCount = mUSBFx3->DeviceCount();
   
   for (UCHAR d = 0; d < DeviceCount; d++)
   {
	   // Open() automatically calls Close() if necessary
	   mUSBFx3->Open(d);

	   VendorID = mUSBFx3->VendorID;
	   ProductID = mUSBFx3->ProductID;

	   if ((VendorID == VENDOR_ID) &&
		   ((ProductID == PRODUCT_BOOT1) || (ProductID == PRODUCT_BOOT2) ||
		   (ProductID == PRODUCT_APL)))
	   {

		   if (ProductID == PRODUCT_APL) {

			   delete mUSBFx3; //Close

			   mUSBDevice = new CCyUSBDevice(FextWin);
			   mUSBDevice->Open(d);

			   bFirmvare = true;
			   pSerialNum = mUSBDevice->SerialNumber;
			   return SUCCESS;
		   }
		   else {  //load Firmware
				   //Log Download Firmware

			   if (SUCCESS == mUSBFx3->DownloadFw(RsCyFx3_FIRMWARE, RAM))
			   {
				   //  mUSBFx3.Reset();
				   mUSBFx3->ReConnect();
				   delete mUSBFx3;
				   mUSBFx3 = nullptr;


				   for (int i = 0; i < 10; i++) {
					   Sleep(1000);   // delay for reenumeration
					   mUSBDevice = new CCyUSBDevice(FextWin);
					   pSerialNum = mUSBDevice->SerialNumber;
					   if ((mUSBDevice->VendorID == VENDOR_ID) && (mUSBDevice->ProductID == PRODUCT_APL))
					   {
						   bFirmvare = true;
						   last_error_str = "";
						   return SUCCESS;
						  
					   }
					   else {
						   delete mUSBDevice;
						   mUSBDevice = nullptr;
					   }

				   }//for
				   last_error_str = "Error reenumeration device.";
				   return -1;

			   }
			   else {
				   delete mUSBFx3;
				   last_error_str = "Error Load Firmware to RAM";
				   return INVALID_FILE;
			   }
		   }
	   }
	   }//for
   
			if(mUSBFx3)
					delete mUSBFx3;

		  //last_error_str = "Could not find device.";
		  return -1;
  }
 

//============================================================================
/*
int  RsFx3_USB::loadFirmware( const char * filename )
 {

	if((ProductID == PRODUCT_BOOT1)||(ProductID == PRODUCT_BOOT2)){
			if(SUCCESS == mUSBDevice->DownloadFw((char *)filename , RAM))
			{
						bFirmvare=true;
						return SUCCESS;
			}
				last_error_str = "Error Load Firmware to RAM";
				return INVALID_FILE;
			}else {
				bFirmvare=true;
				return SUCCESS;
	}
 }
 */
//============================================================================
int RsFx3_USB::loadFpga( const char * filename )
{
	long fwSize = 0;
	PUCHAR FwImage;
	FILE *FwImagePtr;
	UCHAR  buf[16]={0};
	long len;
	bool bRetCode;

	  fopen_s(&FwImagePtr, filename, "rb");

	if (FwImagePtr == NULL)
		return INVALID_FILE;

	 /* Find the length of the image */
	fseek (FwImagePtr, 0, SEEK_END);
	fwSize = ftell(FwImagePtr);
	fseek (FwImagePtr, 0, SEEK_SET);

	if (fwSize <= 0)
	{
		fclose (FwImagePtr);
		last_error_str = "Size file <=0";
		return INVALID_FILE;
	}

	/* Allocate memory for the image */
	FwImage =  new unsigned char[fwSize];

	if (FwImage == NULL)
		return INVALID_FILE;


	  /* Read into buffer */
	fread (FwImage, fwSize, 1, FwImagePtr);
	fclose (FwImagePtr);
	//Получим стфтуы
	mUSBDevice->ControlEndPt->Target    = TGT_DEVICE; //TGT_ENDPT ;
	mUSBDevice->ControlEndPt->ReqType   = REQ_VENDOR ;
	mUSBDevice->ControlEndPt->Direction = DIR_FROM_DEVICE;
	mUSBDevice->ControlEndPt->ReqCode   = VND_CMD_CFGSTAT;
	mUSBDevice->ControlEndPt->Value     = 0;
	mUSBDevice->ControlEndPt->Index     = 1;
	len =sizeof(buf);
	buf[0]=0;
	//send vendor command to know the status of Configuration
	bRetCode =	mUSBDevice->ControlEndPt->XferData(buf,len,NULL);
	if(!bRetCode){
		 delete[] FwImage;
		 last_error_str = "Error Read Status";
		 return  FAILED;
	}
	 if(!buf[0]){
		buf[0] = (fwSize & 0x000000FF);
		buf[1] = ((fwSize & 0x0000FF00) >> 8);
		buf[2] = (UCHAR)((fwSize & 0x00FF0000) >> 16);
		buf[3] = ((fwSize & 0xFF000000) >> 24);
		mUSBDevice->ControlEndPt->Target    = TGT_DEVICE ;
		mUSBDevice->ControlEndPt->ReqType   = REQ_VENDOR ;
		mUSBDevice->ControlEndPt->Direction = DIR_TO_DEVICE;
		mUSBDevice->ControlEndPt->ReqCode   = VND_CMD_CFGLOAD;
		mUSBDevice->ControlEndPt->Value     = 0;
		mUSBDevice->ControlEndPt->Index     = 1;
		len =sizeof(buf);

	 bRetCode =	mUSBDevice->ControlEndPt->XferData(buf,len,NULL);

	if(!bRetCode){
		delete[] FwImage;
		last_error_str = "Error Change Status";
		 return  FAILED;
	}
	   bRetCode =	mUSBDevice->BulkOutEndPt->XferData(FwImage,fwSize,NULL);
	   delete[] FwImage;

	   if (!bRetCode) {
		   last_error_str = "Error Write Image FPGA";
		   return  FAILED;
	   }


	Sleep(500);
	mUSBDevice->ControlEndPt->Target    = TGT_DEVICE ;
	mUSBDevice->ControlEndPt->ReqType   = REQ_VENDOR ;
	mUSBDevice->ControlEndPt->Direction = DIR_FROM_DEVICE;
	mUSBDevice->ControlEndPt->ReqCode   = VND_CMD_CFGSTAT;
	mUSBDevice->ControlEndPt->Value     = 0;
	mUSBDevice->ControlEndPt->Index     = 1;

	len =sizeof(buf);
	 buf[0]=0;
	//send vendor command to know the status of Configuration
	bRetCode =	mUSBDevice->ControlEndPt->XferData(buf,len,NULL);
	if(!bRetCode || (buf[0]==0) ){
		 last_error_str = "Error Load FPGA";
		 return  FAILED;
	}

 }
	 bFpga=true;
	 return      SUCCESS;
}
//=============================================================================
 void RsFx3_USB::SetSampleRate(double rate)
 {
	 
	 if (SampleRate == rate)
		 return;
//	 std::unique_lock<std::recursive_mutex> lock(_accessMutex);
	 std::unique_lock<std::mutex> lock(_accessMutex);

	  int decim_rate  = (int)std::round(FsamleAdc/rate);
	  int decim = decim_rate;

//determine which half-band filters are activated
		int hb0 = 0, hb1 = 0;
		// hb0 can only be enabled if the rate will be decimated by at least 2 between the CIC and hb1
		if (decim >= 4 && decim % 2 == 0){
			hb0 = 1;
			decim /= 2;
		}
		if (decim % 2 == 0){
			hb1 = 1;
			decim /= 2;
		}
   decim = decim & 0xff;
   const double _dsp_extra_scaling = 1.0;
   const double rate_pow = std::pow(double(decim),4);

   double     _scaling_adjustment   = std::pow(2, ceil_log2(rate_pow))/(1.65 * rate_pow);
   
/*
  Borland
  const double factor = 1.0 + std::max(ceil_log2(_scaling_adjustment), 0.0);
*/
   const double factor = 1.0 + std::max(log2(_scaling_adjustment), 0.0);

   const double target_scalar = (1 << 17)*_scaling_adjustment/_dsp_extra_scaling/factor;
   
   const  int actual_scalar = (int)std::round(target_scalar);
   
   //_fxpt_scalar_correction = target_scalar/actual_scalar*factor; //should be small
   	OutCmdReg(MB_RX_SCALE_IQ, actual_scalar);
	OutCmdReg(MB_RX_DECIM, (hb1 << 9) | (hb0 << 8) | (decim));
	
	SampleRate = std::round(FsamleAdc / decim_rate);

//	logf("FPGA SR hb0=%i: hb1=%i: decim=%i",hb0,hb1,decim);
 
  }
  //==============================================================================
void  RsFx3_USB::get_freq_and_freq_word(
		const double requested_freq,
		const double tick_rate,
		double &actual_freq,
		int &freq_word
)
{

	//correct for outside of rate (wrap around)
	double freq = std::fmod(requested_freq, tick_rate);
		if(std::abs(freq) > tick_rate/2.0)
							freq -= Sign(freq) * tick_rate;

   //confirm that the target frequency is within range of the CORDIC
   // UHD_ASSERT_THROW(std::abs(freq) <= tick_rate/2.0);
	/* Now calculate the frequency word. It is possible for this calculation
	 * to cause an overflow. As the requested DSP frequency approaches the
	 * master clock rate, that ratio multiplied by the scaling factor (2^32)
	 * will generally overflow within the last few kHz of tunable range.
	 * Thus, we check to see if the operation will overflow before doing it,
	 * and if it will, we set it to the integer min or max of this system.
	 */
	freq_word = 0;
	static const double scale_factor = std::pow(2.0, 32);
	{
		/* The operation is safe. Perform normally. */
		//freq_word = (int)_round((freq / tick_rate) * scale_factor);
		freq_word = (int)std::round((freq / tick_rate) * scale_factor);

	}

	actual_freq = (double(freq_word) / scale_factor) * tick_rate;
}
//===============================================================================
  double  RsFx3_USB::SetFreqDDC(const double requested_freq)
  {
//	  std::unique_lock<std::recursive_mutex> lock(_accessMutex);
	  std::unique_lock<std::mutex> lock(_accessMutex);

	    double actual_freq;
		int freq_word;

		get_freq_and_freq_word(requested_freq, FsamleAdc, actual_freq, freq_word);
		OutCmdReg(MB_RX_FREQ, freq_word);
		FreqDDC = actual_freq;
		return actual_freq;
 }
//=======================================================================================
  bool     RsFx3_USB::SelectBand(bool band) {
	  //band ? MB_CONTROL0_BR |= BAND : MB_CONTROL0_BR &= ~BAND;
	  return OutCmdReg(MB_RX_CONTRL0, 
		  (band ? MB_CONTROL0_BR |= BAND : MB_CONTROL0_BR &= ~BAND)
	  );
  }
 //======================================================================================
  /*
  Return name rbf for FPGA
  */
  bool RsFx3_USB::Get_Id(UCHAR *pData)
  {
	  std::unique_lock<std::mutex> lock(_regMutex);
	  long len=0;
	  //Out Data
	  mUSBDevice->ControlEndPt->TimeOut = 1000;
	  mUSBDevice->ControlEndPt->Target = TGT_DEVICE;
	  mUSBDevice->ControlEndPt->ReqType = REQ_VENDOR;
//	  mUSBDevice->ControlEndPt->Direction = DIR_FROM_DEVICE;
	  mUSBDevice->ControlEndPt->ReqCode = VND_CMD_ID_CHECK;
	  mUSBDevice->ControlEndPt->Value = 0;
	  mUSBDevice->ControlEndPt->Index = 0;
	  //out
	  if (!mUSBDevice->ControlEndPt->Write(pData, len)) {
		  last_error_str = "OutSerial(): OUT control transfer failed.";
		  return false;
	  }
	  len = 32;
	

	  if (!mUSBDevice->ControlEndPt->Read(pData, len)) {
		  last_error_str = "OutSerial(): IN control transfer failed.";
		  return false;
	  }
	  
	  

	  return true;

  }

 bool RsFx3_USB::OutSerial(UCHAR *pData,DWORD *Snt,int Echo_Size)
{
	 std::unique_lock<std::mutex> lock(_regMutex);
	long len;
   //Out Data
   	mUSBDevice->ControlEndPt->TimeOut = 1000;
	mUSBDevice->ControlEndPt->Target    = TGT_DEVICE ;
	mUSBDevice->ControlEndPt->ReqType   = REQ_VENDOR ;
	// ----- CMD_UART ------
	mUSBDevice->ControlEndPt->ReqCode   = VND_CMD_UART;
	mUSBDevice->ControlEndPt->Value     = 0;
	mUSBDevice->ControlEndPt->Index     = Echo_Size;
	len =pData[1];
	//out
	if(!mUSBDevice->ControlEndPt->Write(pData,len)){
		last_error_str = "OutSerial(): OUT control transfer failed.";
		*Snt=len;
		return false;
	 }

	//Input  Data
   len = Echo_Size;
   if(!len){    //Skip Read Echo
		*Snt=len;
		return true;
   }
    Sleep(1);
	mUSBDevice->ControlEndPt->Index   = 200;  //Time_Out wait echo ( ms )

	if(!mUSBDevice->ControlEndPt->Read(pData,len)){
		last_error_str = "OutSerial(): IN control transfer failed.";
		*Snt=len;
		return false;
	 }
		*Snt=len;
		return true;

}
 //=============================================================
 bool RsFx3_USB::Command(cmd_USB *pData)
 {
	 std::unique_lock<std::mutex> lock(_regMutex);
	 long len = pData->Len;
	 //Write Data
	 mUSBDevice->ControlEndPt->Target = TGT_DEVICE;
	 mUSBDevice->ControlEndPt->ReqType = REQ_VENDOR;

	 mUSBDevice->ControlEndPt->ReqCode = VND_CMD_TASK;

	 mUSBDevice->ControlEndPt->Value = pData->wValue;
	 mUSBDevice->ControlEndPt->Index = pData->wIndex;
	 bool result = mUSBDevice->ControlEndPt->Write(pData->inData, len);

	
	 if (!result) {
		 last_error_str = "RsFx3_USB::OutSerial: OUT control transfer failed.";
		 return false;
	 }
	 else
		 last_error_str = "";
	 return true;
 }
 //=============================================================================
 bool RsFx3_USB::StopCommand()
{
 return OutCmdReg(MB_RAW_CTRL,MODE_STOP | 2048);
}
 //=============================================================================
bool RsFx3_USB::BurstCommand(USHORT size)
{
 return OutCmdReg(MB_RAW_CTRL,MODE_BURST | ((size>2048)? 2048:size ));
}
//==============================================================================
bool RsFx3_USB::RawCommand()
{
  return	OutCmdReg(MB_RAW_CTRL,MODE_RAW | 2048);
}
 //=====================================================================================
  bool RsFx3_USB::OutCmdReg(UCHAR adr,ULONG data)
  {
		UCHAR  buf[5];
		buf[0] = adr;
		buf[1] = (UCHAR)((data & 0xFF000000) >> 24);
		buf[2] = (UCHAR)((data & 0x00FF0000) >> 16);
		buf[3] = (UCHAR)((data & 0x0000FF00) >> 8);
		buf[4] = (UCHAR)(data & 0x000000FF);

		cmd_USB 	data_out={0};

		data_out.wValue = 0;    //Command FPGA
		data_out.wIndex = 0;

		data_out.Len = sizeof(buf);
		data_out.inData= buf;
		
//		if((adr==0)|| (adr == 8))
//		SoapySDR_logf(mDEBUG, "OutCmdReg Adr = %X : Data = %X " , adr, data);

		return SetCmdFPGA(&data_out);
  }
 //=============================================================
  bool RsFx3_USB::SetCmdFPGA(cmd_USB *pData)
  {
	  std::unique_lock<std::mutex> lock(_regMutex);

	  long len = pData->Len;
	  mUSBDevice->ControlEndPt->Target = TGT_DEVICE;
	  mUSBDevice->ControlEndPt->ReqType = REQ_VENDOR;
	  mUSBDevice->ControlEndPt->ReqCode = VND_CMD_TASK;
	  mUSBDevice->ControlEndPt->Value = pData->wValue;
	  mUSBDevice->ControlEndPt->Index = pData->wIndex;
	  bool result = mUSBDevice->ControlEndPt->Write(pData->inData, len);
	  
	  if (!result) {
		  last_error_str = "Command(): OUT control transfer failed.";
		  return false;
	  }else{
		  last_error_str = "";
	       return true;
	  }	   
  }
