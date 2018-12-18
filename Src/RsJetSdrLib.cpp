#include "Rsjet\RsJetSuite.h"
#include "Rs_FX3_USB.h"
#include "RtAudio.h"
#include "Streaming.h"
#include "SpectrHigh.h"
//
#include "RsSdrDefs.h"
#include "RsDemod.h"

//void ErrorHandler(LPTSTR lpszFunction);
//============================================================
BOOL APIENTRY DllMain( HMODULE hModule, DWORD ul_reason_for_call, LPVOID lpReserved)
{
	return TRUE;
}
 
//==============================================================
/*
Driver USB Hardware
*/
RsFx3_USB  *pUSBDev = nullptr; 
//============================================================
class TRsJetSdr:public RsJetSdr
{
public:
	 TRsJetSdr();
	~TRsJetSdr();
	
   const   CHAR *  __stdcall  GetLastError(void) { return CurrError; }

   HANDLE __stdcall OpenUSBDevice(int DebugLevel, HANDLE hWnd, char **Str){
		  if (DebugLevel) {
			  if (AllocConsole()) {
			  std::setlocale(LC_ALL, "");
			  freopen_s((FILE**)stdout, "CONOUT$", "w", stdout);
			  //freopen("CONOUT$", "w", stdout);
			  SetConsoleTextAttribute(GetStdHandle(STD_OUTPUT_HANDLE), FOREGROUND_GREEN | FOREGROUND_BLUE | FOREGROUND_RED);
			  SetConsoleTitle("RsSDR: stdout");
		  }

		  //refresh
		  std::cout.clear();
		  /* std::ofstream ob;
		  std::streambuf *sb = std::cout.rdbuf();
		  std::cout.rdbuf(sb);
		  */
	  }
	  
	  pUSBDev = new RsFx3_USB(hWnd);
	  *Str = tmpBuf;

	  if (!pUSBDev->OpenDevice(tmpBuf)) {
		  memcpy(tmpBuf, pUSBDev->lastError(), sizeof(tmpBuf));
		  delete pUSBDev;
		  pUSBDev = nullptr;
		  return nullptr;
	  }
	  else {
		  mDem =  new Demodulator();
		  rsGetApp().OnInit();
		  return pUSBDev->DrvHandle();
	  }
  }
	//>>>>>>>>>>>>>> Methods of IUnknown <<<<<<<<<<<<<<<<<<<<<<<<
	HRESULT __stdcall QueryInterface(REFIID riid, void **ppvObject)
	{
		if (IsEqualGUID(riid, __uuidof(RsJetSdr))) {
			*ppvObject = (void *)this;
			return S_OK;
		}
		else {
			*ppvObject = NULL;
			return E_NOINTERFACE;
		}
	}
	//=====================================================================
	ULONG __stdcall AddRef(void)
	{
		//std::cout << "AddRef" << std::endl;
		return InterlockedIncrement(&FRefCount);
	}
	//=====================================================================
	ULONG __stdcall Release(void)
	{
		//std::cout << "Release" << std::endl;
		ULONG result = InterlockedDecrement(&FRefCount);
		if (!result)
			delete this;
		return result;
	}
	BOOL   __stdcall  GetPwPPM(int *Pw); //0 - 100 %
	BOOL   __stdcall  SetPwPPM(int Pw);
	BOOL   __stdcall  WriteFlash(UCHAR *Stat);
	BOOL	__stdcall  SetFreqGain(int index_in, FREQSTRUCT *FreqStr, UCHAR *Stat);
	BOOL __stdcall     SetConv(DWORD Freq, UINT8 Sw, UINT8 Pw);
	//========================================================================
	BOOL	__stdcall  GetFreqGain(int num, FREQSTRUCT *FreqStr);
	//========================================================================
	int   __stdcall  GetSpectr_10KHz(ControlStruct *Control);
	DWORD	__stdcall  GetDataFFT(PUCHAR Data, DWORD Len);
//=============================================================================
	HANDLE		__stdcall SetModem(Demod_Str *pDemodulator);
	int		__stdcall GetIQData(SdrIQData *piqData);
	float   __stdcall GetAudioGain(HANDLE demod);
	void  __stdcall   SetAudioGain(HANDLE demod, float gain);
	float __stdcall   GetSignalLevel(HANDLE demod);
	int __stdcall     GetAudioSampleRate(HANDLE demod);
	void __stdcall    SetAudioSampleRate(HANDLE demod, int sampleRate);
	int  __stdcall    GetBandwidth(HANDLE demod);
	void  __stdcall   SetBandwidth(HANDLE demod, int bw);
//=============================================================================
	int     __stdcall StartDevice(int SampleRate){ 
		if (!pUSBDev)return -1;
		
		StopSpectr();
		
		rsGetApp().startDevice(SampleRate); 
		mDem = (Demodulator *)rsGetApp().getInstance;
		ModeDem.store(true);
		return 0;
	}
//======================================================================
	void     __stdcall StopDevice(int waitMsForTermination) {
	if (!pUSBDev) return;
		   rsGetApp().stopDevice(waitMsForTermination);
		   ModeDem.store(false);
	}
 //=====================================================================
	void  __stdcall   StopSpectr() {
		if (mSpectrHigh) {
			delete  mSpectrHigh;
			mSpectrHigh = nullptr;
		}
	}
private:
	
	Demodulator *pmDem;
	ULONG FRefCount;
	CHAR tmpBuf[256];
	CHAR *CurrError;
	SpectrHigh *mSpectrHigh;
	Demodulator *mDem;
	DemodulatorThreadIQDataPtr data_in;
	DemodulatorInstancePtr newDemod;
	std::atomic_bool ModeDem;

	BYTE DbToBin(BYTE  val)	{return ((63 - (val << 1)) & 0x3F);}
	BYTE BinToDb(BYTE val) {
		BYTE  data = ~val;int db = 0;
		if (data & 0x20) db += 16;if (data & 0x10) db += 8;if (data & 0x08) db += 4;
		if (data & 0x04) db += 2; if (data & 0x02) db += 1; return db;
	}
};
//=================================================================

//==============================================================
TRsJetSdr::TRsJetSdr():
	FRefCount(0),
	mSpectrHigh(nullptr),//constr
	mDem(nullptr)

{
	newDemod = nullptr;
	ModeDem.store(false);
}
TRsJetSdr::~TRsJetSdr() 
{
	StopSpectr();

	if (mDem) {
		rsGetApp().OnExit();
		mDem = nullptr;
		}

	  if (pUSBDev) {
		  delete pUSBDev;
		  pUSBDev = nullptr;
	  }
}
//==============================================================
int    __stdcall  TRsJetSdr::GetSpectr_10KHz(ControlStruct *Control)
{

	if (!pUSBDev) 
		 return SDR_STREAM_ERROR;

	if (ModeDem.load()) {
		DemodulatorMgr *mgr = &rsGetApp().getDemodMgr();
		//terminate All
		//terminate
		if (newDemod != nullptr) {
			//Delete demodulator
			mgr->setActiveDemodulator(nullptr, true);
			mgr->setActiveDemodulator(nullptr, false);
			newDemod->setActive(false);
			mgr->deleteThread(newDemod);
		}
		mgr->terminateAll();
		newDemod = nullptr;
		StopDevice(100);
	}

	if (!mSpectrHigh) {
		mSpectrHigh = new SpectrHigh(Control);
	}
	else
		if ((mSpectrHigh->GetTypeWin() != Control->w_type)
			|| (mSpectrHigh->GetSize() != Control->nFFT))
		{
			StopSpectr();
			mSpectrHigh = new SpectrHigh(Control);
		}
/*
	std::cout << "FreqStart " << Control->FreqStart << std::endl;
	std::cout << "FreqEnd " << Control->FreqEnd << std::endl;
	std::cout << "nFFT " << Control->nFFT << std::endl;
	std::cout << "w_type " << Control->w_type << std::endl;
	std::cout << "SampleRate " << Control->SampleRate << std::endl;
	std::cout << "ModeAttn " << Control->ModeAttn << std::endl;
	std::cout << "AttnInput " << Control->AttnInput << std::endl;
	std::cout << "AttnOutPres " << Control->AttnOutPres << std::endl;
	std::cout << "AttnIF " << Control->AttnIF << std::endl;
*/
	//set task
	mSpectrHigh->Calculate_Start_Task(Control);

	DWORD  Status = WaitForMultipleObjects(
		2,
		mSpectrHigh->Event,
		false,
		TIME_OUT); //Wait time
	DWORD len = 0;
	switch (Status)
	{
	case WAIT_OBJECT_0:
		CurrError = "";
		len = mSpectrHigh->GetReadSize();
		return len;
	case WAIT_TIMEOUT:
		CurrError = "UsbDevice TIMEOUT";
		std::cout << "SpectrHigh::UsbDevice TIMEOUT" << std::endl;
		StopSpectr();
		return SDR_TIMEOUT;
	default:
		CurrError = "Error UsbDevice";
		std::cout << "SpectrHigh::Error UsbDevice" << std::endl;
		StopSpectr();
		return SDR_STREAM_ERROR;
	}
}
//===================================================================
DWORD  __stdcall  TRsJetSdr::GetDataFFT(PUCHAR Data, DWORD Len)
{
	if (!mSpectrHigh)
		return 0;
	UINT32 mLen = mSpectrHigh->GetData(Data, Len);
	return mLen;
}
//====================================================================
BOOL   __stdcall  TRsJetSdr::GetFreqGain(int num, FREQSTRUCT *FreqStr)
{
	BYTE Data[16] = { 0x5,5,0x3,0,0 }; //
	Data[3] = num;
	// CalcCS(Data);
	DWORD Snt = 0;
	if (!pUSBDev->OutSerial(Data, &Snt, 5 + sizeof(FREQSTRUCT))) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	if (!Snt) {
		CurrError = "Not Ack Presel";
		return FALSE;
	}
	FreqStr->freq = Data[4] + Data[5] * 256;
	FreqStr->att1 = BinToDb(Data[7]);
	FreqStr->att2 = BinToDb(Data[8]);
	FreqStr->att3 = BinToDb(Data[9]);
	return TRUE;
}
BOOL   __stdcall  TRsJetSdr::SetFreqGain(int index_in, FREQSTRUCT *FreqStr, UCHAR *Stat)
{

	BYTE cmd_SetRam[16] = { 0x5, //0 Start
		11,  		//1 Длина
		4,  			//2 Команда
		0,  			//3  Индех Входа
		0,0, 		//4:5 Частота
		0,			//6 Sw_Band
		0,0,0,       //Atn1,Atn2,Atn3
		0 //CS
	};

	cmd_SetRam[3] = index_in;
	cmd_SetRam[4] = (BYTE)(FreqStr->freq);
	cmd_SetRam[5] = (BYTE)(FreqStr->freq >> 8);

	if (FreqStr->freq > 0) {
		cmd_SetRam[6] = 0;
		cmd_SetRam[7] = DbToBin(FreqStr->att1);
		cmd_SetRam[8] = DbToBin(FreqStr->att2);
		cmd_SetRam[9] = DbToBin(FreqStr->att3);
	}
	else { //Set End
		cmd_SetRam[6] = 0xFF;
		cmd_SetRam[7] = 0xFF;
		cmd_SetRam[8] = 0xFF;
		cmd_SetRam[9] = 0xFF;

	}

	//	 CalcCS(cmd_SetRam);

	DWORD Snt = 0;
	if (!pUSBDev->OutSerial(cmd_SetRam, &Snt, 5)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	if (!Snt) {
		CurrError = "Not Ack Presel";
		return FALSE;
	}
	*Stat = cmd_SetRam[3];
	return TRUE;
}
BOOL   __stdcall  TRsJetSdr::SetPwPPM(int Pw)
{
	BYTE Data[16] = { 0x5,6,12,0,0,0 }; //
										/*
										Pw = Pw/1.25;
										if(Pw>=80)
										Pw=79;
										*/
	Data[3] = Pw;
	Data[4] = Pw >> 8;
	// CalcCS(Data);
	DWORD Snt = 0;
	if (!pUSBDev->OutSerial(Data, &Snt, 5)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	return TRUE;
}
BOOL   __stdcall  TRsJetSdr::GetPwPPM(int *Pw)
{
	BYTE Data[16] = { 0x5,4,11,0,0 }; //
									  //	 CalcCS(Data);
	DWORD Snt = 0;
	if (!pUSBDev->OutSerial(Data, &Snt, 6)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	// *Pw=((Data[3]+Data[4]*256)*1.25);
	*Pw = ((Data[3] + Data[4] * 256));

	return TRUE;

}
BOOL   __stdcall  TRsJetSdr::WriteFlash(UCHAR *Stat)
{
	BYTE Data[16] = { 0x5,4,5,0 }; //
	DWORD Snt = 0;
	if (!pUSBDev->OutSerial(Data, &Snt, 5)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	if (!Snt) {
		CurrError = "Not Ack Presel";
		return FALSE;
	}
	*Stat = Data[3];
	return TRUE;
}
//=============================================================
BOOL __stdcall TRsJetSdr::SetConv(DWORD Freq, UINT8 Sw, UINT8 Pw)
{
	unsigned char cmdbuf[32] = { 5,
		5,//Len
		9,//Cmd
		0 //Len Data
	};
	unsigned char CmdCnv[8] = { 5,
		7,//Len
		5,//Cmd
		0,//Freq_L
		0,//Freq_H
		0,//SW/PW
		0 //CS
	};

	if (Freq>12000)
		Freq = 12000;
	if (Freq < 500)
		Freq = 500;

	if (Sw > 4)
		Sw = 4;
	if (Pw > 4)
		Pw = 4;

	CmdCnv[3] = Freq;
	CmdCnv[4] = Freq >> 8;
	CmdCnv[5] = Sw << 4;
	CmdCnv[5] |= Pw;
	//  CalcCS(Data);

	unsigned char CS = 0;

	unsigned Len = CmdCnv[1];

	for (unsigned i = 0; i<Len - 1; i++)
		CS += CmdCnv[i];
	CmdCnv[Len - 1] = (~CS) + 1;
	cmdbuf[1] += Len;
	cmdbuf[3] = Len;
	memcpy(&cmdbuf[4], CmdCnv, Len);

	DWORD Snt = 0;

	if (!pUSBDev->OutSerial(cmdbuf, &Snt, 5)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	if (!Snt) {
		CurrError = "Not Ack Presel";
		return FALSE;
	}
	return TRUE;
}
/*
BOOL DLL_EXP  __stdcall Conv5_13GHz(DWORD Freq, UINT8 Sw, UINT8 Pw)
{
	unsigned char cmdbuf[32] = { 5,
		5,//Len
		9,//Cmd
		0 //Len Data
	};
	unsigned char CmdCnv[8] = { 5,
		7,//Len
		5,//Cmd
		0,//Freq_L
		0,//Freq_H
		0,//SW/PW
		0 //CS
	};

	if (Freq>12000)
		Freq = 12000;
	if (Freq < 500)
		Freq = 500;

	if (Sw > 4)
		Sw = 4;
	if (Pw > 4)
		Pw = 4;

	CmdCnv[3] = Freq;
	CmdCnv[4] = Freq >> 8;
	CmdCnv[5] = Sw << 4;
	CmdCnv[5] |= Pw;

	//  CalcCS(Data);

	unsigned char CS = 0;

	unsigned Len = CmdCnv[1];

	for (unsigned i = 0; i<Len - 1; i++)
		CS += CmdCnv[i];
	CmdCnv[Len - 1] = (~CS) + 1;
	cmdbuf[1] += Len;
	cmdbuf[3] = Len;
	memcpy(&cmdbuf[4], CmdCnv, Len);

	DWORD Snt = 0;
	if (!OutReceiver(cmdbuf, 5, &Snt)) {
		CurrError = (char *)pUSBDev->lastError();
		return FALSE;
	}
	if (!Snt) {
		CurrError = "Not Ack Presel";
		return FALSE;
	}
	return TRUE;
}
*/
//=============================================================
HANDLE __stdcall TRsJetSdr::SetModem(Demod_Str *pDemodulator) {

//Guard
	if (!pUSBDev) return nullptr;
	
	DemodulatorMgr *mgr = &rsGetApp().getDemodMgr();
	
	//terminate
	if (newDemod != nullptr) {
		//Delete demodulator
		mgr->setActiveDemodulator(nullptr, true);
		mgr->setActiveDemodulator(nullptr, false);
		newDemod->setActive(false);
		Sleep(10);
		mgr->deleteThread(newDemod);
		Sleep(10);
	}

	//terminate All
//	mgr->setActiveDemodulator(nullptr, false);
//	mgr->terminateAll();
	newDemod = nullptr;
	
	std::string type;
	std::string output_device = "";

	long sample_rate = pDemodulator->SampleRate;
	long long freq = pDemodulator->freq;

	rsGetApp().setSampleRate(sample_rate);

	FreqAttnStr m_FreqAttnStr;
	m_FreqAttnStr.freq = freq;
	m_FreqAttnStr.attnStr.mode = pDemodulator->attn.mode;
	m_FreqAttnStr.attnStr.att1 = pDemodulator->attn.att1;
	m_FreqAttnStr.attnStr.att2 = pDemodulator->attn.att2;
	m_FreqAttnStr.attnStr.att3 = pDemodulator->attn.att3;
	m_FreqAttnStr.attnStr.att_low = pDemodulator->attn.att_low;
	// set hardware
	rsGetApp().setFreqAttn(m_FreqAttnStr);

	switch (pDemodulator->Type) {   // legacy demod ID
		case 1: type =  "FM"; break;
		case 2: type ="NBFM"; break;
		case 3: type = "FMS"; break;
		case 4: type =  "AM"; break;
		case 5: type = "LSB"; break;
		case 6: type = "USB"; break;
		case 7: type = "DSB"; break;
		case 8: type = "I/Q"; break;
	default: 
				return nullptr;
	}

#if 1
	int bandwidth = pDemodulator->BandWidth;
	float squelch_level = -100.0;
	float gain = 1.0;
	rsGetApp().setSoloMode(false);

//create new demodulator	
		newDemod = mgr->newThread();


	newDemod->setDemodulatorType(type);
//	newDemod->setDemodulatorUserLabel(user_label);
//	newDemod->writeModemSettings(mSettings);
	newDemod->setBandwidth(bandwidth);
	newDemod->setFrequency(freq);
	newDemod->setGain(gain);
	newDemod->updateLabel(freq);
	newDemod->setMuted(false);
	newDemod->setDeltaLock(false);
	newDemod->setSquelchEnabled(false);
	newDemod->setSquelchLevel(squelch_level);

/*
//Attach to sound output:

	std::map<int, RtAudio::DeviceInfo>::iterator i;
			bool matching_device_found = false;
	for (i = mgr->outputDevices.begin(); i != mgr->outputDevices.end(); i++) {
		if (i->second.name == output_device) {
			newDemod->setOutputDevice(i->first);
			matching_device_found = true;
			break;
		}
	}
	//if no device is found, choose the first of the list anyway.
	if (!matching_device_found) {
		newDemod->setOutputDevice(mgr->outputDevices.begin()->first);
	}
*/

	newDemod->run();
	newDemod->setActive(true);
	//change  sdrPostThread
	rsGetApp().notifyDemodulatorsChanged();
	
	return (HANDLE)&newDemod;

#else
	bool isNew = /*shiftDown || */ (mgr->getLastActiveDemodulator() == nullptr)
		|| (mgr->getLastActiveDemodulator()
		&& !mgr->getLastActiveDemodulator()->isActive());
	
	DemodulatorInstancePtr demod = isNew ? nullptr : mgr->getLastActiveDemodulator();
	DemodulatorInstancePtr activeDemod = isNew ? nullptr : mgr->getActiveDemodulator();
/*
	if (pDemodulator->Type = ModemFM_Off) {
		wxGetApp().setSampleRate(pDemodulator->SampleRate);
		wxGetApp().setGain(pDemodulator->attn);
		wxGetApp().setFrequency(pDemodulator->freq);
	}
	else 
*/
	{
		isNew = true;
		demod = mgr->newThread();
//		demod->setFrequency(freq);

		wxGetApp().setFrequency(pDemodulator->freq);
		demod->setFrequency(pDemodulator->freq);

		demod->setDemodulatorType(mgr->getLastDemodulatorType());
		demod->setBandwidth(mgr->getLastBandwidth());
		demod->setSquelchLevel(mgr->getLastSquelchLevel());
		demod->setSquelchEnabled(mgr->isLastSquelchEnabled());
		demod->setGain(mgr->getLastGain());
		demod->setMuted(mgr->isLastMuted());
		
		if (mgr->getLastDeltaLock()) {
			demod->setDeltaLock(true);
			demod->setDeltaLockOfs(wxGetApp().getFrequency() - pDemodulator->freq);
		}
		else {
			demod->setDeltaLock(false);
		}
		demod->writeModemSettings(mgr->getLastModemSettings(mgr->getLastDemodulatorType()));
		demod->run();

		wxGetApp().notifyDemodulatorsChanged();

	//	DemodulatorThread::releaseSquelchLock(nullptr);
	}
	return 0;
#endif
	return nullptr;
}
float   __stdcall TRsJetSdr::GetAudioGain(HANDLE demod)
{
	if (!demod)
		return 0.0f;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!demod)
		return 0.0f;
	else
		return _demod->getGain();

}
//=================================================================
void  __stdcall TRsJetSdr::SetAudioGain(HANDLE demod, float gain)
 {
	if (!demod)
		return;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!_demod)
		return;
	else
		_demod->setGain(gain);
}
//=================================================================
int  __stdcall TRsJetSdr::GetBandwidth(HANDLE demod) {
	if (!demod)
		return 0;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!demod)
		return 0;
	else
		return _demod->getBandwidth();
}
//=================================================================
void  __stdcall TRsJetSdr::SetBandwidth(HANDLE demod, int bw) {
	if (!demod)
		return;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!_demod)
		return;
	else
		_demod->setBandwidth(bw);

}

//=================================================================
float __stdcall TRsJetSdr::GetSignalLevel(HANDLE demod) {
	if (!demod)
		return 0.0f;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!demod)
		return 0.0f;
	else
		return _demod->getSignalLevel();
}
//=================================================================
void __stdcall TRsJetSdr::SetAudioSampleRate(HANDLE demod, int sampleRate) {
	if (!demod)
		return ;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);
	if (!_demod)
		return;
	else
		_demod->setAudioSampleRate(sampleRate);
}

int __stdcall TRsJetSdr::GetAudioSampleRate(HANDLE demod) {
	if (!demod)
		return 0;
	DemodulatorInstancePtr _demod = *static_cast<DemodulatorInstancePtr*>(demod);

	if (!demod)
		return 0;
	else
		return _demod->getAudioSampleRate();
}
//====================================================
int   __stdcall TRsJetSdr::GetIQData(SdrIQData *iqData) {
	if(!pUSBDev)
		return 0;
	
	DemodulatorThreadInputQueuePtr 	pipe = rsGetApp().getIQVisualQueue();
	//DemodulatorThreadInputQueuePtr 	pipe = rsGetApp().getWaterfallVisualQueue();

//	if (pipe->empty())
		
	
	if (!pipe->pop(data_in, (50 * 1000))) {
		return 0;  //empty data .... continue
	}
	if (data_in && data_in->data.size()) {
		iqData->sampleRate = data_in->sampleRate;
		iqData->frequency = (double)data_in->frequency;
		iqData->data = (float *)&data_in->data[0];
		return data_in->data.size();
	}
	return 0;


}
//===============================================================
extern "C" HRESULT VCPP_API CreateRsJetSdr(RsJetSdr **pInterface)
{
	if (pUSBDev) return E_ACCESSDENIED;

	TRsJetSdr* pTRsJetSdr = new TRsJetSdr();

	*pInterface = pTRsJetSdr;

	if (*pInterface)
	{
		(*pInterface)->AddRef();
		return S_OK;
	}
	else return E_NOINTERFACE;
}

