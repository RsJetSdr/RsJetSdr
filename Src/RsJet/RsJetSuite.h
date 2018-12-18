#pragma once

#include <windows.h>
#include <commctrl.h>
#include <dbt.h>
#include <strsafe.h>
#include <iostream>
#include <clocale>


#ifdef _DLL_EXPORTS
#define VCPP_API __declspec(dllexport) __stdcall
#else
#define VCPP_API __declspec(dllimport) __stdcall
#endif

#define MIN_FREQ    20000000L
#define MAX_FREQ  3000000000L
#define LOW_FREQ  22000000.0

#define MIN_FREQ_MHz    	20L
#define MAX_FREQ_MHz  	  3000L
#define IF_FILTER_MHz       16L

#define IF_FILTER_HZ  16000000L	  //16.00 MHz
#define SAMPLERATE    20480000L   //20.48 MHz


//------ SampleRate;
typedef enum {
	    SR_160KHz = 160000,  //0.16   MHz
	    SR_320KHz = 320000,  //0.32   MHz
	    SR_640KHz = 640000,  //0.64   MHz
	  SR_1280KHz = 1280000,  //1.28   MHz
	  SR_2560KHz = 2560000,  //2.56   MHz
	  SR_5120KHz = 5120000,  //5.12   MHz
	SR_102400KHz = 10240000,  //10.24  MHz
	SR_204800KHz = 20480000   //20.48  MHz  base to Fast_Scan Mode
}Sample_Rate;

//---------- TYPE_WIN
typedef enum {
	Rectangular = 0, Hanning, Welch, Parzen, Bartlett, Hamming, Blacman2, Blacman3, Blacman4,
	Exponential, Riemann, BlackmanHarris
}Type_Win;



//---  Freq RESOLUTION,SampleRate = 20.48 MHz ; Fast_Scan Mode
typedef enum {
	Scan_80KHz = 256,
	Scan_40KHz = 512,
	Scan_20KHz = 1024,
	Scan_10KHz = 2048,
	Scan_5KHz = 4096
} FFT_SIZE;

//=====================================
#pragma pack(push,1)
typedef struct
{
	DWORD			FreqStart;		 // Начальная частота  MHz
	DWORD			FreqEnd;      	 // Конечная  частота  MHz
	Sample_Rate		SampleRate; // Частота выборки
	Type_Win		w_type;      // Тип окна
	FFT_SIZE		nFFT;
	//
	UCHAR  ModeAttn;
	UCHAR  AttnInput;   	//Значение Att в dB без знака 0-31
	UCHAR  AttnOutPres; 	//Значение Att в dB без знака 0-31
	UCHAR  AttnIF;      	//Значение Att в dB без знака 0-31


}ControlStruct, *pControlStruct;
#pragma pack(pop)
//=====================================
#pragma pack(push,1)
typedef struct
{
	short freq;
	UCHAR att1;
	UCHAR att2;
	UCHAR att3;
} FREQSTRUCT, *PFREQSTRUCT;
#pragma pack(pop)
//=====================================
typedef enum {
	ManualALL = 0, //Set All Struct
	ManualATT1,    //Set Attn1 Struct, Attn2 & Attn3 predefined(eeprom)
	AutoALL        //Set All  predefined (eeprom)
}Mode_ATT;

#pragma pack(push,1)
typedef struct
{
	Mode_ATT  mode;
	int  att1;
	int  att2;
	int  att3;
	int  att_low;
}AttnStr, *pAttnStr;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
	double freq;
	AttnStr attnStr;
}FreqAttnStr, *pFreqAttnStr;
#pragma pack(pop)

//------
typedef enum {
	Demod_Off = 0,
 //Analog
	DemodFM,
	DemodNBFM,
	DemodFMS,
	DemodAM,
	DemodLSB,
	DemodUSB,
	DemodDSB,
	DemodIQ
}Type_Demod;

#pragma pack(push,1)
typedef struct
{
 Type_Demod  Type;
    void *DemodPar;
    double freq;
	int  SampleRate;
	int  BandWidth; //max 500 Khz
	AttnStr attn;
 }Demod_Str, *pDemod_Str;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
	double frequency;
	int sampleRate;
	float *data;  //Complex
}SdrIQData, *pSdrIQData;
#pragma pack(pop)


//=====================================
//Error
#define SDR_TIMEOUT  (-1)
#define SDR_STREAM_ERROR (-2)
#define SDR_OVERFLOW (-4)
#define SDR_NOT_SUPPORTED (-5)
//Flags
#define SDR_MORE_FRAGMENTS (1 << 5)
//=====================================
typedef enum { 
	RX_FORMAT_CF32, //IQ Float32 0 (default)
	RX_FORMAT_CS16  //IQ Short16 1 (base RsJet) 
}RxFormat;
//=============================================
typedef enum {
	RAW=0,
	BURST
}modeStream;

//=====================================================================
struct __declspec(uuid("{9BBDA1A4-21E7-4D11-8F1C-E2AD13D2779C}"))
    RsJetSdr : public IUnknown
{
public:
//-------------------------------------------------------------- 
//Current function 
	virtual HANDLE __stdcall OpenUSBDevice(int DebugLevel,HANDLE hWnd, char **Str) = 0;
//------------------  fast spectrum ----------------------------
	virtual int  __stdcall  GetSpectr_10KHz(ControlStruct *Control) = 0;
	virtual DWORD  __stdcall  GetDataFFT(PUCHAR Data, DWORD Len) = 0;
	virtual void  __stdcall   StopSpectr() = 0;
 //-------------------------------------------------------------- 
 //Hardware  function
	virtual BOOL   __stdcall  GetPwPPM(int *Pw) = 0;
	virtual BOOL   __stdcall  SetPwPPM(int Pw) = 0;
	virtual BOOL   __stdcall  SetFreqGain(int index_in, FREQSTRUCT *FreqStr, UCHAR *Stat) = 0;
	virtual BOOL   __stdcall  GetFreqGain(int num, FREQSTRUCT *FreqStr) = 0;
	virtual BOOL   __stdcall  WriteFlash(UCHAR *Stat) = 0;
	virtual const   CHAR *  __stdcall  GetLastError(void) = 0;
	virtual BOOL __stdcall SetConv(DWORD Freq, UINT8 Sw, UINT8 Pw) = 0;
//
//>>>> Modem Function
	virtual int   __stdcall StartDevice(int SampleRate) = 0;
	virtual void  __stdcall StopDevice(int waitMsForTermination) = 0;

	virtual HANDLE __stdcall SetModem(Demod_Str *pDemodulator) = 0;
	virtual int   __stdcall GetIQData(SdrIQData *iqData)=0;

	virtual float   __stdcall GetAudioGain(HANDLE demod) = 0;
	virtual void  __stdcall SetAudioGain(HANDLE demod, float gain) = 0; //0.0 - 2.0 

	virtual float __stdcall GetSignalLevel(HANDLE demod)=0;
	virtual int __stdcall GetAudioSampleRate(HANDLE demod) = 0;
	virtual void __stdcall SetAudioSampleRate(HANDLE demod,int sampleRate) = 0;;

	virtual int  __stdcall GetBandwidth(HANDLE demod) = 0;
	virtual void  __stdcall SetBandwidth(HANDLE demod,int bw) = 0;




//=========================================================
	//Destructor
	//ULONG __stdcall Release(void);
};

extern "C" HRESULT VCPP_API CreateRsJetSdr(RsJetSdr **pInterface);



