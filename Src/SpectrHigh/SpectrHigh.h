//---------------------------------------------------------------------------


#ifndef SpectrHighH
#define SpectrHighH

#include <windows.h>
#include <process.h>
#pragma hdrstop

//---------------------------------------------------------------------------
#include "rs_fft.h"
#include "Rs_FX3_USB.h"
#include "Rsjet\RsJetSuite.h"

#define QUEUE_SISE_BURST 8
#define MAX_EVENTS       3
#define TIME_OUT    1000

#pragma pack(push,1)
typedef struct{
	int StartFreq;  //
	int StopFreq;
	int Offset_Start;
	int Size_Stop;
	int Offset_FFT;
	int Size_data;
}TaskScan,*pTaskScan;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct{
	uint16_t StartFreq;  //
	uint16_t StopFreq;
	uint16_t Size;
	uint8_t attn[4];
}RecTask,*pRecTask;
#pragma pack(pop)

class SpectrHigh { //Sample Rate Constant = 20.48 Mhz

private:
 //	Async_Buf  Buffers[READ_BUFF_COUNT];
	FFT  		*pFFT;
	int 		Wtype;
	liquid_float_complex 	*in_cpx;
	float 		*bufWin;
	char *psdBm;
   //thread procedure
	static	unsigned __stdcall ThreadProc_Burst(LPVOID lParam);
protected:
 //	rsAlloc *mData;
	int SampleRate;
	int Size;
	float Gain_FFT;
	bool fStart;
	DWORD   dwThreadPri;
	USHORT StartFreq,StopFreq,BeginChan;
	USHORT Offset_FFT,Offset_Start,Size_Stop,Size_fft;
	bool MathSpectrum(CFrac16*	pbufOut);
	HANDLE hStartEvent;   //Wait Start Thread
	HANDLE hCloseEvent;   //Stop Thread
	HANDLE hReadEvent;    //Готовность Данных
	HANDLE hErrorEvent;   //Error Bus_USB
	bool Stop_thread;
	RecTask mRecTask;
	HANDLE hThread;
	UINT32 ThreadID;
	int Len_dbm;
	char * pData_DBm;
	UINT32 Index;

public:
	HANDLE hStartTask;
	HANDLE Event[2]; 	//ReadEvent and ErrorEvent;
	 SpectrHigh(ControlStruct *mControl);
	 ~SpectrHigh(void);
	bool Calculate_Start_Task(ControlStruct *mControl);
	int GetSize(){return Size;}
	int GetTypeWin(void){return Wtype;}
//	DWORD GetReadSize(void){return mData->GetSize();}
	DWORD GetReadSize(void){return Index;}

//	DWORD GetData(PUCHAR Data,DWORD Len){return mData->Read_Buf(Data, Len);}
	DWORD GetData(PUCHAR Data,DWORD Len){
	memcpy(Data,pData_DBm,Len);
	return Index;
	}

	int GetSamleRate(){return SampleRate;}
};
//extern SpectrHigh *mSpectrHigh;
#endif
