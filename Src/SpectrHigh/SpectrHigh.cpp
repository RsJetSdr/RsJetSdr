//---------------------------------------------------------------------------

#pragma hdrstop

#include "SpectrHigh.h"

//---------------------------------------------------------------------------
//===================================================================
bool  SpectrHigh:: Calculate_Start_Task(ControlStruct *mControl)
{
	int Bin_fft = SampleRate / Size; //-10 Khz
	int start_offset = (mControl->FreqStart - MIN_FREQ_MHz) % IF_FILTER_MHz;
	int stop_offset = (mControl->FreqEnd - MIN_FREQ_MHz) % IF_FILTER_MHz;
	int SizeBlk = 1000000 / Bin_fft;
	//Cтартовая центральная частота в MHz
	StartFreq = mControl->FreqStart - start_offset + IF_FILTER_MHz / 2;
	BeginChan = StartFreq;

	//Конечная центральная частота
	StopFreq = mControl->FreqEnd - stop_offset + IF_FILTER_MHz / 2;

	Offset_FFT = (SampleRate - IF_FILTER_HZ) / 2 / Bin_fft;
	Offset_Start = start_offset * 1000000L / Bin_fft;
	Size_Stop = 0;
	if (stop_offset)
		Size_Stop = (IF_FILTER_HZ - stop_offset * 1000000L) / Bin_fft;

	Size_fft = IF_FILTER_HZ / Bin_fft;

	//if(Size_Stop)   StopFreq +=IF_FILTER_MHz;

	 mRecTask.StartFreq = StartFreq;
	 if(stop_offset)
		mRecTask.StopFreq =StopFreq +16;
	 else
		mRecTask.StopFreq =StopFreq;

		mRecTask.Size = Size;
		mRecTask.attn[0]= 2-(mControl->ModeAttn & 0x03);
//===============================================================
		mRecTask.attn[1]= 63 -((mControl->AttnInput & 0x1F) * 2);
		mRecTask.attn[2]= 63 -((mControl->AttnOutPres  & 0x1F) * 2);
		mRecTask.attn[3]= 63 -((mControl->AttnIF & 0x1F) *  2);

 //===============================================================
		//old
	if(Len_dbm != ((mRecTask.StopFreq - mRecTask.StartFreq)  * SizeBlk))
		{
		//new
		 Len_dbm =  (mRecTask.StopFreq - mRecTask.StartFreq) * SizeBlk;
		  if(pData_DBm) delete[] pData_DBm;
			pData_DBm = new char[Len_dbm];

		}
		Index=0;    //clear index
/*
		std::cout << "Task FreqStart " << mRecTask.StartFreq << std::endl;
		std::cout << "Task FreqEnd " << mRecTask.StopFreq << std::endl;
		std::cout << "Task Size " << mRecTask.Size << std::endl;
*/

		SetEvent(hStartTask);
		Sleep(0);
	 return true;


}
//=====================================================================
SpectrHigh::SpectrHigh(ControlStruct *mControl)
		:SampleRate(20480000)   //constant
{
	Size   = mControl->nFFT;
	Wtype  = mControl->w_type;
	pFFT   = new FFT(Size,Wtype);
	in_cpx = pFFT->cpxbuf;
	bufWin = pFFT->pBufWin;
	psdBm = new char[Size];



	Gain_FFT = 10.0f * log10f((float)(Size/2)); //Gain FFT
   //	mData  = new rsAlloc();
	hThread = NULL;
	Len_dbm = 0;
	pData_DBm = NULL;
	//CalculateTask(mControl);

	//тип сброса TRUE - ручной
	// начальное состояние TRUE - сигнальное
	hStartEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
	hCloseEvent = CreateEvent(NULL,TRUE,FALSE,NULL);
	hStartTask  = CreateEvent(NULL,FALSE,FALSE,NULL);
	Event[0]=hReadEvent  = CreateEvent(NULL,FALSE,FALSE,NULL);
	Event[1]=hErrorEvent = CreateEvent(NULL,TRUE,FALSE,NULL);


	 fStart = false;
	 Stop_thread= false;
	dwThreadPri = GetThreadPriority(GetCurrentThread());
	 // Create the second thread.
	hThread = (HANDLE)_beginthreadex(
				NULL,
				0,
				&ThreadProc_Burst,
				this,
				0,
				&ThreadID );
	if(hThread){
	 WaitForSingleObject(hStartEvent, INFINITE);
	}
  CloseHandle(hStartEvent);
  Sleep(1);
  std::cout << "SpectrHigh::Start thread....." << std::endl;
  }
 //---------------------------------------------------------------------------
SpectrHigh::~SpectrHigh(void)
{
	Stop_thread= true;
	SetEvent(hCloseEvent);
	WaitForSingleObject( hThread, INFINITE );

	CloseHandle(hThread );
	CloseHandle(hCloseEvent);
	CloseHandle(hStartTask);
	CloseHandle(hReadEvent);
	CloseHandle(hErrorEvent);

	delete pFFT;   
		pFFT=NULL;
	if(pData_DBm) delete[] pData_DBm; 
		pData_DBm=NULL;
	delete [] psdBm ;
	std::cout << "SpectrHigh::thread done....." << std::endl;
}
//=============================================================================
bool SpectrHigh::MathSpectrum(CFrac16*	pbufOut)
{
	 float tmpfloat;
   //	 float _ofset=42;  //


	 if(BeginChan > StopFreq)
			return false;

					if(BeginChan >= 1444) //center

						for(int i=0;i<Size;i++){
							tmpfloat = bufWin[i];
							in_cpx[i].real =tmpfloat * pbufOut[i].real;
							in_cpx[i].imag =tmpfloat * pbufOut[i].imag;
						}

					else

						for(int i=0;i<Size;i++){
							tmpfloat = bufWin[i];
							in_cpx[i].imag =tmpfloat * pbufOut[i].real;
							in_cpx[i].real =tmpfloat * pbufOut[i].imag;
						}

						pFFT->DoFFTWMagnCh(Offset_FFT,
							(float)1.0e-13,   //-130 dB 1.55e-13 => -128 
							+2.0f, 	  //offset_fft,  =>-130 + 2 => -128
							psdBm);


					   if(BeginChan == StartFreq){
						memcpy(&pData_DBm[Index],&psdBm[Offset_Start],Size-(2 * Offset_FFT)-Offset_Start);
						Index+= Size-(2 * Offset_FFT)-Offset_Start;
					   //	mData->Write_Buf(&psdBm[Offset_Start],Size-(2 * Offset_FFT)-Offset_Start);
						}
						else
						if(BeginChan < StopFreq){
//						  mData->Write_Buf(psdBm,Size-(2 * Offset_FFT));
						memcpy(&pData_DBm[Index],psdBm,Size-(2 * Offset_FFT));
						Index+= Size-(2 * Offset_FFT);

					   }
						else
						if(BeginChan == StopFreq){
//						   mData->Write_Buf(psdBm,Size-(2 * Offset_FFT)-Size_Stop);
						memcpy(&pData_DBm[Index],psdBm,Size-(2 * Offset_FFT)-Size_Stop);
						Index+= Size-(2 * Offset_FFT)-Size_Stop;

						 }

						 BeginChan+=IF_FILTER_MHz;

						 if(BeginChan < StopFreq)
											return false;  //Wait


			 if((BeginChan == StopFreq) && (Size_Stop))
									return false; // Wait добрать остаток
						  else
									return true; //stop
}
//=============================================================================
unsigned __stdcall SpectrHigh::ThreadProc_Burst(LPVOID lParam)
{
	SpectrHigh *pThis = (SpectrHigh*)lParam;
	int Size 	= pThis->Size;         // Количество точек
	int 		totalTransferSize = Size * 4;  //16 I 16 Q
	long    	BytesXferred=0;
	long  		readLength=0;
	cmd_USB 	data_out={0};
	DWORD   	waitEvents;

	DWORD      		 nCount;//=MaxBUF;

	PUCHAR			buffers[QUEUE_SISE_BURST];
	PUCHAR			contexts[QUEUE_SISE_BURST];
	OVERLAPPED		 inOvLap[QUEUE_SISE_BURST];

	HANDLE inEvent[QUEUE_SISE_BURST]={NULL};
	HANDLE wait_Event[MAX_EVENTS]={NULL};


	//Status = GetThreadPriority(GetCurrentThread());

	if(!SetThreadPriority(GetCurrentThread(),
	THREAD_PRIORITY_NORMAL
   //	THREAD_PRIORITY_TIME_CRITICAL
	))
   {
	  //LastError =
	  GetLastError();
	  /*
	  if( ERROR_THREAD_MODE_ALREADY_BACKGROUND == LastError)
		 _tprintf(TEXT("Already in background mode\n"));
	  else _tprintf(TEXT("Failed to enter background mode (%d)\n"), dwError);
	  goto Cleanup;
	   */
   }

   pThis->dwThreadPri = GetThreadPriority(GetCurrentThread());

   pUSBDev->StopCommand();
	Sleep(50);
	pThis->fStart=false;
	CCyUSBEndPoint *eptIn = pUSBDev->StreameptIn;

	eptIn->Abort();
  //	eptIn->Reset();
	Sleep(50);
	eptIn->XferMode = XMODE_DIRECT;
	eptIn->SetXferSize(totalTransferSize);

	pUSBDev->SetFreqDDC(CENTER_FFT_140MHZ);  //Center  DDC
	pUSBDev->SetSampleRate(pThis->SampleRate); // Частота выборки


  // Allocate all the buffers for the queues
	for (nCount = 0; nCount < QUEUE_SISE_BURST; nCount++)
	{
		buffers[nCount]   = new UCHAR[totalTransferSize];
		memset(&inOvLap[nCount],0,sizeof(OVERLAPPED));
		inOvLap[nCount].hEvent = CreateEvent(NULL, false, false, NULL);
		inEvent[nCount] = inOvLap[nCount].hEvent;
		contexts[nCount] = NULL;
	}
	wait_Event[0] = inEvent[0];
	wait_Event[1] = pThis->hStartTask;
	wait_Event[2] = pThis->hCloseEvent;


	// Queue-up the first batch of transfer requests
	for (nCount = 0; nCount < QUEUE_SISE_BURST; nCount++) {
	contexts[nCount] = eptIn->BeginDataXfer(buffers[nCount], totalTransferSize, &inOvLap[nCount]);
	Sleep(0);
		/*
		if (eptIn->NtStatus || eptIn->UsbdStatus)
		 LastError =  eptIn->LastError;

								goto error;
	   */
	}

	SetEvent(pThis->hStartEvent);
	nCount=0;

	while(!pThis->Stop_thread){

		waitEvents = WaitForMultipleObjects(MAX_EVENTS,
											wait_Event,
											false,
											INFINITE);
	//input data
	if (waitEvents == 0)
	  {

		readLength = totalTransferSize;
		if(eptIn->FinishDataXfer(buffers[nCount],
								readLength,&inOvLap[nCount],
								contexts[nCount]))
		 {

		   BytesXferred += readLength;

				if((readLength == totalTransferSize) && (pThis->fStart)){

					if(pThis->MathSpectrum((CFrac16 *)buffers[nCount]))
					{
							pThis->fStart=false;
							SetEvent(pThis->hReadEvent);
					 }
				}

		  //Re-submit this queue element to keep the queue full
		  contexts[nCount] = eptIn->BeginDataXfer(
								buffers[nCount],
								totalTransferSize,
								&inOvLap[nCount]);

			 if (++nCount == QUEUE_SISE_BURST )
										nCount = 0;
			 wait_Event[0] = inEvent[nCount];  //wait next event
	  }
	  else{
		MessageBox(NULL,"Error FinishDataXfer","Wrong USB Driver",
		MB_ICONERROR | MB_SYSTEMMODAL | MB_OK);
		WaitForSingleObject(pThis->hCloseEvent, INFINITE);
		break;
	   }
   }else
		//New Task
    if(waitEvents == 1){ 
					BytesXferred=0;

					//===============================================
					data_out.wValue = 6; //Command
					data_out.wIndex = 0;
					data_out.Len = sizeof(RecTask);
					data_out.inData= (unsigned char *)&pThis->mRecTask;
					pThis->fStart=true;

					pUSBDev->Command(&data_out);


	   }   else
		//Close Event
	 if(waitEvents == 2){  
				break;
	 }
 } //=== end ====
	pUSBDev->StopCommand();
   eptIn->Abort();
   eptIn->TimeOut = 10000;   //restore default
   //eptIn->Reset();

   Sleep(100);
   for ( DWORD i=0; i< QUEUE_SISE_BURST; i++)
	{   //AbortXferLoop
		eptIn->WaitForXfer(&inOvLap[nCount], 100);
		readLength = totalTransferSize;

		eptIn->FinishDataXfer(buffers[nCount] ,readLength,
					 &inOvLap[nCount],
					  contexts[nCount]);
		BytesXferred += readLength;
		CloseHandle(inOvLap[nCount].hEvent);
		delete [] buffers[nCount];
		if (++nCount == QUEUE_SISE_BURST ) nCount = 0;
	}
	// SetEvent(pThis->hCloseEvent);
   _endthreadex( 0 );
	return 0;

   
}


