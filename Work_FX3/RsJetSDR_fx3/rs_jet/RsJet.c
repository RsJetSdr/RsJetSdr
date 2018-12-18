
#include "RsJet.h"


Command_Task command_Host[MAX_TASK];
volatile uint32_t pnt_HeadCmd,pnt_TailCmd;
//=================================================
CyBool_t Out_CmdFPGA(Command_Task *pcmd){
	CyU3PReturnStatus_t status = CY_U3P_SUCCESS;
	CyBool_t notEmpty=0;

	if(pcmd->inData[0]==0)
	   CyU3PGpioSetValue(GPIO_SHB,(pcmd->inData[4] &((1<<3))));

	CyFxSpiTransfer(0,5,pcmd->inData, CyFalse); //Out to FPGA


	status = CyU3PGpioSimpleGetValue(GPIO_Empty, &notEmpty);
	if(status)
	return notEmpty;
	else return status;
}

volatile uint32_t Cnt;

#if 0

void Spectr_High(Command_Task *pcmd)
{
	uint8_t CmdRegRaw[5]={0};
	uint16_t ChanEnd;
	uint16_t SizeSample;
//	uint32_t Time;
	Freq_Attn mFreq_Attn;
	CyBool_t notEmpty=0;
/*
	uvint32_t *regPtr;
	uvint32_t regValOn,regValOff;
	regPtr = &GPIO->lpp_gpio_simple[GPIO_SHB];
	regValOff=(*regPtr & ~CY_U3P_LPP_GPIO_INTR);
	regValOff &= ~CY_U3P_LPP_GPIO_OUT_VALUE;
	regValOn = regValOff  | CY_U3P_LPP_GPIO_OUT_VALUE;
*/
	/* Flush the Endpoint memory */
	 CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

	mFreq_Attn.Freq =pcmd->inData[0] + pcmd->inData[1] * 256;
	mFreq_Attn.mode = pcmd->inData[6];
	mFreq_Attn.attn[0]= pcmd->inData[7];
	mFreq_Attn.attn[1]= pcmd->inData[8];
	mFreq_Attn.attn[2]= pcmd->inData[9];


	ChanEnd 	   = pcmd->inData[2] + pcmd->inData[3] * 256;

	SizeSample	= (pcmd->inData[4] + pcmd->inData[5] * 256) | MODE_BURST ;
	CmdRegRaw[0] =  MB_RAW_CTRL;
	CmdRegRaw[1] =  0;
	CmdRegRaw[2] =  0;
	CmdRegRaw[4] = (SizeSample & 0x000000FF);
	CmdRegRaw[3] = ((SizeSample & 0x0000FF00) >> 8);
      Set_Freq_Attn(&mFreq_Attn); //100 Us
	//First Delay
	  Delay_Us(300);
	 Cnt=0;

//==============================================================
while(pnt_TailCmd == pnt_HeadCmd){

	//Get current Sample
	CyFxSpiTransfer(0,5,CmdRegRaw, CyFalse);

	mFreq_Attn.Freq += 16;
	if(mFreq_Attn.Freq >= ChanEnd)
					 	 		 break;
	//Set New Freq
	Set_Freq_Attn(&mFreq_Attn); //100 Us

	//Wait buffer Empty
	do{//(Sync to HOST) min 100 uS
			CyU3PGpioSimpleGetValue(GPIO_Empty, &notEmpty);
	}while(notEmpty);

	 //Delay Next Task
	//*regPtr = regValOn;

			Delay_Us(150+100);

	// *regPtr = regValOff;
	//Timer_delay_uS(150);
}

	do{//Wait buffer Empty (Sync to PC)
		CyU3PGpioSimpleGetValue(GPIO_Empty, &notEmpty);
	}while(notEmpty);


}
#else
void Spectr_High(Command_Task *pcmd)
{
	uint8_t CmdRegRaw[5]={0};
	uint16_t ChanEnd;
	uint16_t SizeSample;
	uint32_t Time;
	Freq_Attn mFreq_Attn;
	CyBool_t notEmpty=0;
	/* Flush the Endpoint memory */
	 CyU3PUsbFlushEp(CY_FX_EP_CONSUMER);

	mFreq_Attn.Freq =pcmd->inData[0] + pcmd->inData[1] * 256;
	mFreq_Attn.mode = pcmd->inData[6];
	mFreq_Attn.attn[0]= pcmd->inData[7];
	mFreq_Attn.attn[1]= pcmd->inData[8];
	mFreq_Attn.attn[2]= pcmd->inData[9];


	ChanEnd 	   = pcmd->inData[2] + pcmd->inData[3] * 256;

	SizeSample	= (pcmd->inData[4] + pcmd->inData[5] * 256) | MODE_BURST ;
	CmdRegRaw[0] =  MB_RAW_CTRL;
	CmdRegRaw[1] =  0;
	CmdRegRaw[2] =  0;
	CmdRegRaw[4] = (SizeSample & 0x000000FF);
	CmdRegRaw[3] = ((SizeSample & 0x0000FF00) >> 8);

	 Set_Freq_Attn(&mFreq_Attn); //100 Us

	//First Delay
	// Timer_delay_uS(200);

	  for(Time=0;(Time<(_100us*2)) ;Time++)
	  {
			   if(pnt_TailCmd != pnt_HeadCmd)
				break;
	  }

	Cnt=0;
//==============================================================
while(pnt_TailCmd == pnt_HeadCmd){

	//Get current Sample
	CyFxSpiTransfer(0,5,CmdRegRaw, CyFalse);

	mFreq_Attn.Freq += 16;

	if(mFreq_Attn.Freq >= ChanEnd)
					 	 		 break;
	//Set New Freq
	Set_Freq_Attn(&mFreq_Attn); //100 Us

    //Delay Next Task
	for(Time=0;(Time<((_100us *2)) ) ;Time++){
			   if(pnt_TailCmd != pnt_HeadCmd)
				   	   	   	   	   	   	   break;
	 }
	//Wait buffer Empty
	do{//(Sync to HOST) min 100 uS
	    if(pnt_TailCmd != pnt_HeadCmd)
								  return;
		CyU3PGpioSimpleGetValue(GPIO_Empty, &notEmpty);
	}while(notEmpty);
}

	do{//Wait buffer Empty (Sync to PC)
		CyU3PGpioSimpleGetValue(GPIO_Empty, &notEmpty);
	}while(notEmpty);

}


#endif
