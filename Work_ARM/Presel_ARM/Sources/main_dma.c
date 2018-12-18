/* ###################################################################
**     Filename    : main.c
**     Project     : Uart_DMA
**     Processor   : MK20DX128VLH5
**     Version     : Driver 01.01
**     Compiler    : GNU C Compiler
**     Date/Time   : 2016-09-28, 11:22, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.01
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */
/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "UART_Control.h"
#include "DMAT1.h"
#include "DMA1.h"
#include "IFsh1.h"
#include "IntFlashLdd1.h"
#include "SW_IF.h"
#include "SW_PRESEL.h"
#include "OUT_RFC.h"
#include "IN_RFC.h"
#include "CS_ATTN1.h"
#include "CS_ATTN2.h"
#include "CS_LO1.h"
#include "CS_LO2.h"
#include "LD1.h"
#include "LD2.h"
#include "PWM1.h"
#include "PwmLdd1.h"
#include "TU1.h"
#include "SPI_CLK.h"
#include "MOSI.h"
#include "AS1.h"
#include "ASerialLdd1.h"
/* Including shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
//=========================================
#include "const.h"
//#include "ADF4351.H"
#include "MAX2871.h"
#include <string.h>

/* User includes (#include below this line is not maintained by Processor Expert) */
void Exe_Cmd(byte *InputBuf);
word SearchIndex(word  toFind );
void Init_Receiver(pRecData pData);
void Set_Freq(pRecData pData,word Mode);
void Set_FreqNew(pRecData Data, word Mode);
void Echo_Out(byte *data);
void Set_Receiver(pReceiverStruct rStr);

void out_adf_IF1(dword data);
void out_adf_IF2(dword data);
void out_Atn1(word data);
void out_Atn2(word data);



enum { Pres_Gain_Off=0,Pres_Gain_On=1};
//---------------------------------------------
#define MAX_FLT 17
const int16_t mapSW[MAX_FLT][2]=
{
  {  20,1 /*0b0001 1101*/},//1
  {  36,2 /*0b1111 1101*/},//2
  {  52,3 /*0b0101 1101*/},//3
  {  84,4 /*0b1011 1101*/},//4
  { 132,5 /*0b1001 1101*/},//5
  { 228,6 /*0b0111 1101*/},//6
  { 372,7 /*0b1101 1101*/},//7
  { 612,8 /*0b0011 1101*/},//8
  {1044,9 /*0b0001 0110*/},//9
  {1444,10 /*0b0000 1110*/},//10
  {1844,11 /*0b0000 0110*/},//11
  {2244,12 /*0b0001 1010*/},//12
  {2644,13 /*0b0001 0010*/},//13
  {2996,14},
  {4500,15},
  {5012,16},
  {6000,16},
  {-1,0}
};

//#define Const_FLASH_ADDR (IntFlashLdd1_PFLASH_SIZE - IntFlashLdd1_ERASABLE_UNIT_SIZE)
__attribute__ ((aligned(IntFlashLdd1_ERASABLE_UNIT_SIZE))) const FLASHRAMDATA _FLASHDATA=
{
  0x8000, //50%
  16,
  {
  {  20,1,atn0,atn0,atn0},//1
  {  36,2,atn0,atn0,atn0},//2
  {  52,3,atn0,atn0,atn0},//3
  {  84,4,atn0,atn0,atn0},//4
  { 132,5,atn0,atn0,atn0},//5
  { 228,6,atn0,atn0,atn0},//6
  { 372,7,atn0,atn0,atn0},//7
  { 612,8,atn0,atn0,atn0},//8

  {1044,9,atn0,atn0,atn0},//9
  {1444,10,atn0,atn0,atn0},//10
  {1844,11,atn0,atn0,atn0},//11
  {2244,12,atn0,atn0,atn0},//12
  {2644,13,atn0,atn0,atn0},//13
  {2996,14,atn0,atn0,atn0},//14
  {4500,15,atn0,atn0,atn0},//15
  {5012,16,atn0,atn0,atn0},//16
  {6008,16,atn0,atn0,atn0},//16
  { 0,   0,   0,   0,   0}
  }  //End
};
volatile FLASHRAMDATA mRamData;
word DelayVal;

/*lint -save  -e970 Disable MISRA rule (6.3) checking. */
int main(void)
/*lint -restore Enable MISRA rule (6.3) checking. */
{
  /* Write your local variable definition here */
	LDD_TError Error;
	byte CmdBuf[32];
  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  if(!(SYST_CALIB & SysTick_CALIB_SKEW_MASK ))
		  AdjustValue = 50;
  else
	    AdjustValue = 50;

  //Init Ram_Data
  memcpy((byte *)&mRamData,(void *)&_FLASHDATA,sizeof(mRamData));

  pntTail=0;
  LDD_TDeviceData *MyDmaTransferPtr = DMAT1_Init(NULL);
  Error = DMAT1_AllocateChannel(MyDmaTransferPtr); /* Channel allocation */
  Error = DMAT1_EnableChannel(MyDmaTransferPtr); /* Channel enable */

  {
	  uint32_t i;	  for(i=0;i<100000;i++);
  }

  {
    RecData Data;
    Data.freq=200;
  	Data.att1=atn0;
  	Data.att2=atn2;
  	Data.att3=atn0;
  	Init_Receiver(&Data);
  }
//  SEGGER_RTT_WriteString(0, "Hello World from SEGGER!\r\n");
  PWM1_SetRatio16(mRamData.freq_pw);



  UART_Control_Init();
  /* Write your code here */
  /* For example: for(;;) { } */
	pIn=0;
  for(;;)
	  if(pntTail != ((DMA_TCD0_DADDR) & 0xFF))
	  {
		  CmdBuf[pIn++]=InBuffer[pntTail++];
		   		 if(pIn==1){
		   		  	if(CmdBuf[0]!=0x05)
		   		 	 				 pIn=0;
		   		 }else
		   		 if(pIn==CmdBuf[1])
		   		 {
		   			 byte cs=0;
		   			 do{cs+= CmdBuf[--pIn];
		   			 }while(pIn);
		    	 	if(!cs){
		    	 	 	 	Exe_Cmd(CmdBuf);
		    	 	}
		   		 }else
		   		 if(pIn == sizeof(CmdBuf))
		   				pIn=0;

	  }
  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** RTOS startup code. Macro PEX_RTOS_START is defined by the RTOS component. DON'T MODIFY THIS CODE!!! ***/
  #ifdef PEX_RTOS_START
    PEX_RTOS_START();                  /* Startup of the selected RTOS. Macro is defined by the RTOS component. */
  #endif
  /*** End of RTOS startup code.  ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
void Exe_Cmd(byte *InputBuf)
{
	word Len;
	RecData Data;
	ReceiverStruct mStr;

	switch(InputBuf[2]){

	case 0x40:  //Get Max  Freq
    	InputBuf[0]=5;
    	InputBuf[1]=6;
    	InputBuf[2]|=0x80;
    	InputBuf[3]=(byte)(6000);
    	InputBuf[4]=(byte)(6000>>8);
  	 	Echo_Out(InputBuf);
	break;


	case 0x30:
		mStr.IF1= (word)InputBuf[4]<<8 | InputBuf[3];
		mStr.IF2= (word)InputBuf[6]<<8 | InputBuf[5];
		mStr.Atn_In =63-(InputBuf[7]*2);
		mStr.Atn_Gain=63-(InputBuf[8]*2);
		mStr.Atn_Out = 63-(InputBuf[9]*2);
		mStr.Pres = InputBuf[10]+1;

		if(mStr.IF2<1500)
		  SW_IF_PutVal(0,IF_970);
		else
			  SW_IF_PutVal(0,IF_2140);

		Set_Receiver(&mStr);

		InputBuf[0]=5;
		InputBuf[1]=4; //size
		InputBuf[2]|=0x80;
	 	Echo_Out(InputBuf);

	break;


	case 0x20:	//Установка параметров преселектора
		  		//   0 - 0x5
		  		//   1 - Len
		  		//   2 - Cmd (0)
		  		// 3:4 - Freq MHz
		  		//   5  -mode
		  		//   6 - Atn
		  		//   7 - Atn2	(atn3)
		  		//   8 - Gain
		  		//   9 - CS
		  			Data.freq =(word)InputBuf[4]<<8 | InputBuf[3];
		  			Data.att1= (byte)(InputBuf[6] & 0x3F);
					Data.att2 =(byte)(InputBuf[7] & 0x3F);
					Data.att3 =(byte)(InputBuf[8] & 0x3F);
				//	InputBuf[5] & 0x80 ? SW_IF_ClrVal() : SW_IF_SetVal();   //Внешний выход OR //Внутренний выход
					Set_Freq(&Data,(word)(InputBuf[5]&3));
					InputBuf[0]=5;
					InputBuf[1]=4; //size
					InputBuf[2]|=0x80;
	      	 	Echo_Out(InputBuf);

	      	 	break;
	//============================================================
	case 0:	//Установка параметров преселектора
	 	  		//   0 - 0x5
	 	  		//   1 - Len
	 	  		//   2 - Cmd (0)
	 	  		// 3:4 - Freq MHz
	 	  		//   5 - Atn
	 	  		//   6 - Atn2	(atn3)
	 	  		//   7 - Gain
	 	  		//   8 - CS

	 				Data.freq = InputBuf[3] + InputBuf[4]*256;
	 	  			Data.att1 =(byte)(InputBuf[5] & 0x3F);
					Data.att2 =(byte)(InputBuf[6] & 0x3F);
					Data.att3 =(byte)(InputBuf[7] & 0x3F);
		        	Set_Freq(&Data,0);

		        	 	InputBuf[0]=5;
	    	    	 	InputBuf[1]=6; //size
	        		 	InputBuf[2]|=0x80;
	        		 	InputBuf[3]=(byte)(DelayVal);
	        	 		InputBuf[4]=(byte)(DelayVal>>8);
	        	 	Echo_Out(InputBuf);

	break;
	//============================================================
	case 1: //Установка скорости UART
	 	  		//   0 - 0x5
	 	  		//   1 - Len
	 	  		//   2 - Cmd (1)
	 	  		//	 3 - Mod
	 	  		//   4 - CS
	 	  		if(InputBuf[3]>2)
	 	  			Init_Serial(0);
	 	  		else
	 	  			Init_Serial(InputBuf[3]);
	break;
//============================================================
	case 2:	//Установка выхода сигнала
	 	  		//   0 - 0x5
	 	  		//   1 - Len
	 	  		//   2 - Cmd (2)
	 	  		//	 3 - Out
	 	  		//   4 - CS
     	 	InputBuf[0]=5;
     	 	InputBuf[1]=6;     //Size
   		 	InputBuf[2]|=0x80; //Echo
   		    InputBuf[3] = 0xff;
      	 	Echo_Out(InputBuf);

	break;
	//============================================================
	case 3:	//Чтение структуры
	  		//   0 - 0x5
 	  		//   1 - Len
 	  		//   2 - Cmd (3)
	  		//   3 - Номер входа в структуру
 	  		//   4 - CS

  		 	InputBuf[0]=5;
       	 	InputBuf[1]=5+sizeof(FREQSTRUCT);
       	 	InputBuf[2]|= 0x80; //Echo
       	 	if(InputBuf[3]<mRamData.size)
	        //	 	InputBuf[3]=; Текущая
       	 	memcpy(InputBuf+4,(void *)&mRamData.freq[InputBuf[3]],sizeof(FREQSTRUCT));
       	 	else {
           	 	InputBuf[1]=5;
           	    InputBuf[3]=0xFF;
       	 	}

       	 	Echo_Out(InputBuf);
	break;
  //============================================================
	case 4: //Запись структуры в RAM
	 	    	  		//   0 - 0x5
	 	    	  		//   1 - Len
	 	    	  		//   2 - Cmd (4)
	 	    	  		//   3 - Номер входа в структуру
	 	    	  		//   4:10
	 	    	  		//   11 - CS
	{
		uint16_t freq=(uint16_t)InputBuf[4] + (uint16_t)InputBuf[5] * 256;
		if(freq==0){
		 if(InputBuf[3]>=MAX_INDX)
					   mRamData.size=MAX_INDX;
		 else{
							mRamData.size=InputBuf[3];
							mRamData.freq[mRamData.size+1].freq=0;
		     }


		}else
		if(InputBuf[3]<MAX_INDX){
					int16_t index;

			for(index=0;;) //Search Pres Switc
			{
						if(mapSW[++index][0]==-1)
							                        break ;  //error

							if(freq < mapSW[index][0])
												    break;
			}
				InputBuf[6]=(uint8_t)(mapSW[--index][1]);

			    memcpy((void *)&mRamData.freq[InputBuf[3]],InputBuf+4,sizeof(FREQSTRUCT));


			}
			else
	 	   		   InputBuf[3]=0xFF; 	//	Ошибка

				   InputBuf[0]=5;
	 	       	   InputBuf[1]=5;
	 	           InputBuf[2]|=0x80;
	 	           Echo_Out(InputBuf);
	}
	break;
	//============================================================
	case 5: //Запись структуры во FLASH
	 	  		//   0 - 0x5
	 	  		//   1 - Len
	 	  		//   2 - Cmd (5)
	 	  		//   3 - CS
	   				/* Todo check
	   				 *
	   				 *
	   				 */
	 	  			for(Len=0;Len < MAX_INDX;Len++)
	   					if(!mRamData.freq[Len].freq){
	   						break ;  //error
	   					}


						mRamData.size=Len;
		 	  			IFsh1_EraseSector((uint32_t)&_FLASHDATA);
						Len = IFsh1_SetBlockFlash((byte *)&mRamData,(uint32_t)&_FLASHDATA,sizeof(mRamData));
						InputBuf[0]=5;
	 	        	 	InputBuf[1]=6;
	 	        	 	InputBuf[2]|=0x80;
	 	        	 	InputBuf[3]=Len;      //
	 	        	 	InputBuf[4]=Len>>8;      //
	 	                Echo_Out(InputBuf);

		break;
//=================================================================
  		case 6: //Установка частоты
		 	  		//   0 - 0x5
		 	  		//   1 - Len
		 	  		//   2 - Cmd (6)
		 	  		// 3:4 - Freq MHz
		 	  		//   5 - CS
		 	  			//Установка значений аттенюаторов в зависимости от частоты из структуры
	 				Data.freq = InputBuf[3] + InputBuf[4]*256;
	  			    Set_Freq(&Data,2);
		 	        InputBuf[0]=5;
		 	        InputBuf[1]=6;
		 	        InputBuf[2]|=0x80;
		 	        InputBuf[3]=(byte)(DelayVal);
		 	        InputBuf[4]=(byte)(DelayVal>>8);
		       	 	Echo_Out(InputBuf);
		break;
//============================================================
  		case 7: //Получить статус LD1,LD2,V_H,V_RSSI,Temp
		 	  		//   0 - 0x5
		 	  		//   1 - Len
		 	  		//   2 - Cmd (7)
		 	  		//   3 - CS
		 	        	 		InputBuf[0]=5;
		 	        	 		InputBuf[1]=4+5;
		 	        	 		InputBuf[2]|=0x80;
//		 	        	 		_memcpy(InputBuf+3,adc_data,5);
		 	        	 	Echo_Out(InputBuf);
	   break;
//======================================================================================
	   case 8: //Получить статус только LD1 & LD2
		 	  		//   0 - 0x5
		 	  		//   1 - Len
		 	  		//   2 - Cmd (8)
		 	  		//   3 - CS
		 	        	 		InputBuf[0]=   5;
		 	        	 		InputBuf[1]=   6;
		 	        	 		InputBuf[2]|=0x80;
//		 	        	 		_memcpy(InputBuf+3,adc_data,2);
		 	        	 	Echo_Out(InputBuf);
	 break;
//============================================================
  	 case 9: //Вывести блок по UART
			  			 	  			//   0 - 0x5
			  			 	  		//   1 - Len
			  			 	  		//   2 - Cmd (9)
			  			 	  		//   3 - Size
			  			 	  		//   4 -  data
			AS1_SendBlock(InputBuf+4,InputBuf[3],&Len);
			  			       	 		InputBuf[0]=   5;
			  			       	 		InputBuf[1]=   5; //SIze out
			  			       	 		InputBuf[2]|=0x80;
			  			       	 		InputBuf[3]=(byte)Len;

			  			       	 	Echo_Out(InputBuf);
	 break;
//============================================================
  	case 10: //Установка частоты & Atn1
	 		//   0 - 0x5
	  		//   1 - Len
	  		//   2 - Cmd (10)
	  		// 3:4 - Freq MHz
	  		//   5 - Atn
	  		//   6 - CS
			//Установка значений аттенюаторов в зависимости от частоты из структуры
  			Data.freq = InputBuf[3] + InputBuf[4]*256;
  		    Data.att1 =(byte)(InputBuf[5] & 0x3F);
            Set_Freq(&Data,1);
   	 		InputBuf[0]=5;
   	 		InputBuf[1]=6;
   	 		InputBuf[2]|=0x80;
   	 		InputBuf[3]=(byte)(DelayVal);
   	 		InputBuf[4]=(byte)(DelayVal>>8);
       	 	Echo_Out(InputBuf);
		 break;
		 //============================================================
  		case 11: //Чтение PW
		  	  		//   0 - 0x5
		  	  		//   1 - Len
		  	  		//   2 - Cmd (11)
		  	  		//   4 - CS
        	 		InputBuf[0]=5;
        	 		InputBuf[1]=6;
        	 		InputBuf[2]|=0x80;
        	 		InputBuf[3]=(byte)(mRamData.freq_pw);
        	 		InputBuf[4]=(byte)(mRamData.freq_pw>>8);
  	        	 	Echo_Out(InputBuf);
	  			 break;
		 //============================================================
	  		  case 12: //Запись PW в RAM
		  	  		//   0 - 0x5
		  	  		//   1 - Len
		  	  		//   2 - Cmd (12)
		  	  		//   3:4 - PW
		  	  		//   5 - CS
		 			mRamData.freq_pw=InputBuf[3]+((word)InputBuf[4]<<8);

		 	        	 	InputBuf[0]=5;
		     	    	 	InputBuf[1]=5;
		         	 		InputBuf[2]|=0x80;
		         	 		InputBuf[3] = PWM1_SetRatio16(mRamData.freq_pw);
		         	 	Echo_Out(InputBuf);
		  	  		break;
//=================================================================================
 		case 13: //Установка частоты & Atn1
		//   0 - 0x5
		//   1 - Len  8
		//   2 - Cmd (13)
		// 3:4 - Freq MHz
		//   5 - Atn
		//   6 - Atn
		//   7 - Atn
		//   8 - Sw  0 - Ext 1-Int
		//   9 - CS
			//Установка значений аттенюаторов в зависимости от частоты из структуры
			Data.freq = InputBuf[3] + InputBuf[4]*256;

			Data.att1 =(byte)Atn_tbl[InputBuf[5] & 0x1F];
			Data.att2 =(byte)Atn_tbl[InputBuf[6] & 0x1F];
			Data.att3 =(byte)Atn_tbl[InputBuf[7] & 0x1F];
/*
			if(InputBuf[8]==0) 	SW_IF_ClrVal();	//Внешний выход
			else                SW_IF_SetVal();	//Внутренний выход
*/
     	 Set_Freq(&Data,0);
     	 		InputBuf[0]=5;
     	 		InputBuf[1]=6;
     	 		InputBuf[2]|=0x80;
     	 		InputBuf[3]=(byte)(DelayVal);
     	 		InputBuf[4]=(byte)(DelayVal>>8);
   	 	Echo_Out(InputBuf);
	   break;

	}
	//	 Delay1mS(335);
//	 SPI0_PUSHR = 0xAAAA;
}
//=======================================================================================
void Echo_Out(byte *data)
{
	byte CS=0;
	word size=(word)(data[1]-1);
	while(size)
	{
		CS+=*data;
  		while(! (UART0_S1 & UART_S1_TDRE_MASK )){}
  		UART0_D = *data++;
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
      	size--;
 		//Delay
      	while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}

  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}
  		while(! (UART0_S1 & UART_S1_TC_MASK )){}


	}
  		//wait out Tx
	while(! (UART0_S1 & UART_S1_TDRE_MASK )){};
	UART0_D = (~CS+1); /* Store char to transmitter register */
}

//=================================================================

void Init_Receiver(pRecData Data)
{
dword regOut=R5_MAX2871(
		0,//VAS_DLY
		PLL_ON,//SDPLL
		AUTO_INT_ON,//F01
		LD_PIN_MODE_DLD,//LD
		MUX_OUT,   //MUXOUT
		0,//ADCS Normal Mode
		0 //ADCM ADC Disabled
		);

out_adf_IF1(regOut);
out_adf_IF2(regOut);
//============================================
regOut = R4_MAX2871(
	0,//SDLDO Enable LDO
	0,//SDDIV Enable N_DIV
	0,//SDREF Enable R_DIV
	520, //BS Band Select 50 KHz
	1,   //FB Fundamental
	0,//DIVA
	0,//SDVCO Enable VCO
	1,//MTLD
	0,//BDIV
	RF_OFF,//RFB_EN
	OUTPUT_POWER_M4DBM,//BPWR
	RF_ON,//RFA_EN
	OUTPUT_POWER_M4DBM //APWR
	);

out_adf_IF1(regOut);
out_adf_IF2(regOut);
//============================================

regOut =R3_MAX2871(
		0,			 //nVCO  Manual   Select VCO
	    VAS_EN,      //VCO Auto Select Enable
		VAS_TEMP_DI, //VAS temperature compensation
		CSM_OFF,		 //Cycle Slip Mode Disable
		0,//MUTEDEL
		0,//CDM
		1 //CDIV
		);

out_adf_IF1(regOut);
out_adf_IF2(regOut);
//============================================
 regOut =R2_MAX2871(
			0,//LDS
			LOW_NOISE_AND_SPUR_LOW_SPUR_2,//SDN
			MUX_OUT,//MUXOUT
			DOUBLER,//REFx2
			0,//RDIV2
			1,//R_DIV
			1,//D_BUF
			CHARGE_PUMP_CURRENT_5_12MA,//ICP
			MODE_FRAC,	//LDF
			0,	//LDP
			1,	//POL
			0,	//SHDN
			0,	//TRI
			0	//RST
			);

out_adf_IF1(regOut);
out_adf_IF2(regOut);
//============================================
regOut = R1_MAX2871(
	1,	//CPL
	0,	//CPT Normal Mode
	1,	//
	F_PFD	//MOD
	);
    out_adf_IF1(regOut);
	out_adf_IF2(regOut);

	 Set_Freq(Data,0);


}
//------------------------------------------------------------
void __attribute__((optimize("Os"))) Set_Freq(pRecData Data, word Mode)
{
 ReceiverStruct rStr;
 word index;

 word Freq=Data->freq;

 if(Freq < 20 )   Freq=20;
 if(Freq > 6008 ) Freq=6008;



 for(index=0;;) //Search Freq ROS
   {
   	if(!Rec_chan[++index][0])
   								return;  //error
   	if(Freq < Rec_chan[index][0])
   		break;
   }

 DelayVal=0;

 //Calculate ROS & CPLH
  rStr.IF2 = Fros[Rec_chan[--index][1]];


 //Calculate  CIF
  if(Rec_chan[index][1]>=8){
 		SW_IF_PutVal(0,IF_2140);
   		rStr.IF1 = Freq - rStr.IF2 - 140;

  }else
   if(Rec_chan[index][1]>=4)
     {

   		rStr.IF1 = Freq+rStr.IF2+140;
   		SW_IF_PutVal(0,IF_2140);
    }
   else  {
   	  		rStr.IF1=Freq+rStr.IF2-140;
   	   		SW_IF_PutVal(0,IF_970);

   	 }
 //-----------------------------------
   if(Freq<37)
 			index=0;
 else
  			index=SearchIndex(Freq );


    rStr.Pres=(word)mRamData.freq[index].kluch;
   // rStr.Pres=16;
    switch(Mode)
    {
    	case 0:
    		rStr.Atn_In=(word)Data->att1;
    		rStr.Atn_Out=(word)Data->att2;
    		rStr.Atn_Gain=(word)Data->att3;
    	break;
    	case 1:
    	 	rStr.Atn_In=(word)Data->att1;
    	 	rStr.Atn_Out=(word)mRamData.freq[index].att2;
    	 	rStr.Atn_Gain=(word)mRamData.freq[index].att3;
    	break;
    	//Установка значений аттенюаторов в зависимости от частоты из структуры
    	default :
    	 	rStr.Atn_In=(word)mRamData.freq[index].att1;
    	 	rStr.Atn_Out=(word)mRamData.freq[index].att2;
    	 	rStr.Atn_Gain=(word)mRamData.freq[index].att3;
    	break;

    }
    Set_Receiver(&rStr);

}
//===========================================================
void __attribute__((optimize("Os"))) Set_Receiver(pReceiverStruct rStr)
{

	static word tmp_IF1=0;
	static word tmp_IF2=0;
	static word Atn_In=0;
	static word Atn_Out=0;
	static word Atn_Gain=0xFFFF;
	static word Pres=0xFFFF;


   word pw,rf_div;
   word tmp = rStr->IF1;
   dword reg;

    if(tmp!=tmp_IF1) //New Value
    {
    	tmp_IF1 = tmp;
    	rf_div=0;

	while(tmp < 3000)
	{
		tmp<<=1;
		rf_div++;
	}

	reg = R4_MAX2871(
			0,//SDLDO Enable LDO
			0,//SDDIV Enable N_DIV
			0,//SDREF Enable R_DIV
			520, //BS Band Select 50 KHz
			1,   //FB Fundamental
			rf_div,//DIVA
			0,//SDVCO Enable VCO
			0,//MTLD
			0,//BDIV
			RF_OFF,//RFB_EN
			OUTPUT_POWER_M4DBM,//BPWR
			RF_ON,//RFA_EN
//			OUTPUT_POWER_M4DBM //APWR
			 OUTPUT_POWER_2DBM
			);

	out_adf_IF1(reg);

#if  (MODE_PLL == M_FRAC)
		reg = R0_MAX2871(MODE_FRAC, //if mode Frac
				tmp/F_PFD,//N_CNT
				tmp%F_PFD//FRAC
        		);

      out_adf_IF1(reg);

#else

      out_adf_IF1( R0_ADF4351(tmp,0));
#endif

    	if(!DelayVal) 		DelayVal=DELAY_PLL;
    }
//------------------------------------------------
    tmp = rStr->IF2;
    if(tmp!=tmp_IF2) //New Value
    {
		tmp_IF2=(word)tmp;
		rf_div=0;

	while(tmp<3000)
	{
		tmp<<=1;
		rf_div++;
	}

	reg =R4_MAX2871(
			0,//SDLDO Enable LDO
			0,//SDDIV Enable N_DIV
			0,//SDREF Enable R_DIV
			520, //BS Band Select 50 KHz
			1,   //FB Fundamental
			rf_div,//DIVA
			0,//SDVCO Enable VCO
			0,//MTLD
			0,//BDIV
			RF_OFF,//RFB_EN
			OUTPUT_POWER_M4DBM,//BPWR
			RF_ON,//RFA_EN
			//OUTPUT_POWER_M4DBM //APWR
			 OUTPUT_POWER_2DBM
			);
		out_adf_IF2(reg);


#if  (MODE_PLL == M_FRAC)
		reg =R0_MAX2871(MODE_FRAC, //if mode Frac
				tmp/F_PFD,//N_CNT
				tmp%F_PFD//FRAC
        		);
        out_adf_IF2(reg);
#else
      out_adf_IF2( R0_ADF4351(tmp,0));
#endif

    	if(!DelayVal)
       			DelayVal=DELAY_PLL;

    }
//------------------------------------------------
    if(rStr->Atn_In!=Atn_In)
    			out_Atn1(Atn_In=rStr->Atn_In);
//------------------------------------------------
   /*
    if(rStr->Atn_Out!=Atn_Out)
    	out_Atn2(Atn_Out=rStr->Atn_Out);
   */
//------------------------------------------------

    if(rStr->Atn_Gain!=Atn_Gain)
    	out_Atn2(Atn_Gain=rStr->Atn_Gain);

//------------------------------------------------
    if(rStr->Pres!=Pres)
    {
    	Pres=rStr->Pres;
    	SW_PRESEL_PutVal(NULL,mSw_Struct[Pres].SW_PRESEL);
    	IN_RFC_PutVal(NULL,mSw_Struct[Pres].SW_IN);
    	OUT_RFC_PutVal(NULL,mSw_Struct[Pres].SW_OUT);
    //	out_Pres(Pres);
    	if(!DelayVal)
    			DelayVal=DELAY_PLL;
    }
}
//====================================================================================
void __attribute__((optimize("Os"))) out_Atn1(word data)
{
//CS_ATTN1_ClrVal();
GPIO_PDD_ClearPortDataOutputMask(CS_ATTN1_MODULE_BASE_ADDRESS, CS_ATTN1_PORT_MASK);
uint16_t mask=0x20;

do{

	(mask & data) ? GPIO_PDD_SetPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK):
					GPIO_PDD_ClearPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK);

	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
//	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
	mask >>=1;
	GPIO_PDD_ClearPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
}while(mask);

//CS_ATTN1_SetVal();
GPIO_PDD_SetPortDataOutputMask(CS_ATTN1_MODULE_BASE_ADDRESS, CS_ATTN1_PORT_MASK);


}
//====================================================================================
void __attribute__((optimize("Os"))) out_Atn2(word data)
{
	//CS_ATTN2_ClrVal();
GPIO_PDD_ClearPortDataOutputMask(CS_ATTN2_MODULE_BASE_ADDRESS, CS_ATTN2_PORT_MASK);
uint16_t mask=0x20;

do{

    	(mask & data) ? GPIO_PDD_SetPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK):
    					GPIO_PDD_ClearPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK);

    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	//GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	mask >>=1;
    	GPIO_PDD_ClearPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    }while(mask);

//CS_ATTN2_SetVal();
GPIO_PDD_SetPortDataOutputMask(CS_ATTN2_MODULE_BASE_ADDRESS, CS_ATTN2_PORT_MASK);



}
//====================================================================================
void __attribute__((optimize("Os"))) out_adf_IF1(dword data)
{
	GPIO_PDD_ClearPortDataOutputMask(CS_LO1_MODULE_BASE_ADDRESS, CS_LO1_PORT_MASK);
	uint32_t mask=0x80000000;

	do{

    	(mask & data) ? GPIO_PDD_SetPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK):
    					GPIO_PDD_ClearPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK);

    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
       	mask >>=1;
    	GPIO_PDD_ClearPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);

    }while(mask);

	GPIO_PDD_SetPortDataOutputMask(CS_LO1_MODULE_BASE_ADDRESS, CS_LO1_PORT_MASK);

}
//====================================================================================
void __attribute__((optimize("Os"))) out_adf_IF2(dword data)
{
	GPIO_PDD_ClearPortDataOutputMask(CS_LO2_MODULE_BASE_ADDRESS, CS_LO2_PORT_MASK);
	uint32_t mask=0x80000000;

	do{

    	(mask & data) ? GPIO_PDD_SetPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK):
    					GPIO_PDD_ClearPortDataOutputMask(MOSI_MODULE_BASE_ADDRESS,MOSI_PORT_MASK);

    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	GPIO_PDD_SetPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);
    	mask >>=1;
    	GPIO_PDD_ClearPortDataOutputMask(SPI_CLK_MODULE_BASE_ADDRESS, SPI_CLK_PORT_MASK);

    }while(mask);

	GPIO_PDD_SetPortDataOutputMask(CS_LO2_MODULE_BASE_ADDRESS, CS_LO2_PORT_MASK);

}

//====================================================================================
word __attribute__((optimize("Os"))) SearchIndex(word  toFind )
{
	word    mid,m;
	word    low = 0;
	word    high =(int)( mRamData.size-1);
	word    l  = mRamData.freq[low].freq;
	word    h = mRamData.freq[high].freq;


    while (l <= toFind && h >= toFind) {
        mid = (low + high)/2;

         m = mRamData.freq[mid].freq;

        if (m < toFind) {
            l = mRamData.freq[low = mid + 1].freq;
        } else if (m > toFind) {
            h = mRamData.freq[high = mid - 1].freq;
        } else {
            return mid;
        }
    }

    if( toFind < l)
    					return low-1;
    else
     					return high;
}
//==========================================================
const uint8_t PrescUART[3][2] ={ //based 3.125
		 {27,4},
		 {3,4},
		 {2,16}
};
const uint8_t BaudUART[3][16] ={
			 {"Baud_115200\r\n"},
			 {"Baud_1000000\r\n"},
			 {"Baud_1250000\r\n"}

};
//=================================================
void Init_Serial(word speed)
{

 /* UART0_C2: TE=0,RE=0 */
  UART0_C2 &= (uint8_t)~(uint8_t)((UART_C2_RIE_MASK |UART_C2_TE_MASK | UART_C2_RE_MASK));
  UART0_BDH = UART_BDH_SBR(0x00);
   /* UART0_BDL: SBR=0x1B */
   UART0_BDL = UART_BDL_SBR(PrescUART[speed][0]);
   UART0_C4 = UART_C4_BRFA(PrescUART[speed][1]);
  UART0_C2 = (UART_C2_RIE_MASK | UART_C2_TE_MASK | UART_C2_RE_MASK);
//  SEGGER_RTT_WriteString(0, &BaudUART[speed]);

}
